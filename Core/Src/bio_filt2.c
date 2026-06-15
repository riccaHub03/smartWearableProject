#include "bio_filt2.h"
#include <math.h>

// ==========================================
// COSTANTI E DIMENSIONI BUFFER
// ==========================================
#define BASELINE_WIN  (int)(FS_HZ * 1.5f) // 150 campioni
#define CLIP_STD_WIN  (int)(FS_HZ * 2.0f) // 200 campioni (Calcolo STD per il clipping)
#define INT_WIN       (int)(FS_HZ * 0.15f) // 15 campioni per integrazione Pan-Tompkins
#define THRESH_WIN    (int)(FS_HZ * 1.5f) // 150 campioni per soglia adattiva
#define COOLDOWN_WIN  (int)(FS_HZ * 0.45f) // 45 campioni (450ms) refrattari post-battito (Anti-doppio picco)

#define DC_WIN        (int)(FS_HZ * 2.0f) // 200 campioni per componente DC
#define AC_WIN        (int)(FS_HZ * 1.5f) // 150 campioni per componente AC
#define ACC_WIN       (int)(FS_HZ * 1.0f) // 100 campioni per varianza IMU

// ==========================================
// VARIABILI DI STATO (MEMORIA GLOBALE DSP)
// ==========================================

// Struttura Biquad per il Filtro IIR
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} Biquad_t;

static Biquad_t ppg_filter_red[4];
static Biquad_t ppg_filter_ir[4];

// Buffer Circolari per Baseline (STEP 1)
static float baseline_red_buf[BASELINE_WIN];
static float baseline_ir_buf[BASELINE_WIN];
static int   baseline_idx = 0;
static float baseline_red_sum = 0.0f;
static float baseline_ir_sum = 0.0f;

// Buffer Circolari per Clipping Dinamico (STEP 2)
static float clip_red_sq_buf[CLIP_STD_WIN];
static float clip_ir_sq_buf[CLIP_STD_WIN];
static int   clip_idx = 0;
static float clip_red_sq_sum = 0.0f;
static float clip_ir_sq_sum = 0.0f;

// Buffer Circolari per Pan-Tompkins (usa il canale RED)
static float int_buf[INT_WIN];
static int   int_idx = 0;
static float int_sum = 0.0f;

static float thresh_buf[THRESH_WIN];
static int   thresh_idx = 0;
static float thresh_sum = 0.0f;

static float pt_hist[5] = {0.0f};
static int   cooldown_counter = 0;
static int   time_since_last_beat = 0;

typedef enum { WAITING_FOR_PEAK, IN_BEAT_RISE } PeakState_t;
static PeakState_t peak_state = WAITING_FOR_PEAK;

// Buffer veloci per SpO2, PI e IMU
static float dc_red_buf[DC_WIN];
static float dc_ir_buf[DC_WIN];
static int   dc_idx = 0;
static float dc_red_sum = 0.0f;
static float dc_ir_sum = 0.0f;

static float ac_red_buf[AC_WIN]; 
static float ac_ir_buf[AC_WIN];
static int   ac_idx = 0;
static float ac_red_sq_sum = 0.0f;
static float ac_ir_sq_sum = 0.0f;

static float acc_buf[ACC_WIN];
static int   acc_idx = 0;
static float acc_sum = 0.0f;
static float acc_sq_sum = 0.0f;

// Stato output e logica di controllo
static BioState_t current_state;
static const float EMA_ALPHA = 0.20f; // EMA reattiva (~5 battiti) poiché si aggiorna solo ad ogni battito
static int hangover_counter = 0;


// ==========================================
// FUNZIONI PRIVATE DSP
// ==========================================

static float process_biquad(float input, Biquad_t* bq) {
    float output = (bq->b0 * input) + (bq->b1 * bq->x1) + (bq->b2 * bq->x2) 
                 - (bq->a1 * bq->y1) - (bq->a2 * bq->y2);
    bq->x2 = bq->x1; bq->x1 = input;
    bq->y2 = bq->y1; bq->y1 = output;
    return output;
}

static void init_biquad_cascade(Biquad_t* cascade) {
    cascade[0].b0 = 0.000111f; cascade[0].b1 = 0.000223f; cascade[0].b2 = 0.000111f;
    cascade[0].a1 = -1.676695f; cascade[0].a2 = 0.710802f;
    
    cascade[1].b0 = 1.000000f; cascade[1].b1 = 2.000000f; cascade[1].b2 = 1.000000f;
    cascade[1].a1 = -1.806759f; cascade[1].a2 = 0.862590f;
    
    cascade[2].b0 = 1.000000f; cascade[2].b1 = -2.000000f; cascade[2].b2 = 1.000000f;
    cascade[2].a1 = -1.933393f; cascade[2].a2 = 0.934906f;
    
    cascade[3].b0 = 1.000000f; cascade[3].b1 = -2.000000f; cascade[3].b2 = 1.000000f;
    cascade[3].a1 = -1.979631f; cascade[3].a2 = 0.980657f;
    
    for(int i=0; i<4; i++) {
        cascade[i].x1 = 0; cascade[i].x2 = 0;
        cascade[i].y1 = 0; cascade[i].y2 = 0;
    }
}


// ==========================================
// INTERFACCIA PUBBLICA
// ==========================================

void BIO_DSP2_Init(void) {
    init_biquad_cascade(ppg_filter_red);
    init_biquad_cascade(ppg_filter_ir);
    
    for(int i=0; i<BASELINE_WIN; i++) { baseline_red_buf[i] = 0; baseline_ir_buf[i] = 0; }
    for(int i=0; i<CLIP_STD_WIN; i++) { clip_red_sq_buf[i] = 0; clip_ir_sq_buf[i] = 0; }
    for(int i=0; i<INT_WIN; i++) int_buf[i] = 0;
    for(int i=0; i<THRESH_WIN; i++) thresh_buf[i] = 0;
    for(int i=0; i<DC_WIN; i++) { dc_red_buf[i] = 0; dc_ir_buf[i] = 0; }
    for(int i=0; i<AC_WIN; i++) { ac_red_buf[i] = 0; ac_ir_buf[i] = 0; }
    for(int i=0; i<ACC_WIN; i++) acc_buf[i] = 0;
    
    current_state.heart_rate_bpm = 70.0f;
    current_state.heart_rate_ema = 70.0f;
    current_state.spo2 = 98.0f;
    current_state.perfusion_index = 0.0f;
    current_state.signal_quality_good = true;
    current_state.beat_detected = false;
    
    for(int i=0; i<5; i++) pt_hist[i] = 0.0f;
}


BioState_t BIO_DSP2_ProcessSample(float ppg_red, float ppg_ir, float acc_mag) {
    current_state.beat_detected = false;
    time_since_last_beat++;
    
    // ==========================================
    // 0. CALCOLO COMPONENTI DC (Sangue Statico)
    // ==========================================
    float old_dc_red = dc_red_buf[dc_idx];
    float old_dc_ir  = dc_ir_buf[dc_idx];
    dc_red_buf[dc_idx] = ppg_red;
    dc_ir_buf[dc_idx]  = ppg_ir;
    dc_red_sum = dc_red_sum - old_dc_red + ppg_red;
    dc_ir_sum  = dc_ir_sum - old_dc_ir + ppg_ir;
    dc_idx = (dc_idx + 1) % DC_WIN;
    
    float dc_red_val = dc_red_sum / (float)DC_WIN;
    float dc_ir_val  = dc_ir_sum / (float)DC_WIN;
    
    // ==========================================
    // 1. RIMOZIONE BASELINE (Wandering Baseline)
    // ==========================================
    float old_base_red = baseline_red_buf[baseline_idx];
    float old_base_ir  = baseline_ir_buf[baseline_idx];
    baseline_red_buf[baseline_idx] = ppg_red;
    baseline_ir_buf[baseline_idx]  = ppg_ir;
    baseline_red_sum = baseline_red_sum - old_base_red + ppg_red;
    baseline_ir_sum  = baseline_ir_sum - old_base_ir + ppg_ir;
    baseline_idx = (baseline_idx + 1) % BASELINE_WIN;
    
    float centered_red = ppg_red - (baseline_red_sum / (float)BASELINE_WIN);
    float centered_ir  = ppg_ir - (baseline_ir_sum / (float)BASELINE_WIN);

    // ==========================================
    // 2. CLIPPING DINAMICO DEGLI ARTEFATTI ESTREMI
    // ==========================================
    // Calcoliamo la deviazione standard del segnale centrato sugli ultimi 2 secondi
    float sq_centered_red = centered_red * centered_red;
    float sq_centered_ir  = centered_ir * centered_ir;
    
    clip_red_sq_sum = clip_red_sq_sum - clip_red_sq_buf[clip_idx] + sq_centered_red;
    clip_ir_sq_sum  = clip_ir_sq_sum - clip_ir_sq_buf[clip_idx] + sq_centered_ir;
    clip_red_sq_buf[clip_idx] = sq_centered_red;
    clip_ir_sq_buf[clip_idx]  = sq_centered_ir;
    clip_idx = (clip_idx + 1) % CLIP_STD_WIN;
    
    // Evitiamo clipping a zero se il segnale è piattissimo aggiungendo un piccolo bias
    float rolling_std_red = sqrtf(clip_red_sq_sum / (float)CLIP_STD_WIN) + 1.0f;
    float rolling_std_ir  = sqrtf(clip_ir_sq_sum / (float)CLIP_STD_WIN) + 1.0f;
    
    float bound_red = 2.5f * rolling_std_red;
    float bound_ir  = 2.5f * rolling_std_ir;
    
    float clipped_red = centered_red;
    if(clipped_red > bound_red) clipped_red = bound_red;
    if(clipped_red < -bound_red) clipped_red = -bound_red;
    
    float clipped_ir = centered_ir;
    if(clipped_ir > bound_ir) clipped_ir = bound_ir;
    if(clipped_ir < -bound_ir) clipped_ir = -bound_ir;

    // ==========================================
    // 3. FILTRAGGIO PASSA-BANDA (0.5 - 4.0 Hz)
    // ==========================================
    float filtered_red = clipped_red;
    float filtered_ir  = clipped_ir;
    for(int i=0; i<4; i++) {
        filtered_red = process_biquad(filtered_red, &ppg_filter_red[i]);
        filtered_ir  = process_biquad(filtered_ir, &ppg_filter_ir[i]);
    }
    filtered_red = -filtered_red;
    filtered_ir  = -filtered_ir;

    // ==========================================
    // 4. CALCOLO COMPONENTI AC E PI
    // ==========================================
    // La AC è la Standard Deviation del segnale filtrato
    float sq_filt_red = filtered_red * filtered_red;
    float sq_filt_ir  = filtered_ir * filtered_ir;
    
    float old_ac_red = ac_red_buf[ac_idx];
    float old_ac_ir  = ac_ir_buf[ac_idx];
    ac_red_buf[ac_idx] = sq_filt_red;
    ac_ir_buf[ac_idx]  = sq_filt_ir;
    ac_red_sq_sum = ac_red_sq_sum - old_ac_red + sq_filt_red;
    ac_ir_sq_sum  = ac_ir_sq_sum - old_ac_ir + sq_filt_ir;
    ac_idx = (ac_idx + 1) % AC_WIN;
    
    float ac_red_val = sqrtf(ac_red_sq_sum / (float)AC_WIN) * 2.828f; // Da RMS a Peak-Peak
    float ac_ir_val  = sqrtf(ac_ir_sq_sum / (float)AC_WIN) * 2.828f;
    
    // Perfusion Index (PI) sul canale IR (formula % AC/DC)
    if (dc_ir_val != 0.0f) {
        current_state.perfusion_index = (ac_ir_val / dc_ir_val) * 100.0f;
    }

    // ==========================================
    // 5. SENSOR FUSION (QUALITY ASSESSMENT)
    // ==========================================
    /* -- ACCELEROMETRO DISABILITATO TEMPORANEAMENTE --
    // Varianza della magnitudo accelerometro (per rilevare il moto)
    float old_acc = acc_buf[acc_idx];
    acc_buf[acc_idx] = acc_mag;
    acc_sum = acc_sum - old_acc + acc_mag;
    acc_sq_sum = acc_sq_sum - (old_acc * old_acc) + (acc_mag * acc_mag);
    acc_idx = (acc_idx + 1) % ACC_WIN;
    
    float acc_mean = acc_sum / (float)ACC_WIN;
    float acc_var = (acc_sq_sum / (float)ACC_WIN) - (acc_mean * acc_mean);
    */
    
    bool is_valid_now = true;
    // if(acc_var > MAX_ACCEL_VAR) is_valid_now = false; // Disabilitato
    
    // Manteniamo solo il controllo ottico sul Perfusion Index e sulla Prossimità
    if(dc_ir_val < MIN_DC_LEVEL) is_valid_now = false;
    else if(current_state.perfusion_index < MIN_PI || current_state.perfusion_index > MAX_PI) is_valid_now = false;
    
    // Effetto Hangover: blocca i calcoli per 2s dopo il disturbo
    if(!is_valid_now) {
        hangover_counter = HANGOVER_SAMPLES;
    } else if (hangover_counter > 0) {
        hangover_counter--;
    }
    
    current_state.signal_quality_good = (hangover_counter == 0);

    // ==========================================
    // 6. PAN-TOMPKINS: DERIVATA (5-Point), INTEGRAZIONE
    // ==========================================
    pt_hist[4] = pt_hist[3];
    pt_hist[3] = pt_hist[2];
    pt_hist[2] = pt_hist[1];
    pt_hist[1] = pt_hist[0];
    pt_hist[0] = filtered_red;
    
    // Filtro derivativo che attenua il rumore ad alta frequenza
    float diff = (2.0f * pt_hist[0] + pt_hist[1] - pt_hist[3] - 2.0f * pt_hist[4]) / 8.0f;
    
    float old_int_val = int_buf[int_idx];
    int_buf[int_idx] = diff * diff; // Quadrato
    int_sum = int_sum - old_int_val + int_buf[int_idx];
    int_idx = (int_idx + 1) % INT_WIN;
    
    float integrated = int_sum / (float)INT_WIN;
    
    float old_thresh_val = thresh_buf[thresh_idx];
    thresh_buf[thresh_idx] = integrated;
    thresh_sum = thresh_sum - old_thresh_val + integrated;
    thresh_idx = (thresh_idx + 1) % THRESH_WIN;
    
    float adaptive_thresh = (thresh_sum / (float)THRESH_WIN) * 0.85f;
    if (adaptive_thresh < 5.0f) adaptive_thresh = 5.0f; // Soglia minima assoluta anti-rumore
    
    // ==========================================
    // 7. RILEVAMENTO PICCO & CALCOLO SpO2
    // ==========================================
    if(cooldown_counter > 0) cooldown_counter--;
    
    if(peak_state == WAITING_FOR_PEAK) {
        if(integrated > adaptive_thresh && cooldown_counter == 0) {
            peak_state = IN_BEAT_RISE;
        }
    } 
    else if(peak_state == IN_BEAT_RISE) {
        int test_idx = thresh_idx - 2;
        if(test_idx < 0) test_idx += THRESH_WIN;
        
        // Pendenza negativa = Vertice dell'onda
        if(integrated < thresh_buf[test_idx]) {
            current_state.beat_detected = true;
            peak_state = WAITING_FOR_PEAK;
            cooldown_counter = COOLDOWN_WIN;
            
            float ibi_sec = (float)time_since_last_beat / FS_HZ;
            if(ibi_sec > 0.3f && ibi_sec < 1.5f) { // Range 40-200 BPM
                current_state.heart_rate_bpm = 60.0f / ibi_sec;
                
                // Aggiornamento solo se il segnale è pulito
                if(current_state.signal_quality_good) {
                    // EMA (Smussamento Smartwatch per il BPM)
                    current_state.heart_rate_ema = (current_state.heart_rate_bpm * EMA_ALPHA) + 
                                                   (current_state.heart_rate_ema * (1.0f - EMA_ALPHA));
                    
                    // Ratio of Ratios (SpO2)
                    if(dc_red_val != 0.0f && dc_ir_val != 0.0f) {
                        float r_ratio = (ac_red_val / dc_red_val) / (ac_ir_val / dc_ir_val);
                        float raw_spo2 = 110.0f - 25.0f * r_ratio;
                        if(raw_spo2 > 100.0f) raw_spo2 = 100.0f;
                        if(raw_spo2 < 80.0f) raw_spo2 = 80.0f;
                        
                        // EMA SpO2 per stabilità display
                        current_state.spo2 = (raw_spo2 * 0.1f) + (current_state.spo2 * 0.9f);
                    }
                }
            }
            time_since_last_beat = 0;
        }
    }
    
    return current_state;
}
