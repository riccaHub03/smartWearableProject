#ifndef BIO_DSP2_H
#define BIO_DSP2_H

#include <stdint.h>
#include <stdbool.h>

/**
 * LIBRERIA BIO DSP 2 - REAL TIME (1:1 TRASLATION OF DATA_ANALYZER3.PY)
 * Questa versione replica fedelmente OGNI step del codice Python (incluso il Clipping Dinamico 
 * pre-filtraggio), rendendo causale (sample-by-sample) tutto ciò che era non-causale.
 * Aggiunge la media mobile esponenziale (EMA) per emulare il display dello Smartwatch.
 */

// Frequenza di campionamento fissa
#define FS_HZ 100.0f

// Soglie Sensor Fusion (Quality Assessment)
#define MIN_PI              0.02f
#define MAX_PI              2.0f    // Oltre questo valore sul nasello è impossibile, è sicuramente un artefatto
#define MIN_DC_LEVEL        10000.0f // Soglia di Prossimità (sotto la quale il sensore è in aria)
#define MAX_ACCEL_VAR       0.05f
#define HANGOVER_SAMPLES    (int)(FS_HZ * 2.0f) // 2 secondi di blocco dopo un artefatto

// Struttura che contiene i parametri vitali aggiornati in tempo reale
typedef struct {
    float heart_rate_bpm;      // HR Istantaneo (Reattivo, per uso clinico)
    float heart_rate_ema;      // HR Smussato (Per display Smartwatch)
    float spo2;                // Ossigenazione stimata (%)
    float perfusion_index;     // Qualità ottica del segnale (%)
    bool  signal_quality_good; // TRUE = Segnale affidabile, FALSE = Artefatto/Staccato
    bool  beat_detected;       // TRUE unicamente nell'istante in cui viene rilevato il picco sistolico
} BioState_t;

void BIO_DSP2_Init(void);
BioState_t BIO_DSP2_ProcessSample(float ppg_red, float ppg_ir, float acc_mag);

#endif // BIO_DSP2_H
