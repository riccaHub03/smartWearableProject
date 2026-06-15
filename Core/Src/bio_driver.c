#include "bio_driver.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes ---
static uint8_t bio_read_registers(uint8_t dev_i2c_addr, uint8_t reg_addr, uint8_t* data, uint16_t data_len);
static uint8_t bio_write_register(uint8_t dev_i2c_addr, uint8_t reg_addr, uint8_t reg_data);

// ==========================================
// FUNZIONI PUBBLICHE
// ==========================================

uint8_t BIO_Init(uint8_t ppg_addr, uint8_t temp_addr) {
    uint8_t part_id = 0;

    // Check MAX30101
    if (HAL_I2C_IsDeviceReady(&hi2c3, ppg_addr << 1, 3, BIO_I2C_TIMEOUT) != HAL_OK) {
        return 1; // Errore 1: MAX30101 non risponde
    }
    
    // Lettura Part ID (0xFF)
    if(bio_read_registers(ppg_addr, 0xFF, &part_id, 1)) {
        if(part_id != 0x15) {
            return 2; // Errore 2: Part ID MAX30101 errato
        }
    } else {
        return 2; // Errore 2: Impossibile leggere Part ID
    }

    // Check MAX30205
    if (HAL_I2C_IsDeviceReady(&hi2c3, temp_addr << 1, 3, BIO_I2C_TIMEOUT) != HAL_OK) {
        return 3; // Errore 3: MAX30205 non risponde
    }

    return 0; // Successo
}

/**
 * @brief Configura il MAX30101 in modalità SpO2 (Rosso + IR)
 */
uint8_t BIO_ConfigPPG(uint8_t dev_i2c_addr) {
    uint8_t reg_val = 0;
    
    // 1. Reset del sensore
    bio_write_register(dev_i2c_addr, MAX30101_MODE_CONFIG, 0x40);
    HAL_Delay(100); // Attesa completamento reset

    // 2. FIFO Config: Media di 1 sample (Nessuna media), rollover attivo
    bio_write_register(dev_i2c_addr, MAX30101_FIFO_CONFIG, 0x10);
    bio_read_registers(dev_i2c_addr, MAX30101_FIFO_CONFIG, &reg_val, 1);
    if(reg_val != 0x10) return 4; // Errore 4: Verifica configurazione fallita

    // 3. SpO2 Config: ADC range = 4096nA, Sample Rate = 100 Hz, Pulse Width = 411us (18-bit)
    bio_write_register(dev_i2c_addr, MAX30101_SPO2_CONFIG, 0x27);
    bio_read_registers(dev_i2c_addr, MAX30101_SPO2_CONFIG, &reg_val, 1);
    if(reg_val != 0x27) return 4;

    // 4. LED Pulse Amplitude (Corrente): Imposta a ~7mA (0x24) per entrambi
    bio_write_register(dev_i2c_addr, MAX30101_LED1_PA, 0x24); // Rosso
    bio_read_registers(dev_i2c_addr, MAX30101_LED1_PA, &reg_val, 1);
    if(reg_val != 0x24) return 4;
    
    bio_write_register(dev_i2c_addr, MAX30101_LED2_PA, 0x24); // IR
    bio_read_registers(dev_i2c_addr, MAX30101_LED2_PA, &reg_val, 1);
    if(reg_val != 0x24) return 4;

    // 5. Azzera i puntatori della FIFO prima di iniziare
    bio_write_register(dev_i2c_addr, MAX30101_FIFO_WR_PTR, 0x00);
    bio_write_register(dev_i2c_addr, MAX30101_OVF_COUNTER, 0x00);
    bio_write_register(dev_i2c_addr, MAX30101_FIFO_RD_PTR, 0x00);

    // 6. Avvia il sensore in modalità SpO2 (0x03)
    bio_write_register(dev_i2c_addr, MAX30101_MODE_CONFIG, 0x03);
    bio_read_registers(dev_i2c_addr, MAX30101_MODE_CONFIG, &reg_val, 1);
    if(reg_val != 0x03) return 4;
    
    return 0; // Successo
}

/**
 * @brief Configura il MAX30205 per la lettura continua della temperatura
 */
uint8_t BIO_ConfigTemp(uint8_t dev_i2c_addr) {
    uint8_t reg_val = 0xFF;
    // Il MAX30205 si accende in modalità normale. Assicuriamoci che il config sia a 0.
    bio_write_register(dev_i2c_addr, MAX30205_CONFIG_REG, 0x00);
    bio_read_registers(dev_i2c_addr, MAX30205_CONFIG_REG, &reg_val, 1);
    if(reg_val != 0x00) return 5; // Errore 5: Verifica configurazione temperatura fallita
    
    return 0; // Successo
}

/**
 * @brief Legge Rosso e Infrarosso dalla FIFO del MAX30101
 */
void BIO_ReadPPGData(PPG_Data *ppg_data, uint8_t *raw_data, uint8_t dev_i2c_addr) {
    // In modalità SpO2, ogni sample occupa 6 byte nella FIFO (3 per il Rosso, 3 per l'IR)
    bio_read_registers(dev_i2c_addr, MAX30101_FIFO_DATA, raw_data, 6);

    // I dati occupano 18 bit, quindi dobbiamo unire i 3 byte e mascherare con 0x03FFFF
    ppg_data->red = ((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) & 0x03FFFF;
    ppg_data->ir  = ((raw_data[3] << 16) | (raw_data[4] << 8) | raw_data[5]) & 0x03FFFF;
}

/**
 * @brief Legge e converte la temperatura del MAX30205
 */
void BIO_ReadTempData(Temp_Data *temp_data, uint8_t *raw_data, uint8_t dev_i2c_addr) {
    // Il sensore restituisce 2 byte: MSB e LSB
    bio_read_registers(dev_i2c_addr, MAX30205_TEMP_REG, raw_data, 2);

    // Ricostruiamo il valore raw a 16-bit (Complemento a 2)
    int16_t raw_temp = (int16_t)((raw_data[0] << 8) | raw_data[1]);

    // La sensibilità fissa da datasheet è 0.00390625 °C per bit
    temp_data->temperature_c = (float)raw_temp * 0.00390625f;
}

// ==========================================
// FUNZIONI PRIVATE (Helper I2C)
// ==========================================

static uint8_t bio_read_registers(uint8_t dev_i2c_addr, uint8_t reg_addr, uint8_t* data, uint16_t data_len) {
    // L'indirizzo a 7 bit viene shiftato a sinistra dall'HAL_I2C
    if (HAL_I2C_Master_Transmit(&hi2c3, dev_i2c_addr << 1, &reg_addr, 1, BIO_I2C_TIMEOUT) != HAL_OK) {
        return 0; 
    }
    if (HAL_I2C_Master_Receive(&hi2c3, dev_i2c_addr << 1, data, data_len, BIO_I2C_TIMEOUT) != HAL_OK) {
        return 0; 
    }
    return 1;
}

static uint8_t bio_write_register(uint8_t dev_i2c_addr, uint8_t reg_addr, uint8_t reg_data) {
    uint8_t tx_buffer[2] = {reg_addr, reg_data};
    if (HAL_I2C_Master_Transmit(&hi2c3, dev_i2c_addr << 1, tx_buffer, 2, BIO_I2C_TIMEOUT) != HAL_OK) {
        return 0; 
    }
    return 1; 
}