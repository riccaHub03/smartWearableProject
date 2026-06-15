#ifndef __BIO_DRIVER_H__
#define __BIO_DRIVER_H__

#include "stdint.h"
#include "main.h"

#define BIO_I2C_TIMEOUT 100 

// ==========================================
// REGISTRI MAX30205 (Temperatura)
// ==========================================
#define MAX30205_TEMP_REG       0x00 
#define MAX30205_CONFIG_REG     0x01 

// ==========================================
// REGISTRI MAX30101 (PPG / SpO2)
#define MAX30101_FIFO_WR_PTR    0x04
#define MAX30101_OVF_COUNTER    0x05
#define MAX30101_FIFO_RD_PTR    0x06
#define MAX30101_FIFO_DATA      0x07
#define MAX30101_FIFO_DATA      0x07
#define MAX30101_FIFO_CONFIG    0x08
#define MAX30101_MODE_CONFIG    0x09
#define MAX30101_SPO2_CONFIG    0x0A
#define MAX30101_LED1_PA        0x0C // Corrente LED Rosso
#define MAX30101_LED2_PA        0x0D // Corrente LED IR

// --- Strutture Dati Biomedicali ---
typedef struct {
    uint32_t red;   // Valore grezzo fotodiodo per LED Rosso (18-bit)
    uint32_t ir;    // Valore grezzo fotodiodo per LED Infrarosso (18-bit)
} PPG_Data;

typedef struct {
    float temperature_c; // Temperatura reale in Gradi Celsius
} Temp_Data;

// --- Public Function Prototypes ---
uint8_t BIO_Init(uint8_t ppg_addr, uint8_t temp_addr);
uint8_t BIO_ConfigPPG(uint8_t dev_i2c_addr);
uint8_t BIO_ConfigTemp(uint8_t dev_i2c_addr);
void BIO_ReadPPGData(PPG_Data *ppg_data, uint8_t *raw_data, uint8_t dev_i2c_addr);
void BIO_ReadTempData(Temp_Data *temp_data, uint8_t *raw_data, uint8_t dev_i2c_addr);

#endif /* __BIO_DRIVER_H__ */