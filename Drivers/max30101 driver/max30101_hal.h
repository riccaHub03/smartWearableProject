/**
 * \file max30101_hal.h
 * \brief Driver library for the MAX30101 Heart Rate / SpO2 sensor.
 *
 * This header file provides the public interface for controlling the
 * MAX30101, including initialization and FIFO data reading functions.
 * The driver communicates via I2C using the same pattern as imu_driver.
 */

#ifndef MAX30101_HAL_H
#define MAX30101_HAL_H

#include "stdint.h"
#include "main.h"

// --- I2C Communication Settings ---
// The I2C slave address of the MAX30101 is 0x57 (7-bit format).
// The shift (<< 1) is applied at runtime inside the read/write functions,
// exactly like the IMU driver does.
#define MAX30101_I2C_ADDRESS    0x57
#define MAX30101_I2C_TIMEOUT    100     // Timeout in milliseconds for I2C operations

// --- Register Addresses ---
// These are the addresses of the key registers used to configure and read the MAX30101.
#define MAX30101_REG_INT_STS1       0x00
#define MAX30101_REG_INT_STS2       0x01
#define MAX30101_REG_INT_EN1        0x02
#define MAX30101_REG_INT_EN2        0x03
#define MAX30101_REG_FIFO_WR        0x04
#define MAX30101_REG_FIFO_OVF       0x05
#define MAX30101_REG_FIFO_RD        0x06
#define MAX30101_REG_FIFO_DATA      0x07
#define MAX30101_REG_FIFO_CFG       0x08
#define MAX30101_REG_MODE_CFG       0x09
#define MAX30101_REG_SPO2_CFG       0x0A
#define MAX30101_REG_LED1_PA        0x0C
#define MAX30101_REG_LED2_PA        0x0D
#define MAX30101_REG_LED3_PA        0x0E
#define MAX30101_REG_PILOT_PA       0x10
#define MAX30101_REG_MULTI_LED      0x11
#define MAX30101_REG_TINT           0x1F
#define MAX30101_REG_TFRAC          0x20
#define MAX30101_REG_TEMP_CFG       0x21
#define MAX30101_REG_PROX_INT       0x30
#define MAX30101_REG_REV_ID         0xFE
#define MAX30101_REG_PART_ID        0xFF

// --- Register Configuration Values ---
// Expected value returned by the PART_ID register to confirm sensor identity
#define MAX30101_PART_ID_VAL        0x15

// Mode Configuration Register bits
#define MAX30101_MODE_RESET_MASK    (1 << 6)    // Bit 6: software reset
#define MAX30101_MODE_HEART_RATE    0x02        // Heart Rate mode (Red LED only)
#define MAX30101_MODE_SPO2          0x03        // SpO2 mode (Red + IR LEDs)

// FIFO Configuration Register bits
#define MAX30101_FIFO_ROLLOVER_EN   (1 << 4)    // Bit 4: enable FIFO rollover
#define MAX30101_FIFO_SMP_AVE_2     (1 << 5)    // Bits 5-7: sample averaging = 2

// SpO2 Configuration Register composed value:
// ADC Range = 4096nA (01 << 5), Sample Rate = 100Hz (001 << 2), Pulse Width = 411us/18-bit (11 << 0)
#define MAX30101_SPO2_CFG_DEFAULT   0x27

// Default LED Pulse Amplitude (~7.2mA)
#define MAX30101_LED_PA_DEFAULT     0x24

// --- Data Constants ---
// Each channel sample is 3 bytes (18-bit ADC value)
#define MAX30101_BYTES_PER_CHANNEL  3
// In SPO2 mode there are 2 active channels (Red + IR)
#define MAX30101_SPO2_SAMPLE_SIZE   (2 * MAX30101_BYTES_PER_CHANNEL)
// Mask to extract the 18 valid data bits from the 3-byte sample
#define MAX30101_FIFO_DATA_MASK     0x03FFFF

// --- Public Function Prototypes ---

/**
 * @brief Initializes the MAX30101 sensor in SpO2 mode and verifies its presence via PART_ID check.
 * @return 1 on success (PART_ID matches and configuration written), 0 on failure.
 */
uint8_t MAX30101_Init(void);

/**
 * @brief Reads one sample (Red + IR) from the MAX30101 FIFO.
 * @param red Pointer to a variable where the 18-bit Red LED raw value will be stored.
 * @param ir  Pointer to a variable where the 18-bit IR LED raw value will be stored.
 * @return 1 on success, 0 on I2C communication error.
 */
uint8_t MAX30101_ReadFifo(uint32_t *red, uint32_t *ir);

#endif /* MAX30101_HAL_H */
