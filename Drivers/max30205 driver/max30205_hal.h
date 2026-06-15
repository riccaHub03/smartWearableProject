/**
 * \file max30205_hal.h
 * \brief Driver library for the MAX30205 body temperature sensor.
 *
 * This header file provides the public interface for controlling the
 * MAX30205, including initialization and temperature reading functions.
 * The driver communicates via I2C using the same pattern as imu_driver.
 */

#ifndef MAX30205_HAL_H
#define MAX30205_HAL_H

#include "stdint.h"
#include "main.h"

// --- I2C Communication Settings ---
// The I2C slave address of the MAX30205 depends on the A0, A1, A2 pin wiring.
// With all three pins connected to GND, the 7-bit address is 0x48.
// The shift (<< 1) is applied at runtime inside the read/write functions.
#define MAX30205_I2C_ADDRESS    0x48
#define MAX30205_I2C_TIMEOUT    100     // Timeout in milliseconds for I2C operations

// --- Register Addresses ---
// These are the addresses of the key registers inside the MAX30205.
#define MAX30205_REG_TEMPERATURE    0x00    // 16-bit temperature data (read-only)
#define MAX30205_REG_CONFIGURATION  0x01    // 8-bit configuration register
#define MAX30205_REG_THYST          0x02    // 16-bit hysteresis threshold
#define MAX30205_REG_TOS            0x03    // 16-bit overtemperature shutdown threshold

// --- Register Configuration Values ---
// Configuration register bit masks
#define MAX30205_CFG_SHUTDOWN       (1 << 0)    // Bit 0: shutdown mode (1 = one-shot, 0 = continuous)
#define MAX30205_CFG_COMP_INT       (1 << 1)    // Bit 1: comparator/interrupt mode
#define MAX30205_CFG_OS_POLARITY    (1 << 2)    // Bit 2: OS pin polarity
#define MAX30205_CFG_FAULT_QUEUE_1  (0 << 3)    // Bits 3-4: fault queue = 1
#define MAX30205_CFG_FAULT_QUEUE_2  (1 << 3)    // Bits 3-4: fault queue = 2
#define MAX30205_CFG_FAULT_QUEUE_4  (2 << 3)    // Bits 3-4: fault queue = 4
#define MAX30205_CFG_FAULT_QUEUE_6  (3 << 3)    // Bits 3-4: fault queue = 6
#define MAX30205_CFG_DATA_FORMAT    (1 << 5)    // Bit 5: data format (extended)
#define MAX30205_CFG_TIMEOUT_DIS    (1 << 6)    // Bit 6: bus timeout disable
#define MAX30205_CFG_ONESHOT        (1 << 7)    // Bit 7: one-shot trigger

// --- Data Constants ---
// Temperature resolution: 0.00390625 °C per LSB (16-bit, 0.00390625 = 1/256)
#define MAX30205_TEMP_RESOLUTION    0.00390625f

// --- Public Function Prototypes ---

/**
 * @brief Initializes the MAX30205 sensor in continuous conversion mode.
 * @return 1 on success (sensor responds on I2C), 0 on failure.
 */
uint8_t MAX30205_Init(void);

/**
 * @brief Reads a single temperature measurement from the MAX30205.
 * @param raw_temp Pointer to a variable where the raw 16-bit temperature will be stored.
 * @param temp_celsius Pointer to a variable where the converted temperature (°C) will be stored.
 * @return 1 on success, 0 on I2C communication error.
 */
uint8_t MAX30205_ReadTemperature(int16_t *raw_temp, float *temp_celsius);

#endif /* MAX30205_HAL_H */
