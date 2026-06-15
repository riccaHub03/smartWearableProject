/**
 * \file max30205_hal.c
 * \brief Implementation file for the MAX30205 body temperature sensor driver.
 *
 * This file contains the low-level functions for communicating with the MAX30205
 * sensor via I2C, including register reads/writes and temperature conversion.
 * The communication pattern follows exactly the same style as imu_driver.c,
 * using HAL_I2C_Master_Transmit and HAL_I2C_Master_Receive.
 */

#include "max30205_hal.h"
#include "main.h"

// Reference to the I2C3 handle, defined in main.c by CubeMX.
// This is the same peripheral used by the IMU driver.
extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t max30205_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len);
static uint8_t max30205_write_register(uint8_t reg_addr, uint8_t reg_data);

// --- Public Function Implementations ---

/**
 * @brief Initializes the MAX30205 sensor in continuous conversion mode.
 *
 * The initialization sequence is:
 * 1. Try to read the configuration register to verify sensor presence on I2C
 * 2. Write 0x00 to the configuration register to ensure continuous mode
 *    (all bits cleared = continuous conversion, comparator mode, OS active low)
 *
 * @return 1 on success, 0 on failure.
 */
uint8_t MAX30205_Init(void) {
    uint8_t config_value = 0;

    // Step 1: Read the configuration register to verify the sensor is present on the I2C bus.
    // Unlike the MAX30101, the MAX30205 does not have a WHO_AM_I / PART_ID register,
    // so we verify its presence by attempting to read the configuration register.
    if (!max30205_read_register(MAX30205_REG_CONFIGURATION, &config_value, 1)) {
        return 0; // Sensor not responding on I2C
    }

    // Step 2: Write 0x00 to the configuration register to set continuous conversion mode.
    // This clears all bits: shutdown=0, comparator mode, OS active low, fault queue=1.
    if (!max30205_write_register(MAX30205_REG_CONFIGURATION, 0x00)) {
        return 0; // Write failed
    }

    return 1; // Initialization completed successfully
}

/**
 * @brief Reads a single temperature measurement from the MAX30205.
 *
 * The temperature register (0x00) contains a 16-bit signed value.
 * The conversion to Celsius is: temperature = raw_value * 0.00390625
 * where 0.00390625 = 1/256 (the resolution is 1/256 °C per LSB).
 *
 * @param raw_temp Pointer where the raw 16-bit signed temperature will be stored.
 * @param temp_celsius Pointer where the converted temperature in °C will be stored.
 * @return 1 on success, 0 on I2C communication error.
 */
uint8_t MAX30205_ReadTemperature(int16_t *raw_temp, float *temp_celsius) {
    uint8_t buffer[2];

    // Read 2 bytes from the temperature register (MSB first, then LSB).
    if (!max30205_read_register(MAX30205_REG_TEMPERATURE, buffer, 2)) {
        return 0; // I2C read error
    }

    // Reconstruct the signed 16-bit raw temperature value.
    // The MAX30205 sends MSB first (buffer[0]) then LSB (buffer[1]).
    *raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);

    // Convert the raw value to degrees Celsius.
    // Each LSB represents 0.00390625 °C (= 1/256).
    *temp_celsius = (float)(*raw_temp) * MAX30205_TEMP_RESOLUTION;

    return 1; // Success
}

// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the MAX30205's register via I2C.
 *
 * This is a low-level function that encapsulates the I2C read operation.
 * It follows the same two-step pattern used in imu_driver.c:
 * first a Transmit to send the register address, then a Receive to read the data.
 *
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param data_len The number of bytes to read.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t max30205_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len) {
    // Send the register address to the sensor to prepare for reading.
    // The address is shifted left by 1 bit at runtime, exactly like the IMU driver.
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30205_I2C_ADDRESS << 1, &reg_addr, 1, MAX30205_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }

    // Read the requested number of bytes from the sensor.
    if (HAL_I2C_Master_Receive(&hi2c3, MAX30205_I2C_ADDRESS << 1, data, data_len, MAX30205_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}

/**
 * @brief Writes a single byte of data to a specific register on the MAX30205 via I2C.
 *
 * This function packs the register address and the data byte into a 2-byte
 * transmit buffer and sends it in a single I2C transaction, exactly like
 * the imu_write_register function does.
 *
 * @param reg_addr The address of the register to write to.
 * @param reg_data The byte of data to write.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t max30205_write_register(uint8_t reg_addr, uint8_t reg_data) {
    // Pack register address and data into a single transmit buffer.
    uint8_t tx_buffer[] = {reg_addr, reg_data};
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30205_I2C_ADDRESS << 1, tx_buffer, sizeof(tx_buffer), MAX30205_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}
