/**
 * \file max30101_hal.c
 * \brief Implementation file for the MAX30101 Heart Rate / SpO2 driver.
 *
 * This file contains the low-level functions for communicating with the MAX30101
 * sensor via I2C, including register reads/writes and FIFO data processing.
 * The communication pattern follows exactly the same style as imu_driver.c,
 * using HAL_I2C_Master_Transmit and HAL_I2C_Master_Receive.
 */

#include "max30101_hal.h"
#include "main.h"

// Reference to the I2C3 handle, defined in main.c by CubeMX.
// This is the same peripheral used by the IMU driver.
extern I2C_HandleTypeDef hi2c3;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t max30101_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len);
static uint8_t max30101_write_register(uint8_t reg_addr, uint8_t reg_data);

// --- Public Function Implementations ---

/**
 * @brief Initializes the MAX30101 sensor in SpO2 mode and verifies its identity.
 *
 * The initialization sequence is:
 * 1. Read PART_ID to confirm the sensor is a MAX30101
 * 2. Perform a software reset and wait for completion
 * 3. Configure FIFO, operating mode, SpO2 ADC, and LED currents
 * 4. Clear the FIFO pointers to start with a clean buffer
 *
 * @return 1 on success (WHO_AM_I matches), 0 on failure.
 */
uint8_t MAX30101_Init(void) {
    uint8_t part_id = 0;
    uint8_t mode_cfg = 0;

    // Step 1: Read the PART_ID register to verify the sensor identity.
    // The expected value is 0x15 as stated in the datasheet.
    if (!max30101_read_register(MAX30101_REG_PART_ID, &part_id, 1) || part_id != MAX30101_PART_ID_VAL) {
        return 0; // Sensor not found or wrong ID
    }

    // Step 2: Perform a software reset by setting the RESET bit in MODE_CFG.
    if (!max30101_write_register(MAX30101_REG_MODE_CFG, MAX30101_MODE_RESET_MASK)) {
        return 0; // Write failed
    }

    // Wait for the sensor to complete the reset. The RESET bit auto-clears when done.
    do {
        if (!max30101_read_register(MAX30101_REG_MODE_CFG, &mode_cfg, 1)) {
            return 0; // Read failed during reset polling
        }
    } while (mode_cfg & MAX30101_MODE_RESET_MASK);

    // Step 3a: Configure the FIFO behavior.
    // Enable rollover (overwrite old data when full) and average 2 samples to reduce noise.
    if (!max30101_write_register(MAX30101_REG_FIFO_CFG, MAX30101_FIFO_ROLLOVER_EN | MAX30101_FIFO_SMP_AVE_2)) {
        return 0;
    }

    // Step 3b: Set operating mode to SpO2 (Red + IR LEDs active alternately).
    if (!max30101_write_register(MAX30101_REG_MODE_CFG, MAX30101_MODE_SPO2)) {
        return 0;
    }

    // Step 3c: Configure the SpO2 ADC: range, sample rate, and pulse width.
    if (!max30101_write_register(MAX30101_REG_SPO2_CFG, MAX30101_SPO2_CFG_DEFAULT)) {
        return 0;
    }

    // Step 3d: Set the LED pulse amplitude (current) for Red and IR LEDs.
    if (!max30101_write_register(MAX30101_REG_LED1_PA, MAX30101_LED_PA_DEFAULT)) {
        return 0;
    }
    if (!max30101_write_register(MAX30101_REG_LED2_PA, MAX30101_LED_PA_DEFAULT)) {
        return 0;
    }

    // Step 4: Clear the FIFO read/write pointers and overflow counter
    // to ensure we start reading from a clean buffer.
    max30101_write_register(MAX30101_REG_FIFO_WR, 0x00);
    max30101_write_register(MAX30101_REG_FIFO_OVF, 0x00);
    max30101_write_register(MAX30101_REG_FIFO_RD, 0x00);

    return 1; // Initialization completed successfully
}

/**
 * @brief Reads one sample of raw data from the FIFO (SpO2 mode: Red + IR).
 *
 * In SpO2 mode each sample in the FIFO consists of 6 bytes:
 *   - Bytes 0-2: Red LED channel (18-bit ADC value)
 *   - Bytes 3-5: IR LED channel (18-bit ADC value)
 *
 * @param red Pointer where the 18-bit Red LED raw value will be stored.
 * @param ir  Pointer where the 18-bit IR LED raw value will be stored.
 * @return 1 on success, 0 on I2C communication error.
 */
uint8_t MAX30101_ReadFifo(uint32_t *red, uint32_t *ir) {
    uint8_t buffer[MAX30101_SPO2_SAMPLE_SIZE];

    // Read 6 consecutive bytes from the FIFO_DATA register.
    // The MAX30101 auto-increments the FIFO read pointer after each byte.
    if (!max30101_read_register(MAX30101_REG_FIFO_DATA, buffer, MAX30101_SPO2_SAMPLE_SIZE)) {
        return 0; // I2C read error
    }

    // Reconstruct the 18-bit Red LED value from the first 3 bytes,
    // masking out the upper 6 unused bits.
    *red = ((uint32_t)(buffer[0] << 16) | (buffer[1] << 8) | buffer[2]) & MAX30101_FIFO_DATA_MASK;

    // Reconstruct the 18-bit IR LED value from the next 3 bytes.
    *ir = ((uint32_t)(buffer[3] << 16) | (buffer[4] << 8) | buffer[5]) & MAX30101_FIFO_DATA_MASK;

    return 1; // Success
}

// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the MAX30101's register via I2C.
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
static uint8_t max30101_read_register(uint8_t reg_addr, uint8_t *data, uint16_t data_len) {
    // Send the register address to the sensor to prepare for reading.
    // The address is shifted left by 1 bit at runtime, exactly like the IMU driver.
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30101_I2C_ADDRESS << 1, &reg_addr, 1, MAX30101_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }

    // Read the requested number of bytes from the sensor.
    if (HAL_I2C_Master_Receive(&hi2c3, MAX30101_I2C_ADDRESS << 1, data, data_len, MAX30101_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}

/**
 * @brief Writes a single byte of data to a specific register on the MAX30101 via I2C.
 *
 * This function packs the register address and the data byte into a 2-byte
 * transmit buffer and sends it in a single I2C transaction, exactly like
 * the imu_write_register function does.
 *
 * @param reg_addr The address of the register to write to.
 * @param reg_data The byte of data to write.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t max30101_write_register(uint8_t reg_addr, uint8_t reg_data) {
    // Pack register address and data into a single transmit buffer.
    uint8_t tx_buffer[] = {reg_addr, reg_data};
    if (HAL_I2C_Master_Transmit(&hi2c3, MAX30101_I2C_ADDRESS << 1, tx_buffer, sizeof(tx_buffer), MAX30101_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}
