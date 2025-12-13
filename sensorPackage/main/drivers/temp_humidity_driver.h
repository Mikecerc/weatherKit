/**
 * @file temp_humidity_driver.h
 * @brief AHT20 temperature and humidity sensor driver
 * 
 * I2C-based temperature and humidity sensor
 * Provides high accuracy environmental measurements
 */

#ifndef TEMP_HUMIDITY_DRIVER_H
#define TEMP_HUMIDITY_DRIVER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief AHT20 sensor data structure
 */
typedef struct {
    float temperature_c;    // Temperature in Celsius
    float humidity_rh;      // Relative humidity in %
    bool valid;             // true if reading is valid
} aht20_data_t;

/**
 * @brief Initialize the AHT20 sensor
 * 
 * Note: I2C bus must be initialized first using i2c_bus_init()
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aht20_init(void);

/**
 * @brief Deinitialize the AHT20 sensor
 * 
 * Removes the device from I2C bus. Call before bus recovery.
 * 
 * @return ESP_OK on success
 */
esp_err_t aht20_deinit(void);

/**
 * @brief Re-initialize the AHT20 sensor
 * 
 * Performs deinit followed by init. Use after I2C bus recovery.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aht20_reinit(void);

/**
 * @brief Read temperature and humidity from AHT20
 * 
 * @param data Pointer to structure to store reading
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aht20_read(aht20_data_t *data);

/**
 * @brief Soft reset the AHT20 sensor
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aht20_reset(void);

/**
 * @brief Check if AHT20 is responding on I2C bus
 * 
 * @return true if sensor is detected, false otherwise
 */
bool aht20_is_present(void);

#endif // TEMP_HUMIDITY_DRIVER_H
