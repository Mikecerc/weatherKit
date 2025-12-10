/**
 * @file i2c_init.h
 * @brief I2C bus initialization for WeatherKit sensors
 */

#ifndef I2C_INIT_H
#define I2C_INIT_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "pinout.h"

/**
 * @brief Initialize the I2C bus for sensors
 * 
 * Sets up I2C master mode with the pins and frequency defined in pinout.h
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Deinitialize the I2C bus
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_bus_deinit(void);

/**
 * @brief Check if I2C bus is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool i2c_bus_is_initialized(void);

#endif // I2C_INIT_H