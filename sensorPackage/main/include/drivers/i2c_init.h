/**
 * @file i2c_init.h
 * @brief I2C bus initialization for WeatherKit sensors (new driver API)
 */

#ifndef I2C_INIT_H
#define I2C_INIT_H

#include "esp_err.h"
#include "driver/i2c_master.h"
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
 * @brief Check if I2C bus is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool i2c_bus_is_initialized(void);

/**
 * @brief Get the I2C bus handle for adding devices
 * 
 * @return I2C master bus handle, or NULL if not initialized
 */
i2c_master_bus_handle_t i2c_bus_get_handle(void);


/**
 * @brief Scan the I2C bus and log all detected devices
 * 
 * Useful for debugging I2C connection issues
 */
void i2c_bus_log_scan(void);

#endif // I2C_INIT_H