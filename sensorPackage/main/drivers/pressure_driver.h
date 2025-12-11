/**
 * @file hx710b_driver.h
 * @brief HX710B barometric pressure sensor driver
 * 
 * Uses bit-banging protocol with CLK and DOUT pins
 * Measures atmospheric pressure for weather monitoring
 */

#ifndef HX710B_DRIVER_H
#define HX710B_DRIVER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief HX710B sensor data structure
 */
typedef struct {
    float pressure_hpa;     // Pressure in hectopascals (hPa/mbar)
    int32_t raw_value;      // Raw ADC value
    bool valid;             // true if reading is valid
} hx710b_data_t;

/**
 * @brief Initialize the HX710B sensor
 * 
 * Sets up GPIO pins for CLK and DOUT
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t hx710b_init(void);

/**
 * @brief Read pressure from HX710B
 * 
 * Performs bit-banging protocol to read 24-bit ADC value
 * and converts to pressure in hPa
 * 
 * @param data Pointer to structure to store reading
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t hx710b_read(hx710b_data_t *data);

/**
 * @brief Check if HX710B is ready (DOUT pin is low)
 * 
 * @return true if ready to read, false otherwise
 */
bool hx710b_is_ready(void);

/**
 * @brief Power down the HX710B (low power mode)
 * 
 * @return ESP_OK on success
 */
esp_err_t hx710b_power_down(void);

/**
 * @brief Wake up the HX710B from power down
 * 
 * @return ESP_OK on success
 */
esp_err_t hx710b_power_up(void);

/**
 * @brief Set calibration offset for pressure readings
 * 
 * @param offset_hpa Offset to add to raw readings (can be negative)
 */
void hx710b_set_offset(float offset_hpa);

#endif // HX710B_DRIVER_H