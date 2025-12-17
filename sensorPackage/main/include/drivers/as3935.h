/**
 * @file Lightning_driver.h
 * @brief 
 * 
 * I2C-based lightning detection sensor
 * Detects lightning strikes and estimates distance
 */

#ifndef LIGHTNING_DRIVER_H
#define LIGHTNING_DRIVER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief AS3935 interrupt event types
 */
typedef enum {
    AS3935_INT_NONE = 0,
    AS3935_INT_NOISE = 1,
    AS3935_INT_DISTURBER = 4,
    AS3935_INT_LIGHTNING = 8
} as3935_interrupt_t;

/**
 * @brief AS3935 sensor data structure
 */
typedef struct {
    uint8_t distance_km;        // Estimated distance to lightning (km)
    uint32_t energy;            // Lightning energy value
    as3935_interrupt_t event;   // Last event type
    uint32_t strike_count;      // Total strikes detected
    bool valid;                 // true if reading is valid
} as3935_data_t;

/**
 * @brief AS3935 configuration structure
 */
typedef struct {
    bool indoor;                // true for indoor mode, false for outdoor
    uint8_t noise_floor;        // Noise floor level (0-7, higher = less sensitive)
    uint8_t watchdog_threshold; // Spike rejection (0-15)
    uint8_t min_strikes;        // Minimum strikes before interrupt (1, 5, 9, 16)
} as3935_config_t;

/**
 * @brief Default AS3935 configuration
 */
#define AS3935_CONFIG_DEFAULT() { \
    .indoor = false, \
    .noise_floor = 2, \
    .watchdog_threshold = 2, \
    .min_strikes = 1 \
}

/**
 * @brief Initialize the AS3935 lightning sensor
 * 
 * Note: I2C bus must be initialized first using i2c_bus_init()
 * 
 * @param config Pointer to configuration (NULL for defaults)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_init(const as3935_config_t *config);

/**
 * @brief Read interrupt status and lightning data
 * 
 * Should be called when IRQ pin goes high
 * 
 * @param data Pointer to structure to store reading
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_read_interrupt(as3935_data_t *data);

/**
 * @brief Get current lightning statistics
 * 
 * @param data Pointer to structure to store statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_get_statistics(as3935_data_t *data);

/**
 * @brief Clear lightning statistics
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_clear_statistics(void);

/**
 * @brief Calibrate the internal oscillator
 * 
 * Should be called after initialization
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_calibrate(void);

/**
 * @brief Set indoor/outdoor mode
 * 
 * @param indoor true for indoor mode, false for outdoor
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_set_indoor(bool indoor);

/**
 * @brief Set noise floor level
 * 
 * @param level Noise floor level (0-7, higher = less sensitive)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_set_noise_floor(uint8_t level);

/**
 * @brief Power down the AS3935
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_power_down(void);

/**
 * @brief Power up the AS3935
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t as3935_power_up(void);

/**
 * @brief Check if AS3935 is responding on I2C bus
 * 
 * @return true if sensor is detected, false otherwise
 */
bool as3935_is_present(void);

/**
 * @brief Register interrupt callback for lightning events
 * 
 * @param callback Function to call when interrupt occurs
 */
void as3935_register_interrupt_callback(void (*callback)(as3935_interrupt_t event));

#endif // AS3935_DRIVER_H
