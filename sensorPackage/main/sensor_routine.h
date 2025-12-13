/**
 * @file sensor_routine.h
 * @brief Sensor routine for WeatherKit Sensor Package
 * 
 * Handles periodic sensor reading and transmission to base station.
 * Currently uses fake data - real sensor drivers to be added later.
 */

#ifndef SENSOR_ROUTINE_H
#define SENSOR_ROUTINE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// =============================================================================
// Configuration
// =============================================================================

/**
 * @brief Sensor routine configuration
 */
typedef struct {
    uint16_t update_interval_sec;       // Weather data send interval (seconds)
    uint16_t heartbeat_interval_sec;    // Status heartbeat interval (seconds)
    bool adaptive_power;                // Enable adaptive TX power
    bool use_fake_data;                 // Use fake sensor data (for testing)
} sensor_config_t;

/**
 * @brief Default sensor configuration
 */
#define SENSOR_CONFIG_DEFAULT() { \
    .update_interval_sec = 5, \
    .heartbeat_interval_sec = 300, \
    .adaptive_power = true, \
    .use_fake_data = true \
}

// =============================================================================
// API Functions
// =============================================================================

/**
 * @brief Initialize the sensor routine
 * 
 * Initializes LoRa module and sets up callbacks.
 * Must be called before sensor_routine_start().
 * 
 * @param config Pointer to configuration (NULL for defaults)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_routine_init(const sensor_config_t *config);

/**
 * @brief Start the sensor routine task
 * 
 * Creates a FreeRTOS task that periodically reads sensors
 * and transmits data to the base station.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_routine_start(void);

/**
 * @brief Stop the sensor routine task
 * 
 * Stops the sensor task and puts LoRa in sleep mode.
 */
void sensor_routine_stop(void);

/**
 * @brief Check if sensor routine is running
 * @return true if running
 */
bool sensor_routine_is_running(void);

/**
 * @brief Update the sensor configuration
 * 
 * Can be called to change intervals or settings at runtime.
 * 
 * @param config New configuration
 */
void sensor_routine_update_config(const sensor_config_t *config);

/**
 * @brief Get current sensor configuration
 * @param config Pointer to store configuration
 */
void sensor_routine_get_config(sensor_config_t *config);

/**
 * @brief Get uptime in minutes
 * @return Uptime in minutes (wraps at 65535)
 */
uint16_t sensor_routine_get_uptime_min(void);

/**
 * @brief Get packet statistics
 * @param sent Output: packets sent
 * @param acked Output: packets acknowledged
 * @param failed Output: packets failed/no ACK
 */
void sensor_routine_get_stats(uint32_t *sent, uint32_t *acked, uint32_t *failed);

#endif // SENSOR_ROUTINE_H
