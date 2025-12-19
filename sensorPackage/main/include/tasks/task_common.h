/**
 * @file task_common.h
 * @brief Common definitions and shared state for sensor package tasks
 * 
 * This file defines shared resources (semaphores, queues) and state
 * that is accessed by multiple tasks in the sensor package.
 */

#ifndef TASK_COMMON_H
#define TASK_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "drivers/lora_protocol.h"

// =============================================================================
// Configuration Structure
// =============================================================================

/**
 * @brief Sensor routine configuration
 */
typedef struct {
    uint16_t update_interval_sec;       // Weather data send interval (seconds)
    bool adaptive_power;                // Enable adaptive TX power
    bool high_power;                    // Allow TX power above 2dBm
    bool use_fake_data;                 // Use fake sensor data (for testing)
} sensor_config_t;

/**
 * @brief Default sensor configuration
 */
#define SENSOR_CONFIG_DEFAULT() { \
    .update_interval_sec = 30, \
    .adaptive_power = true, \
    .high_power = false, \
    .use_fake_data = false \
}

// =============================================================================
// Shared Semaphores
// =============================================================================

/**
 * @brief Mutex for accessing sensor configuration
 */
extern SemaphoreHandle_t g_config_mutex;

/**
 * @brief Mutex for accessing LoRa hardware
 * Must be taken before any LoRa TX/RX operations
 */
extern SemaphoreHandle_t g_lora_mutex;

/**
 * @brief Mutex for accessing lightning data
 */
extern SemaphoreHandle_t g_lightning_mutex;

// =============================================================================
// Shared State
// =============================================================================

/**
 * @brief Current sensor configuration (protected by g_config_mutex)
 */
extern sensor_config_t g_current_config;

/**
 * @brief Accumulated lightning data (protected by g_lightning_mutex)
 */
extern lightning_data_t g_accumulated_lightning;

/**
 * @brief Last weather sequence number sent
 */
extern volatile uint8_t g_last_weather_sequence;

/**
 * @brief Whether we're waiting for weather ACK
 */
extern volatile bool g_awaiting_weather_ack;

/**
 * @brief Global running flag - when false, all tasks should exit
 */
extern volatile bool g_tasks_running;

/**
 * @brief Startup time in microseconds
 */
extern int64_t g_start_time_us;

/**
 * @brief Sensor error flags
 */
extern volatile uint8_t g_sensor_error_flags;

// =============================================================================
// LED Signal Flags (for LED status task)
// =============================================================================

extern volatile bool g_led_flash_packet_sent;
extern volatile bool g_led_flash_packet_failed;
extern volatile bool g_led_flash_packet_received;
extern volatile bool g_led_using_fake_data;

// =============================================================================
// Statistics
// =============================================================================

extern volatile uint32_t g_packets_sent;
extern volatile uint32_t g_packets_acked;
extern volatile uint32_t g_packets_failed;

// =============================================================================
// Functions
// =============================================================================

/**
 * @brief Initialize shared task resources (semaphores, queues)
 * @return ESP_OK on success
 */
esp_err_t task_common_init(void);

/**
 * @brief Deinitialize shared task resources
 */
void task_common_deinit(void);

/**
 * @brief Signal that a packet was sent successfully
 */
void task_signal_packet_sent(void);

/**
 * @brief Signal that a packet failed to send
 */
void task_signal_packet_failed(void);

/**
 * @brief Signal that a packet was received
 */
void task_signal_packet_received(void);

/**
 * @brief Get uptime in seconds
 * @return Uptime in seconds
 */
uint32_t task_get_uptime_sec(void);

/**
 * @brief Load sensor config from NVS flash
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t task_config_load(void);

/**
 * @brief Save current sensor config to NVS flash
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t task_config_save(void);

#endif // TASK_COMMON_H
