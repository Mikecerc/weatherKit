/**
 * @file task_common.h
 * @brief Common definitions and shared state for base station tasks
 * 
 * Contains semaphores, queues, and shared state that tasks need to access.
 */

#ifndef TASK_COMMON_H
#define TASK_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "ui_common.h"
#include "lora_protocol.h"

// =============================================================================
// Timeouts and Limits
// =============================================================================
#define SENSOR_TIMEOUT_MS       60000   // Consider sensor disconnected after 60s
#define ACK_ACK_TIMEOUT_MS      2000    // Resend ACK after 2 seconds
#define ACK_ACK_MAX_RETRIES     3       // Give up after 3 retries
#define CONFIG_RETRY_INTERVAL_MS 2000   // Retry config every 2s

// =============================================================================
// Shared Semaphores
// =============================================================================

/**
 * @brief Semaphore protecting LoRa radio access
 * Must be taken before any LoRa TX/RX operation
 */
extern SemaphoreHandle_t lora_mutex;

/**
 * @brief Semaphore protecting weather/lightning state
 */
extern SemaphoreHandle_t weather_state_mutex;

// =============================================================================
// Shared Queues
// =============================================================================

/**
 * @brief Queue for weather data updates from LoRa to UI
 */
extern QueueHandle_t weather_queue;

/**
 * @brief Queue for pending ACK retries
 */
extern QueueHandle_t ack_retry_queue;

/**
 * @brief Queue for config TX requests
 */
extern QueueHandle_t config_tx_queue;

// =============================================================================
// Pending Lightning State (for ACK-ACK three-way handshake)
// =============================================================================
typedef struct {
    uint32_t last_confirmed_total;   // Confirmed via ACK-ACK
    uint32_t pending_total;          // Waiting for ACK-ACK
    uint8_t pending_count;           // Strikes waiting to record
    float pending_closest;           // Closest distance for pending strikes
} lightning_state_t;

/**
 * @brief Get pointer to lightning state (protected by weather_state_mutex)
 */
lightning_state_t *task_get_lightning_state(void);

// =============================================================================
// Pending ACK State (for ACK retry)
// =============================================================================
typedef struct {
    uint8_t sequence;                // Sequence we're waiting ACK-ACK for
    uint8_t src_id;                  // Device we sent ACK to
    TickType_t sent_time;            // When we sent the ACK
    uint8_t retries;                 // Current retry count
    bool waiting;                    // Are we waiting for ACK-ACK?
    int8_t base_rssi;                // RSSI to include in ACK
} pending_ack_state_t;

/**
 * @brief Get pointer to pending ACK state (protected by lora_mutex)
 */
pending_ack_state_t *task_get_pending_ack_state(void);

// =============================================================================
// Sensor Connection Tracking
// =============================================================================

/**
 * @brief Update last sensor data receive time
 */
void task_update_sensor_time(void);

/**
 * @brief Check if sensor has timed out
 * @return true if sensor has timed out
 */
bool task_check_sensor_timeout(void);

/**
 * @brief Get time since last sensor data in ms
 */
uint32_t task_get_sensor_elapsed_ms(void);

// =============================================================================
// Initialization
// =============================================================================

/**
 * @brief Initialize shared task resources (semaphores, queues)
 * Must be called before starting any tasks
 */
esp_err_t task_common_init(void);

#endif // TASK_COMMON_H
