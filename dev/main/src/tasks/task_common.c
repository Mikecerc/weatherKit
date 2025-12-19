/**
 * @file task_common.c
 * @brief Common shared state and initialization for base station tasks
 */

#include "task_common.h"
#include "esp_log.h"

static const char *TAG = "task_common";

// =============================================================================
// Semaphores
// =============================================================================
SemaphoreHandle_t lora_mutex = NULL;
SemaphoreHandle_t weather_state_mutex = NULL;

// =============================================================================
// Queues
// =============================================================================
QueueHandle_t weather_queue = NULL;
QueueHandle_t ack_retry_queue = NULL;
QueueHandle_t config_tx_queue = NULL;

// =============================================================================
// Shared State
// =============================================================================
static lightning_state_t lightning_state = {
    .last_confirmed_total = 0,
    .pending_total = 0,
    .pending_count = 0,
    .pending_closest = -1.0f
};

static pending_ack_state_t pending_ack_state = {
    .sequence = 0,
    .src_id = 0,
    .sent_time = 0,
    .retries = 0,
    .waiting = false,
    .base_rssi = 0
};

static TickType_t last_sensor_data_time = 0;

// =============================================================================
// State Accessors
// =============================================================================

lightning_state_t *task_get_lightning_state(void)
{
    return &lightning_state;
}

pending_ack_state_t *task_get_pending_ack_state(void)
{
    return &pending_ack_state;
}

void task_update_sensor_time(void)
{
    last_sensor_data_time = xTaskGetTickCount();
}

bool task_check_sensor_timeout(void)
{
    if (last_sensor_data_time == 0) {
        return false;  // Never received data, not a timeout
    }
    
    TickType_t elapsed = (xTaskGetTickCount() - last_sensor_data_time) * portTICK_PERIOD_MS;
    return elapsed > SENSOR_TIMEOUT_MS;
}

uint32_t task_get_sensor_elapsed_ms(void)
{
    if (last_sensor_data_time == 0) {
        return 0;
    }
    return (xTaskGetTickCount() - last_sensor_data_time) * portTICK_PERIOD_MS;
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t task_common_init(void)
{
    ESP_LOGI(TAG, "Initializing task common resources...");
    
    // Create semaphores
    lora_mutex = xSemaphoreCreateMutex();
    if (lora_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create lora_mutex");
        return ESP_ERR_NO_MEM;
    }
    
    weather_state_mutex = xSemaphoreCreateMutex();
    if (weather_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create weather_state_mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Create queues
    weather_queue = xQueueCreate(1, sizeof(weather_data_t));
    if (weather_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create weather_queue");
        return ESP_ERR_NO_MEM;
    }
    
    // ACK retry queue - small, just need to signal retry needed
    ack_retry_queue = xQueueCreate(2, sizeof(uint8_t));
    if (ack_retry_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ack_retry_queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Config TX queue - holds config payloads to send
    config_tx_queue = xQueueCreate(2, sizeof(config_payload_t));
    if (config_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create config_tx_queue");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Task common resources initialized");
    return ESP_OK;
}
