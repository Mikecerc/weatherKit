/**
 * @file config_tx_task.c
 * @brief Config transmission task for base station
 * 
 * Handles:
 * - Sending CONFIG packets to sensor
 * - Sending locate pings
 * - Retrying config until ACK received
 */

#include "config_tx_task.h"
#include "task_common.h"
#include "lora.h"
#include "lora_protocol.h"
#include "ui.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "config_tx";

// Request types
typedef enum {
    CONFIG_REQ_UPDATE,
    CONFIG_REQ_LOCATE
} config_req_type_t;

typedef struct {
    config_req_type_t type;
    union {
        config_payload_t config;
        bool locate_enable;
    };
} config_request_t;

// Queue for config requests
static QueueHandle_t config_request_queue = NULL;

// Sequence counter
static uint8_t config_sequence = 0;

void config_tx_queue_config(const config_payload_t *config)
{
    if (config_request_queue == NULL) return;
    
    config_request_t req = {
        .type = CONFIG_REQ_UPDATE,
        .config = *config
    };
    req.config.config_sequence = ++config_sequence;
    
    xQueueOverwrite(config_request_queue, &req);
    ESP_LOGI(TAG, "Config queued: interval=%d, power=%d", 
             config->update_interval, config->tx_power);
}

void config_tx_queue_locate(bool enable)
{
    if (config_request_queue == NULL) return;
    
    config_request_t req = {
        .type = CONFIG_REQ_LOCATE,
        .locate_enable = enable
    };
    
    xQueueOverwrite(config_request_queue, &req);
    ESP_LOGI(TAG, "Locate %s queued", enable ? "ON" : "OFF");
}

void config_tx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Config TX task started");
    
    config_request_t req;
    
    while (1) {
        // Note: Removed initial config sync - sensor now persists its own config in NVS
        // Config is only sent when user explicitly changes settings in the UI
        
        // Check for UI-triggered config changes
        if (ui_check_config_pending()) {
            config_payload_t config = {
                .update_interval = ui_get_refresh_rate(),
                .tx_power = ui_is_sensor_high_power() ? TX_POWER_HIGH : TX_POWER_LOW,
                .flags = CFG_ADAPTIVE_POWER | (ui_is_sensor_high_power() ? CFG_HIGH_POWER : 0),
                .config_sequence = ++config_sequence
            };
            
            if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t err = lora_send_config(LORA_DEVICE_ID_REMOTE, &config);
                xSemaphoreGive(lora_mutex);
                
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Config sent: interval=%ds, power=%d, flags=0x%02X", 
                             config.update_interval, config.tx_power, config.flags);
                } else {
                    ESP_LOGW(TAG, "Config send failed: %s", esp_err_to_name(err));
                }
            }
        }
        
        // Check for UI-triggered locate
        if (ui_check_locate_pending()) {
            bool enable = ui_is_locate_enabled();
            
            if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t err = lora_send_ping(LORA_DEVICE_ID_REMOTE, enable);
                xSemaphoreGive(lora_mutex);
                
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Locate ping sent: %s", enable ? "ON" : "OFF");
                } else {
                    ESP_LOGW(TAG, "Locate ping failed: %s", esp_err_to_name(err));
                }
            }
        }
        
        // Check for queued requests (from other tasks)
        if (xQueueReceive(config_request_queue, &req, pdMS_TO_TICKS(0)) == pdTRUE) {
            if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                esp_err_t err;
                
                if (req.type == CONFIG_REQ_UPDATE) {
                    err = lora_send_config(LORA_DEVICE_ID_REMOTE, &req.config);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Queued config sent");
                    }
                } else if (req.type == CONFIG_REQ_LOCATE) {
                    err = lora_send_ping(LORA_DEVICE_ID_REMOTE, req.locate_enable);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Queued locate sent");
                    }
                }
                
                xSemaphoreGive(lora_mutex);
            }
        }
        
        // Check for config retry
        if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lora_config_retry_check();
            xSemaphoreGive(lora_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Flag to track if we've synced config with sensor after first packet
static bool config_synced = false;

void config_tx_request_sync(void)
{
    // Mark that we need to send current config to sensor
    // This is called when the first weather packet is received to ensure 
    // the sensor's config matches what the base station expects
    config_synced = false;
    ESP_LOGI(TAG, "Config sync requested");
}

bool config_tx_needs_sync(void)
{
    return !config_synced;
}

void config_tx_mark_synced(void)
{
    config_synced = true;
}

void config_tx_task_start(void)
{
    // Create config request queue
    config_request_queue = xQueueCreate(2, sizeof(config_request_t));
    if (config_request_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create config_request_queue");
        return;
    }
    
    // Create task
    xTaskCreate(config_tx_task, "config_tx", 3072, NULL, 4, NULL);
}
