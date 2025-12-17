/**
 * @file weather_ack_task.c
 * @brief Weather ACK sending and retry task
 * 
 * Handles:
 * - Sending WEATHER_ACK in response to weather packets
 * - Retrying ACK if ACK-ACK not received within timeout
 * - Three-way handshake: Weather -> ACK -> ACK-ACK
 */

#include "weather_ack_task.h"
#include "task_common.h"
#include "lora.h"
#include "lora_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "weather_ack";

// Queue item for pending ACK
typedef struct {
    uint8_t src_id;
    uint8_t sequence;
    int8_t rssi;
} ack_request_t;

// Queue for ACK requests
static QueueHandle_t ack_request_queue = NULL;

void weather_ack_queue_ack(uint8_t src_id, uint8_t sequence, int8_t rssi, int8_t snr)
{
    (void)snr;  // SNR not used in current implementation
    if (ack_request_queue == NULL) return;
    
    ack_request_t req = {
        .src_id = src_id,
        .sequence = sequence,
        .rssi = rssi
    };
    
    // Non-blocking - if queue full, latest wins
    xQueueOverwrite(ack_request_queue, &req);
}

void weather_ack_clear_pending(void)
{
    pending_ack_state_t *state = task_get_pending_ack_state();
    state->waiting = false;
    state->retries = 0;
    ESP_LOGD(TAG, "Pending ACK cleared");
}

/**
 * @brief Send a weather ACK packet
 */
static esp_err_t send_weather_ack(uint8_t dst_id, uint8_t sequence, int8_t rssi)
{
    esp_err_t ret = ESP_FAIL;
    
    if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ret = lora_send_weather_ack(dst_id, sequence, rssi);
        xSemaphoreGive(lora_mutex);
        
        if (ret == ESP_OK) {
            // Update pending state
            pending_ack_state_t *state = task_get_pending_ack_state();
            state->sequence = sequence;
            state->src_id = dst_id;
            state->sent_time = xTaskGetTickCount();
            state->base_rssi = rssi;
            state->waiting = true;
            
            ESP_LOGI(TAG, "Sent WEATHER_ACK seq=%d to 0x%02X", sequence, dst_id);
        } else {
            ESP_LOGW(TAG, "Failed to send WEATHER_ACK: %s", esp_err_to_name(ret));
        }
    }
    
    return ret;
}

void weather_ack_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Weather ACK task started");
    
    ack_request_t req;
    
    while (1) {
        // Check for new ACK requests (non-blocking)
        if (xQueueReceive(ack_request_queue, &req, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Got a new weather packet, send ACK
            send_weather_ack(req.src_id, req.sequence, req.rssi);
        }
        
        // Check for ACK-ACK timeout and retry
        pending_ack_state_t *state = task_get_pending_ack_state();
        if (state->waiting) {
            TickType_t elapsed = (xTaskGetTickCount() - state->sent_time) * portTICK_PERIOD_MS;
            
            if (elapsed > ACK_ACK_TIMEOUT_MS) {
                if (state->retries < ACK_ACK_MAX_RETRIES) {
                    // Retry the ACK
                    state->retries++;
                    ESP_LOGW(TAG, "ACK-ACK timeout, retry %d/%d for seq=%d",
                             state->retries, ACK_ACK_MAX_RETRIES, state->sequence);
                    
                    send_weather_ack(state->src_id, state->sequence, state->base_rssi);
                } else {
                    // Give up
                    ESP_LOGE(TAG, "ACK-ACK failed after %d retries, giving up on seq=%d",
                             ACK_ACK_MAX_RETRIES, state->sequence);
                    state->waiting = false;
                    state->retries = 0;
                    
                    // Still record the lightning - sensor probably received ACK but we missed ACK-ACK
                    if (xSemaphoreTake(weather_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        lightning_state_t *ls = task_get_lightning_state();
                        
                        if (ls->pending_count > 0 && ls->pending_closest >= 0) {
                            ESP_LOGW(TAG, "Recording %d pending strikes anyway (ACK-ACK timeout)",
                                     ls->pending_count);
                            // Note: storm_tracker_record_lightning not called here to avoid 
                            // potential double-count. The next weather packet will reveal
                            // whether sensor cleared data or not.
                        }
                        
                        // Optimistically update confirmed total
                        ls->last_confirmed_total = ls->pending_total;
                        ls->pending_total = 0;
                        ls->pending_count = 0;
                        ls->pending_closest = -1.0f;
                        
                        xSemaphoreGive(weather_state_mutex);
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void weather_ack_task_start(void)
{
    // Create ACK request queue
    ack_request_queue = xQueueCreate(1, sizeof(ack_request_t));
    if (ack_request_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create ack_request_queue");
        return;
    }
    
    // Create task
    xTaskCreate(weather_ack_task, "weather_ack", 3072, NULL, 5, NULL);
}
