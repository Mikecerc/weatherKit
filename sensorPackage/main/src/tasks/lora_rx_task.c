/**
 * @file lora_rx_task.c
 * @brief LoRa receive task for sensor package
 * 
 * Handles receiving packets from base station:
 * - CONFIG messages: Update sensor settings
 * - WEATHER_ACK messages: Clear lightning data, send ACK-ACK
 */

#include "tasks/lora_rx_task.h"
#include "tasks/task_common.h"
#include "drivers/lora.h"
#include "drivers/lora_protocol.h"
#include "drivers/buzzer.h"
#include "esp_log.h"

static const char *TAG = "lora_rx";

static TaskHandle_t rx_task_handle = NULL;

// =============================================================================
// Callbacks (called from lora_process_rx in this task's context)
// =============================================================================

/**
 * @brief Called when configuration message received from base
 */
static void on_config_received(const config_payload_t *config)
{
    ESP_LOGI(TAG, "Config received: interval=%ds, power=%d, flags=0x%02X",
             config->update_interval, config->tx_power, config->flags);
    
    task_signal_packet_received();
    
    uint8_t applied_flags = 0;
    
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update interval if valid
        if (config->update_interval >= MIN_UPDATE_INTERVAL_SEC && 
            config->update_interval <= MAX_UPDATE_INTERVAL_SEC) {
            g_current_config.update_interval_sec = config->update_interval;
        }
        
        // Update adaptive power setting
        if (config->flags & CFG_ADAPTIVE_POWER) {
            g_current_config.adaptive_power = true;
            applied_flags |= CFG_ADAPTIVE_POWER;
        } else {
            g_current_config.adaptive_power = false;
        }
        
        // Update high power mode
        if (config->flags & CFG_HIGH_POWER) {
            g_current_config.high_power = true;
            applied_flags |= CFG_HIGH_POWER;
        } else {
            g_current_config.high_power = false;
        }
        
        xSemaphoreGive(g_config_mutex);
    }
    
    // Handle locate buzzer (outside mutex - buzzer has its own protection)
    if (config->flags & CFG_LOCATE_BUZZER) {
        buzzer_set_locate(true);
        applied_flags |= CFG_LOCATE_BUZZER;
        ESP_LOGI(TAG, "Locate mode ENABLED");
    } else {
        buzzer_set_locate(false);
    }
    
    // Apply LoRa settings (we're already inside g_lora_mutex from the RX loop)
    lora_set_adaptive_power(g_current_config.adaptive_power);
    lora_set_high_power(g_current_config.high_power);
    if (!g_current_config.adaptive_power && config->tx_power >= TX_POWER_MIN) {
        lora_set_tx_power(config->tx_power);
    }
    
    // Save config to NVS flash for persistence across reboots
    esp_err_t save_err = task_config_save();
    if (save_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config to NVS: %s", esp_err_to_name(save_err));
    }
    
    // Send CONFIG_ACK - note: we're already inside g_lora_mutex from lora_rx_task
    // so we cannot take it again (not a recursive mutex)
    lora_send_config_ack(config->config_sequence, applied_flags);
    ESP_LOGI(TAG, "CONFIG_ACK sent for seq=%d, flags=0x%02X", 
             config->config_sequence, applied_flags);
}

/**
 * @brief Called when weather ACK received from base
 * Clears pending lightning data and sends ACK-ACK
 */
static void on_weather_ack_received(const weather_ack_payload_t *ack)
{
    ESP_LOGI(TAG, "Weather ACK received: seq=%d, base_rssi=%d, suggested_power=%d",
             ack->acked_sequence, ack->base_rssi, ack->suggested_power);
    
    task_signal_packet_received();
    
    // Only clear lightning if this ACK matches our last sent weather packet
    if (ack->acked_sequence == g_last_weather_sequence) {
        uint32_t total_count = 0;
        
        // Clear pending lightning strikes (protected by mutex)
        if (xSemaphoreTake(g_lightning_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lightning_data_clear_pending(&g_accumulated_lightning);
            total_count = g_accumulated_lightning.total_count;
            xSemaphoreGive(g_lightning_mutex);
        }
        
        ESP_LOGI(TAG, "Lightning data cleared (ACK seq matched)");
        g_awaiting_weather_ack = false;
        g_packets_acked++;
        
        // Send ACK-ACK to confirm we cleared the lightning data
        // Note: we're already inside g_lora_mutex from lora_rx_task (not a recursive mutex)
        lora_send_weather_ack_ack(ack->acked_sequence, total_count);
        ESP_LOGI(TAG, "WEATHER_ACK_ACK sent for seq=%d", ack->acked_sequence);
    } else {
        ESP_LOGW(TAG, "ACK sequence mismatch: got %d, expected %d (ignoring duplicate)",
                 ack->acked_sequence, g_last_weather_sequence);
    }
}

/**
 * @brief LoRa RX task main loop
 * 
 * Priority is higher than TX to ensure we don't miss packets.
 * After TX completes, we need to quickly enter RX mode to catch ACKs.
 */
static void lora_rx_task(void *arg)
{
    ESP_LOGI(TAG, "LoRa RX task started");
    
    // Register callbacks
    lora_set_config_callback(on_config_received);
    lora_set_weather_ack_callback(on_weather_ack_received);
    
    int loop_count = 0;
    
    while (g_tasks_running) {
        loop_count++;
        
        // Check for incoming packets
        if (lora_is_initialized()) {
            // Take mutex for RX operations - wait longer to ensure we get it
            if (xSemaphoreTake(g_lora_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Ensure we're in receive mode
                lora_receive();
                
                // Log periodically to show we're active and show LoRa status
                if (loop_count % 500 == 0) {
                    lora_status_t st;
                    lora_get_status(&st);
                    ESP_LOGI(TAG, "RX loop %d: RX=%lu, rejected=%lu, CRC_err=%lu, TX=%lu",
                             loop_count, st.packets_received, st.packets_rejected, 
                             st.crc_errors, st.packets_sent);
                }
                
                // Process any received packets
                while (lora_received()) {
                    ESP_LOGI(TAG, "Packet detected! Processing...");
                    if (lora_process_rx()) {
                        ESP_LOGI(TAG, "Packet processed from base");
                    }
                    lora_receive();  // Back to RX mode
                }
                
                xSemaphoreGive(g_lora_mutex);
            } else {
                // Couldn't get mutex - TX might be in progress
                // This is expected, just log occasionally
                if (loop_count % 100 == 0) {
                    ESP_LOGD(TAG, "RX waiting for mutex (TX in progress?)");
                }
            }
        }
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "LoRa RX task stopped");
    rx_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t lora_rx_task_start(TaskHandle_t *handle)
{
    if (rx_task_handle != NULL) {
        ESP_LOGW(TAG, "LoRa RX task already running");
        return ESP_OK;
    }
    
    if (!lora_is_initialized()) {
        ESP_LOGW(TAG, "LoRa not initialized, skipping RX task");
        return ESP_ERR_INVALID_STATE;
    }
    
    BaseType_t ret = xTaskCreate(
        lora_rx_task,
        "lora_rx",
        4096,
        NULL,
        5,  // Higher priority for responsive packet handling
        &rx_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa RX task");
        return ESP_ERR_NO_MEM;
    }
    
    if (handle != NULL) {
        *handle = rx_task_handle;
    }
    
    return ESP_OK;
}

void lora_rx_task_stop(void)
{
    rx_task_handle = NULL;
}
