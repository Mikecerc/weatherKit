/**
 * @file lora_rx_task.c
 * @brief LoRa receive task for base station
 * 
 * Handles:
 * - Continuous LoRa packet reception
 * - Dispatching received packets to appropriate handlers
 * - Sensor timeout detection
 */

#include "lora_rx_task.h"
#include "task_common.h"
#include "weather_ack_task.h"
#include "config_tx_task.h"
#include "lora.h"
#include "lora_protocol.h"
#include "storm_tracker.h"
#include "ui.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lora_rx";

// =============================================================================
// LoRa Callbacks
// =============================================================================

// Track if this is the first weather packet (for initial config sync)
static bool first_weather_received = false;

/**
 * @brief LoRa weather data callback - called when weather packet received
 */
static void on_weather_received(uint8_t src_id, const weather_payload_t *data, int8_t rssi)
{
    ESP_LOGI(TAG, "Weather data from sensor 0x%02X (RSSI: %d dBm)", src_id, rssi);
    
    // On first weather packet, request config sync to ensure sensor has correct settings
    if (!first_weather_received) {
        first_weather_received = true;
        config_tx_request_sync();
        ESP_LOGI(TAG, "First weather packet - requesting config sync");
    }
    
    // Convert from protocol format to UI format
    weather_data_t weather = {
        .temperature = decode_temperature(data->temperature),
        .humidity = decode_humidity(data->humidity),
        .pressure = decode_pressure(data->pressure),
        .lightning_dist = -1.0f,
        .lightning_count = data->lightning.strikes_since_ack,
        .lightning_total = data->lightning.total_count,
        .sensor_connected = true,
        
        // Sensor configuration echo
        .sensor_update_interval = data->config.update_interval,
        .sensor_tx_power = data->config.tx_power,
        .sensor_high_power = (data->config.flags & CFG_HIGH_POWER) != 0,
        .sensor_adaptive_power = (data->config.flags & CFG_ADAPTIVE_POWER) != 0,
        .sensor_rssi = data->last_base_rssi,
        .sensor_uptime_sec = data->uptime_sec,
        
        // Base station link quality
        .base_rssi = rssi,
        .base_tx_power = 0,
        
        // Timing
        .last_rx_time_ms = (uint32_t)(esp_timer_get_time() / 1000)
    };
    
    // Get base TX power from lora status
    lora_status_t lora_st;
    lora_get_status(&lora_st);
    weather.base_tx_power = lora_st.tx_power;
    
    // Process lightning data with mutex protection
    if (xSemaphoreTake(weather_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lightning_state_t *ls = task_get_lightning_state();
        
        uint32_t new_strikes = 0;
        if (data->lightning.total_count > ls->last_confirmed_total) {
            new_strikes = data->lightning.total_count - ls->last_confirmed_total;
            ESP_LOGI(TAG, "  Pending lightning: %lu strikes (confirmed %lu -> pending %lu)",
                     (unsigned long)new_strikes,
                     (unsigned long)ls->last_confirmed_total,
                     (unsigned long)data->lightning.total_count);
            
            // Cache pending lightning data - will be recorded when ACK-ACK received
            ls->pending_total = data->lightning.total_count;
            ls->pending_count = (uint8_t)(new_strikes > 255 ? 255 : new_strikes);
            
            uint8_t closest = lightning_data_closest(&data->lightning);
            if (closest < 255) {
                weather.lightning_dist = (float)closest;
                ls->pending_closest = (float)closest;
            }
            
            ESP_LOGI(TAG, "    Cached %d strikes at closest distance: %.0f km (awaiting ACK-ACK)", 
                     ls->pending_count, ls->pending_closest);
        } else if (data->lightning.strikes_since_ack > 0) {
            uint8_t closest = lightning_data_closest(&data->lightning);
            if (closest < 255) {
                weather.lightning_dist = (float)closest;
            }
            ESP_LOGD(TAG, "  Lightning data unchanged (total=%lu)",
                     (unsigned long)data->lightning.total_count);
        }
        
        weather.lightning_count = (uint8_t)(new_strikes > 255 ? 255 : new_strikes);
        xSemaphoreGive(weather_state_mutex);
    }
    
    // Record weather in storm tracker (not lightning - that waits for ACK-ACK)
    storm_tracker_record_weather(weather.temperature, weather.humidity, weather.pressure);
    
    // Update sensor receive time
    task_update_sensor_time();
    
    // Queue for UI
    if (weather_queue != NULL) {
        xQueueOverwrite(weather_queue, &weather);
    }
    
    // Notify ACK task that we need to send an ACK
    weather_ack_queue_ack(src_id, data->sequence, rssi, lora_st.last_snr);
    
    ESP_LOGI(TAG, "  Temp: %.1f°C, Humidity: %.1f%%, Pressure: %.1f hPa, Lightning: %d/%lu",
             weather.temperature, weather.humidity, weather.pressure,
             weather.lightning_count, (unsigned long)weather.lightning_total);
}

/**
 * @brief LoRa status callback - called when status/heartbeat packet received
 */
static void on_status_received(uint8_t src_id, const status_payload_t *data, int8_t rssi)
{
    ESP_LOGI(TAG, "Status from sensor 0x%02X: Battery %d%% (%dmV), RSSI %d",
             src_id, data->battery_percent, data->battery_mv, rssi);
    
    if (data->error_flags) {
        ESP_LOGW(TAG, "  Sensor errors: 0x%02X", data->error_flags);
    }
    
    task_update_sensor_time();
}

/**
 * @brief LoRa ACK-ACK callback - sensor confirmed it cleared lightning data
 */
static void on_weather_ack_ack_received(const weather_ack_ack_payload_t *data)
{
    ESP_LOGI(TAG, "ACK-ACK received: lightning_total=%lu", (unsigned long)data->lightning_total);
    
    // Clear pending ACK state
    weather_ack_clear_pending();
    
    // Record lightning now that we're confirmed
    if (xSemaphoreTake(weather_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lightning_state_t *ls = task_get_lightning_state();
        
        if (data->lightning_total != ls->pending_total) {
            ESP_LOGW(TAG, "  ACK-ACK total mismatch: expected %lu, got %lu",
                     (unsigned long)ls->pending_total, (unsigned long)data->lightning_total);
        }
        
        // Record to storm tracker
        if (ls->pending_count > 0 && ls->pending_closest >= 0) {
            for (uint8_t i = 0; i < ls->pending_count; i++) {
                storm_tracker_record_lightning(ls->pending_closest);
            }
            ESP_LOGI(TAG, "  Recorded %d strikes at %.0f km to storm tracker", 
                     ls->pending_count, ls->pending_closest);
        }
        
        // Update confirmed state
        ls->last_confirmed_total = data->lightning_total;
        ls->pending_total = 0;
        ls->pending_count = 0;
        ls->pending_closest = -1.0f;
        
        xSemaphoreGive(weather_state_mutex);
    }
}

// =============================================================================
// Task Function
// =============================================================================

void lora_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LoRa RX task started");
    
    static int loop_count = 0;
    
    while (1) {
        loop_count++;
        
        // Take LoRa mutex for RX operations
        if (xSemaphoreTake(lora_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Enter receive mode
            lora_receive();
            
            // Poll for received packets
            while (lora_received()) {
                ESP_LOGD(TAG, "Packet detected");
                lora_process_rx();
                lora_receive();
            }
            
            xSemaphoreGive(lora_mutex);
        }
        
        // Debug every ~30 seconds (3000 loops at 10ms = 30s)
        if (loop_count % 3000 == 0) {
            lora_status_t st;
            lora_get_status(&st);
            ESP_LOGI(TAG, "RX loop %d: RX=%lu, rejected=%lu, CRC_err=%lu", 
                     loop_count, st.packets_received, st.packets_rejected, st.crc_errors);
        }
        
        // Check sensor timeout
        if (task_check_sensor_timeout()) {
            weather_data_t timeout_update = {
                .sensor_connected = false
            };
            if (weather_queue != NULL) {
                xQueueOverwrite(weather_queue, &timeout_update);
            }
            ESP_LOGW(TAG, "Sensor timeout - no data for %lu ms", 
                     (unsigned long)task_get_sensor_elapsed_ms());
        }
        
        // Delay to allo file containing your project source filw other tasks (including IDLE) to run
        // 10ms is plenty fast for LoRa polling (packets take ~100ms+ to transmit)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lora_rx_task_start(void)
{
    // Set up callbacks
    lora_set_weather_callback(on_weather_received);
    lora_set_status_callback(on_status_received);
    lora_set_weather_ack_ack_callback(on_weather_ack_ack_received);
    
    // Enable adaptive power (both base and sensor are battery powered)
    lora_set_adaptive_power(true);
    ESP_LOGI(TAG, "Adaptive power enabled");
    
    // Create task
    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
}
