#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "display.h"
#include "ui.h"
#include "buttons.h"
#include "storm_tracker.h"
#include "lora.h"
#include "lora_protocol.h"

static const char *TAG = "main";

// Queue for weather data updates from LoRa
static QueueHandle_t weather_queue = NULL;

// Track last time we received data from sensor
static TickType_t last_sensor_data_time = 0;
#define SENSOR_TIMEOUT_MS       60000   // Consider sensor disconnected after 60s

/**
 * @brief Button event callback - runs in button task context, keep minimal!
 * 
 * Navigation scheme:
 * - LEFT short: cycle through pages / menu items
 * - LEFT long: show info/help for current page
 * - RIGHT short: select (enter context or toggle option)
 * - RIGHT long: escape context (go back)
 * - BOTH long: toggle standby mode (display off, still receiving data)
 */

// Track if we just woke from standby - ignore events until all buttons released
static bool just_woke_from_standby = false;

static void button_callback(button_event_t event)
{
    // Ignore no-op events
    if (event == BUTTON_EVENT_NONE) return;
    
    // If in standby mode, ANY button press wakes the display
    if (ui_is_standby()) {
        ui_exit_standby();
        just_woke_from_standby = true;  // Set flag to ignore subsequent events
        return;  // Don't process further - just wake up
    }
    
    // If we just woke, ignore all events until ANY_PRESS stops coming
    // (meaning all buttons have been released)
    if (just_woke_from_standby) {
        if (event == BUTTON_ANY_PRESS) {
            // Still pressing - keep ignoring
            return;
        }
        // Check if this is a release event (SHORT events fire on release)
        // We want to ignore this first release
        if (event == BUTTON_LEFT_SHORT || event == BUTTON_RIGHT_SHORT ||
            event == BUTTON_LEFT_LONG || event == BUTTON_RIGHT_LONG ||
            event == BUTTON_BOTH_LONG) {
            just_woke_from_standby = false;  // Clear flag, next press will work
            return;  // But ignore this one
        }
    }
    
    // Normal operation - process button events
    switch (event) {
        case BUTTON_LEFT_SHORT:
            ui_cycle();      // Cycle pages or menu items
            break;
            
        case BUTTON_LEFT_LONG:
            ui_show_info();  // Show help/info for current page
            break;
            
        case BUTTON_RIGHT_SHORT:
            ui_select();     // Select / enter / toggle
            break;
            
        case BUTTON_RIGHT_LONG:
            ui_back();       // Escape current context
            break;
            
        case BUTTON_BOTH_LONG:
            ui_enter_standby();  // Enter standby mode
            break;
            
        case BUTTON_ANY_PRESS:
            // Already handled above for standby wake
            // No action needed in normal mode
            break;
            
        default:
            break;
    }
}

/**
 * @brief LoRa weather data callback - called when weather packet received
 * 
 * This runs in the LoRa RX task context, so keep it fast.
 * We just convert and queue the data for the main task.
 */
static void on_weather_received(uint8_t src_id, const weather_payload_t *data, int8_t rssi)
{
    ESP_LOGI(TAG, "Weather data from sensor 0x%02X (RSSI: %d dBm)", src_id, rssi);
    
    // Convert from protocol format to UI format
    weather_data_t weather = {
        .temperature = data->temperature / 100.0f,      // 0.01°C -> °C
        .humidity = data->humidity / 100.0f,            // 0.01% -> %
        .pressure = data->pressure / 10.0f,             // 0.1 hPa -> hPa
        .lightning_dist = -1.0f,                        // Will be set below if strikes present
        .sensor_connected = true
    };
    
    // Process lightning data
    if (data->lightning.count > 0) {
        // Find the closest strike
        uint8_t closest = 255;
        for (int i = 0; i < data->lightning.count && i < MAX_LIGHTNING_STRIKES; i++) {
            if (data->lightning.distances[i] < closest && data->lightning.distances[i] != 0xFF) {
                closest = data->lightning.distances[i];
            }
            // Record each strike in storm tracker
            if (data->lightning.distances[i] != 0xFF) {
                storm_tracker_record_lightning((float)data->lightning.distances[i]);
                ESP_LOGI(TAG, "  Lightning strike: %d km", data->lightning.distances[i]);
            }
        }
        if (closest < 255) {
            weather.lightning_dist = (float)closest;
        }
    }
    
    // Record weather in storm tracker
    storm_tracker_record_weather(weather.temperature, weather.humidity, weather.pressure);
    
    // Update last received time
    last_sensor_data_time = xTaskGetTickCount();
    
    // Send to queue for UI task
    if (weather_queue != NULL) {
        xQueueOverwrite(weather_queue, &weather);
    }
    
    ESP_LOGI(TAG, "  Temp: %.1f°C, Humidity: %.1f%%, Pressure: %.1f hPa",
             weather.temperature, weather.humidity, weather.pressure);
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
    
    // Update last received time (status counts as valid communication)
    last_sensor_data_time = xTaskGetTickCount();
}

/**
 * @brief Task to receive LoRa packets
 * 
 * Simple polling loop - matches working Inteform library exactly
 */
static void lora_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LoRa RX task started - waiting for sensor data...");
    
    static int loop_count = 0;
    
    while (1) {
        loop_count++;
        
        // Check if UI requested a locate ping
        if (ui_check_locate_pending()) {
            ESP_LOGI(TAG, "Sending LOCATE ping to sensor...");
            esp_err_t err = lora_send_ping(LORA_DEVICE_ID_REMOTE, true);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Locate ping sent successfully");
            } else {
                ESP_LOGW(TAG, "Locate ping failed: %s", esp_err_to_name(err));
            }
        }
        
        // Check if UI changed config (refresh rate, high power)
        if (ui_check_config_pending()) {
            ESP_LOGI(TAG, "Sending CONFIG to sensor...");
            config_payload_t config = {
                .update_interval = ui_get_refresh_rate(),
                .heartbeat_interval = 60,  // Fixed heartbeat interval
                .tx_power = ui_is_sensor_high_power() ? TX_POWER_HIGH : TX_POWER_LOW,
                .flags = ui_is_sensor_high_power() ? 0 : CFG_ADAPTIVE_POWER  // Adaptive when not high power
            };
            esp_err_t err = lora_send_config(LORA_DEVICE_ID_REMOTE, &config);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Config sent: interval=%ds, power=%d", 
                         config.update_interval, config.tx_power);
            } else {
                ESP_LOGW(TAG, "Config send failed: %s", esp_err_to_name(err));
            }
        }
        
        // Enter receive mode
        lora_receive();
        
        // Poll for received packets
        while (lora_received()) {
            ESP_LOGI(TAG, ">>> Packet detected!");
            bool processed = lora_process_rx();
            ESP_LOGI(TAG, ">>> Process: %s", processed ? "OK" : "FAILED");
            lora_receive();
        }
        
        // Debug every ~10 seconds
        if (loop_count % 1000 == 0) {
            lora_status_t st;
            lora_get_status(&st);
            ESP_LOGI(TAG, "RX loop %d: RX=%lu, rejected=%lu, CRC_err=%lu", 
                     loop_count, st.packets_received, st.packets_rejected, st.crc_errors);
        }
        
        vTaskDelay(1);
        
        // Check if sensor has timed out
        if (last_sensor_data_time != 0) {
            TickType_t elapsed = (xTaskGetTickCount() - last_sensor_data_time) * portTICK_PERIOD_MS;
            if (elapsed > SENSOR_TIMEOUT_MS) {
                // Sensor timed out - mark as disconnected
                weather_data_t timeout_update = {
                    .temperature = 0,
                    .humidity = 0,
                    .pressure = 0,
                    .lightning_dist = -1,
                    .sensor_connected = false
                };
                if (weather_queue != NULL) {
                    xQueueOverwrite(weather_queue, &timeout_update);
                }
                last_sensor_data_time = 0;  // Reset so we don't spam
                ESP_LOGW(TAG, "Sensor timeout - no data for %d seconds", SENSOR_TIMEOUT_MS / 1000);
            }
        }
    }
}

/**
 * @brief Task to handle UI updates
 */
static void ui_task(void *pvParameters)
{
    weather_data_t weather;
    TickType_t last_refresh = 0;
    const TickType_t refresh_interval = pdMS_TO_TICKS(500);  // Max 2Hz refresh
    
    ESP_LOGI(TAG, "UI task started");
    
    while (1) {
        // Check for pending refresh from button press (deferred from button task)
        ui_check_refresh();
        
        // Check for new weather data
        if (xQueueReceive(weather_queue, &weather, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Got new data, update UI
            ui_update_weather(&weather);
            last_refresh = xTaskGetTickCount();
        } else {
            // No new data, but still refresh periodically for animations/clock
            TickType_t now = xTaskGetTickCount();
            if ((now - last_refresh) >= refresh_interval) {
                ui_refresh();
                last_refresh = now;
            }
        }
        
        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WeatherKit starting...");

    // Initialize NVS (required for settings storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Create weather data queue (single item, always overwrite with latest)
    weather_queue = xQueueCreate(1, sizeof(weather_data_t));
    if (weather_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create weather queue!");
        return;
    }

    // Initialize storm tracker
    ESP_ERROR_CHECK(storm_tracker_init());
    ESP_LOGI(TAG, "Storm tracker initialized");

    // Initialize display hardware
    ESP_ERROR_CHECK(display_init());
    ESP_LOGI(TAG, "Display initialized");
    
    // Initialize UI system
    ESP_ERROR_CHECK(ui_init());
    ESP_LOGI(TAG, "UI initialized");
    
    // Load saved settings from flash
    ui_load_settings();
    
    // Apply loaded brightness setting
    display_set_brightness(ui_get_settings()->brightness);;
    
    // Initialize buttons with callback
    ESP_ERROR_CHECK(buttons_init(button_callback));
    ESP_LOGI(TAG, "Buttons initialized");
    
    // Initialize LoRa module
    ret = lora_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LoRa initialized");
        // Run diagnostics to verify hardware
        lora_run_diagnostics();
        
        // Set up callbacks for received data
        lora_set_weather_callback(on_weather_received);
        lora_set_status_callback(on_status_received);
        
        // Enable adaptive power - adjusts TX power based on link quality
        lora_set_adaptive_power(true);
        ESP_LOGI(TAG, "Adaptive power enabled");
    } else {
        ESP_LOGE(TAG, "LoRa init failed: %s - cannot receive sensor data!", esp_err_to_name(ret));
    }
    
    // Set initial state - waiting for sensor
    weather_data_t initial_weather = {
        .temperature = 0.0f,
        .humidity = 0.0f,
        .pressure = 0.0f,
        .lightning_dist = -1,
        .sensor_connected = false
    };
    ui_update_weather(&initial_weather);
    
    // Create tasks
    // Button task is created inside buttons_init()
    
    // LoRa receive task - medium-high priority
    if (lora_is_initialized()) {
        xTaskCreate(lora_rx_task, "lora_rx_task", 4096, NULL, 5, NULL);
    }
    
    // UI update task - medium priority
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "WeatherKit Base Station ready!");
    ESP_LOGI(TAG, "Waiting for remote sensor data via LoRa...");
    ESP_LOGI(TAG, "Use LEFT/RIGHT buttons to navigate pages");
    
    // Main task can exit - everything runs in FreeRTOS tasks
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        // Periodic status log
        lora_status_t lora_status;
        lora_get_status(&lora_status);
        ESP_LOGI(TAG, "Status: Heap=%lu, LoRa RX=%lu, Sensor=%s",
                 (unsigned long)esp_get_free_heap_size(),
                 lora_status.packets_received,
                 (last_sensor_data_time != 0) ? "Connected" : "Waiting");
    }
}