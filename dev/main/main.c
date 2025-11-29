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

static const char *TAG = "main";

// Queue for weather data updates
static QueueHandle_t weather_queue = NULL;

/**
 * @brief Button event callback - runs in button task context, keep minimal!
 * 
 * Navigation scheme:
 * - LEFT short: cycle through pages / menu items
 * - LEFT long: show info/help for current page
 * - RIGHT short: select (enter context or toggle option)
 * - RIGHT long: escape context (go back)
 */
static void button_callback(button_event_t event)
{
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
            
        default:
            break;
    }
}

/**
 * @brief Generate fake weather data for testing
 */
static void generate_fake_weather(weather_data_t *data)
{
    // Generate somewhat realistic fake data with small variations
    static float base_temp = 22.0f;
    static float base_humidity = 45.0f;
    static float base_pressure = 1013.25f;
    
    // Add small random variations
    base_temp += ((float)(esp_random() % 100) - 50) / 100.0f;  // +/- 0.5
    base_humidity += ((float)(esp_random() % 100) - 50) / 50.0f;  // +/- 1.0
    base_pressure += ((float)(esp_random() % 100) - 50) / 25.0f;  // +/- 2.0
    
    // Keep values in realistic ranges
    if (base_temp < 15.0f) base_temp = 15.0f;
    if (base_temp > 35.0f) base_temp = 35.0f;
    if (base_humidity < 20.0f) base_humidity = 20.0f;
    if (base_humidity > 80.0f) base_humidity = 80.0f;
    if (base_pressure < 980.0f) base_pressure = 980.0f;
    if (base_pressure > 1040.0f) base_pressure = 1040.0f;
    
    data->temperature = base_temp;
    data->humidity = base_humidity;
    data->pressure = base_pressure;
    data->sensor_connected = false;  // No remote sensor yet
    
    // Randomly generate lightning (10% chance, random distance)
    if ((esp_random() % 10) == 0) {
        data->lightning_dist = (float)(esp_random() % 400) / 10.0f;  // 0-40km
    } else {
        data->lightning_dist = -1;  // No lightning
    }
}

/**
 * @brief Task to read sensors and generate weather data
 */
static void sensor_task(void *pvParameters)
{
    weather_data_t weather;
    
    ESP_LOGI(TAG, "Sensor task started");
    
    while (1) {
        // Generate fake data (replace with real sensor reads later)
        generate_fake_weather(&weather);
        
        // Record weather data in storm tracker
        storm_tracker_record_weather(weather.temperature, weather.humidity, weather.pressure);
        
        // If lightning detected, record it in storm tracker
        if (weather.lightning_dist >= 0) {
            storm_tracker_record_lightning(weather.lightning_dist);
            ESP_LOGI(TAG, "Lightning strike detected: %.1f km", weather.lightning_dist);
        }
        
        // Send to queue for UI task
        if (weather_queue != NULL) {
            xQueueOverwrite(weather_queue, &weather);
        }
        
        // Update every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    
    // Set initial fake data
    weather_data_t initial_weather = {
        .temperature = 22.5f,
        .humidity = 45.0f,
        .pressure = 1013.25f,
        .lightning_dist = -1,
        .sensor_connected = false
    };
    ui_update_weather(&initial_weather);
    
    // Create tasks
    // Button task is created inside buttons_init()
    
    // Sensor reading task - lower priority
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 3, NULL);
    
    // UI update task - medium priority
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "WeatherKit ready!");
    ESP_LOGI(TAG, "Use LEFT/RIGHT buttons to navigate pages");
    ESP_LOGI(TAG, "Hold LEFT to go back, hold RIGHT to enter/edit");
    
    // Main task can exit - everything runs in FreeRTOS tasks
    // Or we can use it for additional monitoring
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Periodic status log
        ESP_LOGI(TAG, "System running - Page: %d, Free heap: %lu bytes",
                 ui_get_current_page(), (unsigned long)esp_get_free_heap_size());
    }
}