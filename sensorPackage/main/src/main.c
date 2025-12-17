/**
 * @file main.c
 * @brief WeatherKit Sensor Package Main Entry Point
 * 
 * Remote sensor unit for the WeatherKit weather station.
 * Reads weather sensors and transmits data to base station via LoRa.
 * 
 * Hardware:
 * - ESP32-S3FH4R2 (4MB Flash, 2MB PSRAM)
 * - SX1278/RA-02 LoRa module (915 MHz)
 * - BME280 temperature/humidity/pressure sensor (future)
 * - AS3935 lightning detector (future)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "pinout.h"
#include "sensor_routine.h"

static const char *TAG = "main";

/**
 * @brief Application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "WeatherKit Sensor Package");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS (needed for some ESP-IDF components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "NVS initialized");
    
    // Configure sensor routine
    // Set use_fake_data = false to use real sensors
    sensor_config_t config = {
        .update_interval_sec = 30,      // Send weather data every 30 seconds
        .adaptive_power = true,         // Enable adaptive TX power
        .high_power = false,            // Start with low power mode
        .use_fake_data = false          // Use real sensor data
    };
    
    // Initialize sensor routine
    ret = sensor_routine_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor routine: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check LoRa wiring: SCK=%d, MISO=%d, MOSI=%d, NSS=%d, RST=%d, DIO0=%d",
                 PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, 
                 PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);
        return;
    }
    
    // Start sensor routine
    ret = sensor_routine_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start sensor routine: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Sensor package running!");
    ESP_LOGI(TAG, "- Weather data interval: %d seconds", config.update_interval_sec);
    ESP_LOGI(TAG, "- Adaptive power: %s", config.adaptive_power ? "enabled" : "disabled");
    ESP_LOGI(TAG, "- High power mode: %s", config.high_power ? "enabled" : "disabled");
    ESP_LOGI(TAG, "- Data source: %s", config.use_fake_data ? "FAKE" : "real sensors");
    
    // Main loop - just monitor status
    while (1) {
        // Print statistics periodically
        uint32_t sent, acked, failed;
        sensor_routine_get_stats(&sent, &acked, &failed);
        
        ESP_LOGI(TAG, "Stats: sent=%lu, acked=%lu, failed=%lu, uptime=%d min",
                 (unsigned long)sent, (unsigned long)acked, (unsigned long)failed,
                 sensor_routine_get_uptime_min());
        
        // Sleep for a while
        vTaskDelay(pdMS_TO_TICKS(60000));  // Print stats every minute
    }
}