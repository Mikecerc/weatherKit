/**
 * @file main.c
 * @brief WeatherKit Base Station - Main entry point
 * 
 * This is the indoor display unit that receives weather data from
 * the outdoor sensor via LoRa and displays it on an OLED screen.
 * 
 * Tasks:
 * - lora_rx_task: LoRa packet reception
 * - weather_ack_task: ACK sending and retry (three-way handshake)
 * - config_tx_task: Config/locate command transmission
 * - ui_task: Display updates
 * - button_task: Button handling (created by buttons_init)
 */

#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "ui.h"
#include "buttons.h"
#include "storm_tracker.h"
#include "lora.h"

// Task headers
#include "task_common.h"
#include "lora_rx_task.h"
#include "weather_ack_task.h"
#include "config_tx_task.h"
#include "ui_task.h"

static const char *TAG = "main";

// Track if we just woke from standby - ignore events until all buttons released
static bool just_woke_from_standby = false;

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
static void button_callback(button_event_t event)
{
    if (event == BUTTON_EVENT_NONE) return;
    
    // If in standby mode, ANY button press wakes the display
    if (ui_is_standby()) {
        ui_exit_standby();
        just_woke_from_standby = true;
        return;
    }
    
    // If we just woke, ignore all events until buttons released
    if (just_woke_from_standby) {
        if (event == BUTTON_ANY_PRESS) {
            return;
        }
        if (event == BUTTON_LEFT_SHORT || event == BUTTON_RIGHT_SHORT ||
            event == BUTTON_LEFT_LONG || event == BUTTON_RIGHT_LONG ||
            event == BUTTON_BOTH_LONG) {
            just_woke_from_standby = false;
            return;
        }
    }
    
    // Normal operation
    switch (event) {
        case BUTTON_LEFT_SHORT:
            ui_cycle();
            break;
        case BUTTON_LEFT_LONG:
            ui_show_info();
            break;
        case BUTTON_RIGHT_SHORT:
            ui_select();
            break;
        case BUTTON_RIGHT_LONG:
            ui_back();
            break;
        case BUTTON_BOTH_LONG:
            ui_enter_standby();
            break;
        default:
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "WeatherKit Base Station starting...");

    // Initialize NVS (required for settings storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    // Initialize task common resources (semaphores, queues)
    ESP_ERROR_CHECK(task_common_init());
    ESP_LOGI(TAG, "Task resources initialized");

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
    display_set_brightness(ui_get_settings()->brightness);
    
    // Initialize buttons with callback
    ESP_ERROR_CHECK(buttons_init(button_callback));
    ESP_LOGI(TAG, "Buttons initialized");
    
    // Initialize LoRa module
    ret = lora_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LoRa initialized");
        lora_run_diagnostics();
    } else {
        ESP_LOGE(TAG, "LoRa init failed: %s", esp_err_to_name(ret));
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
    
    // Start tasks
    if (lora_is_initialized()) {
        lora_rx_task_start();       // LoRa packet reception
        weather_ack_task_start();   // ACK sending and retry
        config_tx_task_start();     // Config transmission
    }
    ui_task_start();                // UI updates
    
    ESP_LOGI(TAG, "WeatherKit Base Station ready!");
    ESP_LOGI(TAG, "Waiting for sensor data via LoRa...");
    
    // Main task - periodic status logging
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        lora_status_t lora_status;
        lora_get_status(&lora_status);
        ESP_LOGI(TAG, "Status: Heap=%lu, LoRa RX=%lu",
                 (unsigned long)esp_get_free_heap_size(),
                 lora_status.packets_received);
    }
}
