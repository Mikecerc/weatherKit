/**
 * @file ui_task.c
 * @brief UI update task for base station
 * 
 * Handles:
 * - Receiving weather data from queue
 * - Updating the display
 * - Periodic UI refresh for animations
 */

#include "ui_task.h"
#include "task_common.h"
#include "ui.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui_task";

void ui_task(void *pvParameters)
{
    weather_data_t weather;
    TickType_t last_refresh = 0;
    const TickType_t refresh_interval = pdMS_TO_TICKS(500);  // Max 2Hz refresh
    
    ESP_LOGI(TAG, "UI task started");
    
    while (1) {
        // Check for pending refresh from button press
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
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void ui_task_start(void)
{
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 4, NULL);
}
