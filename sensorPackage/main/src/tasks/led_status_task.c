/**
 * @file led_status_task.c
 * @brief LED status indicator task for sensor package
 * 
 * Manages LED state based on LoRa connection and packet status:
 * - Flash red: LoRa offline
 * - Solid green: LoRa online, using real sensor data
 * - Solid blue: LoRa online, using fake data
 * - Bright green/blue flash: Packet sent successfully
 * - Red flash: Packet send failed
 * - Purple flash: Packet/config received from base
 */

#include "tasks/led_status_task.h"
#include "tasks/task_common.h"
#include "drivers/led.h"
#include "drivers/lora.h"
#include "esp_log.h"

static const char *TAG = "led_task";

static TaskHandle_t led_task_handle = NULL;

/**
 * @brief LED status indicator task
 */
static void led_status_task(void *arg)
{
    ESP_LOGI(TAG, "LED status task started");
    
    bool led_on = true;
    int flash_counter = 0;
    
    // Start with LED off
    led_off();
    
    while (g_tasks_running) {
        // Check for one-shot packet status flashes (highest priority)
        if (g_led_flash_packet_sent) {
            g_led_flash_packet_sent = false;
            // Bright flash for successful send - green for real data, blue for fake
            if (g_led_using_fake_data) {
                led_set_color(0, 0, 255);  // Bright blue (fake data)
            } else {
                led_set_color(0, 255, 0);  // Bright green (real data)
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        if (g_led_flash_packet_failed) {
            g_led_flash_packet_failed = false;
            led_set_color(255, 0, 0);  // Red
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        
        if (g_led_flash_packet_received) {
            g_led_flash_packet_received = false;
            led_set_color(128, 0, 255);  // Purple
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Handle base LED status based on LoRa state
        if (lora_is_initialized()) {
            // LoRa online - solid color (green for real data, blue for fake)
            if (g_led_using_fake_data) {
                led_set_color(0, 0, 50);  // Dim blue (fake data mode)
            } else {
                led_set_color(0, 50, 0);  // Dim green (real data mode)
            }
            flash_counter = 0;
            led_on = true;
        } else {
            // LoRa offline - flash red (500ms on, 500ms off)
            flash_counter++;
            if (flash_counter >= 10) {
                flash_counter = 0;
                led_on = !led_on;
            }
            
            if (led_on) {
                led_set_color(255, 0, 0);
            } else {
                led_set_color(0, 0, 0);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    led_off();
    ESP_LOGI(TAG, "LED status task stopped");
    led_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t led_status_task_start(TaskHandle_t *handle)
{
    if (led_task_handle != NULL) {
        ESP_LOGW(TAG, "LED status task already running");
        return ESP_OK;
    }
    
    BaseType_t ret = xTaskCreate(
        led_status_task,
        "led_status",
        3072,
        NULL,
        3,  // Lower priority than other tasks
        &led_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED status task");
        return ESP_ERR_NO_MEM;
    }
    
    if (handle != NULL) {
        *handle = led_task_handle;
    }
    
    return ESP_OK;
}

void led_status_task_stop(void)
{
    // Task will stop when g_tasks_running becomes false
    led_task_handle = NULL;
}
