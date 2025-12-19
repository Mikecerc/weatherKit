/**
 * @file led_status_task.h
 * @brief LED status indicator task for sensor package
 */

#ifndef LED_STATUS_TASK_H
#define LED_STATUS_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Start the LED status task
 * @param handle Optional pointer to store task handle
 * @return ESP_OK on success
 */
esp_err_t led_status_task_start(TaskHandle_t *handle);

/**
 * @brief Stop the LED status task
 */
void led_status_task_stop(void);

#endif // LED_STATUS_TASK_H
