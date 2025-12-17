/**
 * @file weather_tx_task.h
 * @brief Weather data transmission task for sensor package
 * 
 * Handles periodic sensor reading and transmission to base station.
 */

#ifndef WEATHER_TX_TASK_H
#define WEATHER_TX_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Start the weather TX task
 * @param handle Optional pointer to store task handle
 * @return ESP_OK on success
 */
esp_err_t weather_tx_task_start(TaskHandle_t *handle);

/**
 * @brief Stop the weather TX task
 */
void weather_tx_task_stop(void);

#endif // WEATHER_TX_TASK_H
