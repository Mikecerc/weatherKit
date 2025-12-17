/**
 * @file lora_rx_task.h
 * @brief LoRa receive task for sensor package
 * 
 * Handles receiving packets from base station (config, ACKs)
 */

#ifndef LORA_RX_TASK_H
#define LORA_RX_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Start the LoRa RX task
 * @param handle Optional pointer to store task handle
 * @return ESP_OK on success
 */
esp_err_t lora_rx_task_start(TaskHandle_t *handle);

/**
 * @brief Stop the LoRa RX task
 */
void lora_rx_task_stop(void);

#endif // LORA_RX_TASK_H
