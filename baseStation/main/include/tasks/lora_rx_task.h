/**
 * @file lora_rx_task.h
 * @brief LoRa receive task for base station
 */

#ifndef LORA_RX_TASK_H
#define LORA_RX_TASK_H

#include "esp_err.h"

/**
 * @brief Start the LoRa RX task
 * Handles packet reception and dispatching to callbacks
 */
void lora_rx_task_start(void);

/**
 * @brief LoRa RX task function (for xTaskCreate)
 */
void lora_rx_task(void *pvParameters);

#endif // LORA_RX_TASK_H
