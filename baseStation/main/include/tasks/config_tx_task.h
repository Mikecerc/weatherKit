/**
 * @file config_tx_task.h
 * @brief Config transmission task for base station
 */

#ifndef CONFIG_TX_TASK_H
#define CONFIG_TX_TASK_H

#include <stdint.h>
#include "esp_err.h"
#include "lora_protocol.h"

/**
 * @brief Start the config TX task
 * Handles sending config and locate commands to sensor
 */
void config_tx_task_start(void);

/**
 * @brief Queue a config update to be sent
 * @param config Config payload to send
 */
void config_tx_queue_config(const config_payload_t *config);

/**
 * @brief Queue a locate ping
 * @param enable Enable or disable locate mode
 */
void config_tx_queue_locate(bool enable);

/**
 * @brief Request a config sync with sensor
 * Call this when the first weather packet is received to ensure sensor has correct config
 */
void config_tx_request_sync(void);

/**
 * @brief Check if config sync is needed
 * @return true if sync hasn't been sent yet
 */
bool config_tx_needs_sync(void);

/**
 * @brief Mark config as synced
 */
void config_tx_mark_synced(void);

/**
 * @brief Config TX task function
 */
void config_tx_task(void *pvParameters);

#endif // CONFIG_TX_TASK_H
