/**
 * @file weather_ack_task.h
 * @brief Weather ACK sending and retry task for base station
 */

#ifndef WEATHER_ACK_TASK_H
#define WEATHER_ACK_TASK_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Start the weather ACK task
 * Handles sending ACKs and retrying if ACK-ACK not received
 */
void weather_ack_task_start(void);

/**
 * @brief Queue an ACK to be sent (called from RX task)
 * @param src_id Sensor device ID
 * @param sequence Weather packet sequence number
 * @param rssi RSSI of received weather packet
 * @param snr SNR of received weather packet
 */
void weather_ack_queue_ack(uint8_t src_id, uint8_t sequence, int8_t rssi, int8_t snr);

/**
 * @brief Clear pending ACK state (called when ACK-ACK received)
 */
void weather_ack_clear_pending(void);

/**
 * @brief Weather ACK task function
 */
void weather_ack_task(void *pvParameters);

#endif // WEATHER_ACK_TASK_H
