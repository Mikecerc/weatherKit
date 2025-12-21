/**
 * @file ui_task.h
 * @brief UI update task for base station
 */

#ifndef UI_TASK_H
#define UI_TASK_H

#include "esp_err.h"

/**
 * @brief Start the UI task
 * Handles UI updates and weather data display
 */
void ui_task_start(void);

/**
 * @brief UI task function
 */
void ui_task(void *pvParameters);

#endif // UI_TASK_H
