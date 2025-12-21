#ifndef UI_COMMON_H
#define UI_COMMON_H

#include "ui.h"
#include "display.h"
#include "weather_calc.h"
#include "storm_tracker.h"
#include <stdio.h>
#include <stdbool.h>

/**
 * @brief Draw a character at position using 5x7 font
 */
void ui_draw_char(int x, int y, char c);

/**
 * @brief Draw a string at position
 */
void ui_draw_string(int x, int y, const char *str);

/**
 * @brief Draw a horizontal line
 */
void ui_draw_hline(int x, int y, int w);

/**
 * @brief Draw a vertical line
 */
void ui_draw_vline(int x, int y, int h);

/**
 * @brief Draw page indicator
 */
void ui_draw_page_indicator(void);

/**
 * @brief Draw page indicator with optional cursor
 */
void ui_draw_page_indicator_with_cursor(bool show_cursor);

/**
 * @brief Get storm risk as string
 */
const char* get_risk_string(storm_risk_t risk);

/**
 * @brief Get cached weather data (read-only) - internal use
 */
const weather_data_t* ui_get_cached_weather(void);

/**
 * @brief Get current page - internal use
 */
ui_page_t ui_get_current_page_internal(void);

/**
 * @brief Get current settings - internal use
 */
const ui_settings_t* ui_get_settings_internal(void);

/**
 * @brief Get settings cursor position
 */
settings_item_t ui_get_settings_cursor(void);

/**
 * @brief Check if in settings edit mode
 */
bool ui_is_settings_edit(void);

/**
 * @brief Check if in about view mode
 */
bool ui_is_about_view(void);

/**
 * @brief Get sensor status page scroll position
 */
int ui_get_sensor_status_scroll(void);

#endif // UI_COMMON_H
