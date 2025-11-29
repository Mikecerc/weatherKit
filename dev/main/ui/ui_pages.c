#include "ui_pages.h"
#include "ui_common.h"
#include "ui.h"
#include "display.h"
#include "weather_calc.h"
#include "storm_tracker.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief Draw the main weather page
 */
void ui_draw_main_page(void)
{
    char buf[22];
    const weather_data_t *weather = ui_get_cached_weather();
    const ui_settings_t *settings = ui_get_settings_internal();
    
    float temp = weather->temperature;
    float pressure = weather->pressure;
    const char *temp_unit = "C";
    const char *press_unit = "hPa";
    const char *dist_unit = "km";
    bool imperial = (settings->units == UNITS_IMPERIAL);
    
    // Convert units if needed
    if (imperial) {
        temp = weather_calc_c_to_f(temp);
        pressure = weather_calc_hpa_to_inhg(pressure);
        temp_unit = "F";
        press_unit = "inHg";
        dist_unit = "mi";
    }
    
    // Title - centered "WeatherKit"
    ui_draw_string(29, 2, "WeatherKit");
    
    // Separator line
    ui_draw_hline(0, 12, 128);
    
    // Get storm status for risk and strikes
    storm_status_t status;
    storm_tracker_get_status(&status);
    
    // Temperature
    snprintf(buf, sizeof(buf), "Temp: %.1f%s", temp, temp_unit);
    ui_draw_string(4, 18, buf);
    
    // Humidity
    snprintf(buf, sizeof(buf), "Humid: %.1f%%", weather->humidity);
    ui_draw_string(4, 28, buf);
    
    // Pressure
    if (imperial) {
        snprintf(buf, sizeof(buf), "Press: %.2f%s", pressure, press_unit);
    } else {
        snprintf(buf, sizeof(buf), "Press: %.0f%s", pressure, press_unit);
    }
    ui_draw_string(4, 38, buf);
    
    // Storm risk indicator
    snprintf(buf, sizeof(buf), "Risk: %s", get_risk_string(status.risk_level));
    ui_draw_string(4, 48, buf);
    
    // Nearest strike (right side of risk line)
    if (status.closest_strike_km > 0 && status.closest_strike_km < 100) {
        float closest = status.closest_strike_km;
        if (imperial) {
            closest = weather_calc_km_to_miles(closest);
        }
        snprintf(buf, sizeof(buf), "Zap:%.0f%s", closest, dist_unit);
        ui_draw_string(76, 48, buf);
    }
    
    // Help hint at bottom right
    ui_draw_string(100, 57, "hL:?");
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw the lightning map page
 */
void ui_draw_lightning_map(void)
{
    char buf[16];
    const weather_data_t *weather = ui_get_cached_weather();
    const ui_settings_t *settings = ui_get_settings_internal();
    bool imperial = (settings->units == UNITS_IMPERIAL);
    const char *dist_unit = imperial ? "mi" : "km";
    
    // Max distance for scale (40km or 25mi)
    float max_dist = imperial ? 25.0f : 40.0f;
    int tick_interval = imperial ? 5 : 10;  // 5mi or 10km intervals
    
    // Title
    ui_draw_string(19, 2, "Lightning Map");
    ui_draw_hline(0, 12, 128);
    
    // Number line at y=34, width 118 pixels (5-123)
    int line_y = 34;
    int line_x_start = 5;
    int line_x_end = 123;
    int line_width = line_x_end - line_x_start;
    
    ui_draw_hline(line_x_start, line_y, line_width);
    
    // Tick marks and labels
    int num_ticks = (int)(max_dist / tick_interval);
    for (int i = 0; i <= num_ticks; i++) {
        int val = i * tick_interval;
        int x = line_x_start + (int)(val * line_width / max_dist);
        ui_draw_vline(x, line_y - 2, 5);  // Tick mark
        
        // Label (last one gets unit)
        if (i == num_ticks) {
            snprintf(buf, sizeof(buf), "%d%s", val, dist_unit);
        } else {
            snprintf(buf, sizeof(buf), "%d", val);
        }
        // Center label under tick (approx)
        int label_x = x - 3;
        if (i == 0) label_x = 3;
        if (i == num_ticks) label_x = x - 12;
        ui_draw_string(label_x, line_y + 4, buf);
    }
    
    // Get recent strikes and plot them
    lightning_strike_t strikes[10];
    int count = storm_tracker_get_recent_strikes(strikes, 10);
    
    for (int i = 0; i < count; i++) {
        float dist = strikes[i].distance_km;
        if (imperial) {
            dist = weather_calc_km_to_miles(dist);
        }
        if (dist >= 0 && dist <= max_dist) {
            int x = line_x_start + (int)(dist * line_width / max_dist);
            // Draw small marker
            display_set_pixel(x, line_y - 4, true);
            display_set_pixel(x, line_y - 5, true);
            display_set_pixel(x - 1, line_y - 3, true);
            display_set_pixel(x + 1, line_y - 3, true);
        }
    }
    
    // Info at top: count and latest distance
    snprintf(buf, sizeof(buf), "Cnt:%d", count);
    ui_draw_string(4, 18, buf);
    
    if (weather->lightning_dist >= 0) {
        float dist = weather->lightning_dist;
        if (imperial) {
            dist = weather_calc_km_to_miles(dist);
        }
        snprintf(buf, sizeof(buf), "Now:%.0f%s", dist, dist_unit);
        ui_draw_string(70, 18, buf);
    }
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw the calculated values page
 */
void ui_draw_calculated_page(void)
{
    char buf[22];
    const weather_data_t *weather = ui_get_cached_weather();
    const ui_settings_t *settings = ui_get_settings_internal();
    float temp_c = weather->temperature;
    float humidity = weather->humidity;
    bool imperial = (settings->units == UNITS_IMPERIAL);
    
    // Title
    ui_draw_string(31, 2, "Calculated");
    ui_draw_hline(0, 12, 128);
    
    // Dew point
    float dew_point = weather_calc_dew_point(temp_c, humidity);
    if (imperial) {
        dew_point = weather_calc_c_to_f(dew_point);
        snprintf(buf, sizeof(buf), "Dew Pt: %.1fF", dew_point);
    } else {
        snprintf(buf, sizeof(buf), "Dew Pt: %.1fC", dew_point);
    }
    ui_draw_string(4, 18, buf);
    
    // Heat index
    float heat_idx = weather_calc_heat_index(temp_c, humidity);
    if (imperial) {
        heat_idx = weather_calc_c_to_f(heat_idx);
        snprintf(buf, sizeof(buf), "Heat Idx: %.1fF", heat_idx);
    } else {
        snprintf(buf, sizeof(buf), "Heat Idx: %.1fC", heat_idx);
    }
    ui_draw_string(4, 28, buf);
    
    // Absolute humidity (g/m³ - metric standard, no imperial equivalent)
    float abs_hum = weather_calc_absolute_humidity(temp_c, humidity);
    snprintf(buf, sizeof(buf), "AbsHum: %.1f g/m3", abs_hum);
    ui_draw_string(4, 38, buf);
    
    // Show current temp/humidity being used
    if (imperial) {
        snprintf(buf, sizeof(buf), "(%.0fF, %.0f%%)", 
                 weather_calc_c_to_f(temp_c), humidity);
    } else {
        snprintf(buf, sizeof(buf), "(%.0fC, %.0f%%)", temp_c, humidity);
    }
    ui_draw_string(4, 48, buf);
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw the storm tracker page
 */
void ui_draw_storm_tracker(void)
{
    char buf[22];
    const ui_settings_t *settings = ui_get_settings_internal();
    bool imperial = (settings->units == UNITS_IMPERIAL);
    
    // Title
    ui_draw_string(22, 2, "Storm Tracker");
    ui_draw_hline(0, 12, 128);
    
    // Get storm status
    storm_status_t status;
    storm_tracker_get_status(&status);
    
    // Risk level (prominent)
    snprintf(buf, sizeof(buf), "Risk: %s", get_risk_string(status.risk_level));
    ui_draw_string(4, 18, buf);
    
    // 3-hour pressure change
    if (imperial) {
        float p_change = status.pressure_change_3h * 0.02953f;  // hPa to inHg
        snprintf(buf, sizeof(buf), "Press/3h: %+.2f\"", p_change);
    } else {
        snprintf(buf, sizeof(buf), "Press/3h: %+.1fhPa", status.pressure_change_3h);
    }
    ui_draw_string(4, 28, buf);
    
    // 3-hour humidity change  
    snprintf(buf, sizeof(buf), "Humid/3h: %+.1f%%", status.humidity_change_3h);
    ui_draw_string(4, 38, buf);
    
    // Lightning activity
    snprintf(buf, sizeof(buf), "Zaps/1h: %d", status.lightning_count_1h);
    ui_draw_string(4, 48, buf);
    
    // Approaching indicator
    if (status.storm_approaching) {
        ui_draw_string(75, 48, "COMING!");
    }
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw the sensor status page
 */
void ui_draw_sensor_status(void)
{
    const weather_data_t *weather = ui_get_cached_weather();
    
    // Title
    ui_draw_string(22, 2, "Sensor Status");
    ui_draw_hline(0, 12, 128);
    
    // Remote sensor status
    ui_draw_string(4, 20, "Remote:");
    if (weather->sensor_connected) {
        ui_draw_string(52, 20, "Connected");
    } else {
        ui_draw_string(52, 20, "N/A");
    }
    
    // Placeholder for future LoRa info
    ui_draw_string(4, 32, "LoRa: N/A");
    
    ui_draw_string(4, 44, "(signal,batt TBD)");
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw the settings page
 */
void ui_draw_settings_page(void)
{
    char buf[22];
    const ui_settings_t *settings = ui_get_settings_internal();
    bool edit_mode = ui_is_settings_edit();
    settings_item_t cursor = ui_get_settings_cursor();
    
    // Title
    ui_draw_string(37, 2, "Settings");
    ui_draw_hline(0, 12, 128);
    
    if (edit_mode) {
        // In edit mode: show cursor on current item
        const char *cursor1 = (cursor == SETTINGS_BRIGHTNESS) ? ">" : " ";
        snprintf(buf, sizeof(buf), "%sBright: %d%%", cursor1, settings->brightness);
        ui_draw_string(4, 16, buf);
        
        const char *cursor2 = (cursor == SETTINGS_UNITS) ? ">" : " ";
        const char *unit_str = (settings->units == UNITS_METRIC) ? "Metric" : "Imper.";
        snprintf(buf, sizeof(buf), "%sUnits: %s", cursor2, unit_str);
        ui_draw_string(4, 26, buf);
        
        const char *cursor3 = (cursor == SETTINGS_LORA_TX) ? ">" : " ";
        const char *pwr_str = settings->lora_high_power ? "HIGH" : "Low";
        snprintf(buf, sizeof(buf), "%sLoRa Pwr: %s", cursor3, pwr_str);
        ui_draw_string(4, 36, buf);
        
        const char *cursor4 = (cursor == SETTINGS_ABOUT) ? ">" : " ";
        snprintf(buf, sizeof(buf), "%sAbout", cursor4);
        ui_draw_string(4, 46, buf);
        
        ui_draw_string(4, 57, "L:next R:set hR:done");
    } else {
        // Not in edit mode: no cursor, just show values
        snprintf(buf, sizeof(buf), "  Bright: %d%%", settings->brightness);
        ui_draw_string(4, 16, buf);
        
        const char *unit_str = (settings->units == UNITS_METRIC) ? "Metric" : "Imper.";
        snprintf(buf, sizeof(buf), "  Units: %s", unit_str);
        ui_draw_string(4, 26, buf);
        
        const char *pwr_str = settings->lora_high_power ? "HIGH" : "Low";
        snprintf(buf, sizeof(buf), "  LoRa Pwr: %s", pwr_str);
        ui_draw_string(4, 36, buf);
        
        ui_draw_string(4, 46, "  About");
        
        ui_draw_string(4, 57, "R:edit  hR:back");
    }
    
    ui_draw_page_indicator();
}

/**
 * @brief Draw about screen (shown when About selected)
 */
void ui_draw_about(void)
{
    ui_draw_string(40, 2, "About");
    ui_draw_hline(0, 12, 128);
    
    ui_draw_string(4, 18, "WeatherKit v1.0");
    ui_draw_string(4, 28, "ESP32-S3 Weather");
    ui_draw_string(4, 38, "Station");
    ui_draw_string(4, 50, "Hold R to go back");
}
