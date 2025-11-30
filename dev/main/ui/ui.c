/**
 * @file ui.c
 * @brief Main UI state management and coordination
 */

#include "ui.h"
#include "ui_common.h"
#include "ui_pages.h"
#include "ui_info.h"
#include "display.h"
#include "lora.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ui";

// NVS namespace and keys
#define NVS_NAMESPACE "ui_settings"
#define NVS_KEY_BRIGHTNESS "brightness"
#define NVS_KEY_UNITS "units"

// ============================================================================
// State Variables
// ============================================================================

static ui_page_t current_page = UI_PAGE_MAIN;
static settings_item_t settings_cursor = SETTINGS_BRIGHTNESS;
static bool in_settings_edit = false;
static bool in_about_view = false;
static bool in_info_view = false;
static bool in_lora_confirm = false;  // Confirmation dialog for LoRa high power
static int info_page_num = 0;
static weather_data_t cached_weather = {0};
static ui_settings_t settings = {
    .brightness = 100,
    .units = UNITS_METRIC,
    .refresh_rate = 5,          // Default 5 second refresh
    .locate_enabled = false,    // Locate mode off by default
    .sensor_high_power = false  // Always starts low power for safety
};
static volatile bool refresh_pending = false;
static volatile bool brightness_pending = false;
static bool standby_mode = false;
static volatile bool locate_pending = false;  // Send locate command to sensor

// ============================================================================
// 5x7 Font Table (ASCII 32-127)
// ============================================================================

static const uint8_t font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00,  // 32 (space)
    0x00, 0x00, 0x5F, 0x00, 0x00,  // 33 !
    0x00, 0x07, 0x00, 0x07, 0x00,  // 34 "
    0x14, 0x7F, 0x14, 0x7F, 0x14,  // 35 #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,  // 36 $
    0x23, 0x13, 0x08, 0x64, 0x62,  // 37 %
    0x36, 0x49, 0x55, 0x22, 0x50,  // 38 &
    0x00, 0x05, 0x03, 0x00, 0x00,  // 39 '
    0x00, 0x1C, 0x22, 0x41, 0x00,  // 40 (
    0x00, 0x41, 0x22, 0x1C, 0x00,  // 41 )
    0x08, 0x2A, 0x1C, 0x2A, 0x08,  // 42 *
    0x08, 0x08, 0x3E, 0x08, 0x08,  // 43 +
    0x00, 0x50, 0x30, 0x00, 0x00,  // 44 ,
    0x08, 0x08, 0x08, 0x08, 0x08,  // 45 -
    0x00, 0x60, 0x60, 0x00, 0x00,  // 46 .
    0x20, 0x10, 0x08, 0x04, 0x02,  // 47 /
    0x3E, 0x51, 0x49, 0x45, 0x3E,  // 48 0
    0x00, 0x42, 0x7F, 0x40, 0x00,  // 49 1
    0x42, 0x61, 0x51, 0x49, 0x46,  // 50 2
    0x21, 0x41, 0x45, 0x4B, 0x31,  // 51 3
    0x18, 0x14, 0x12, 0x7F, 0x10,  // 52 4
    0x27, 0x45, 0x45, 0x45, 0x39,  // 53 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,  // 54 6
    0x01, 0x71, 0x09, 0x05, 0x03,  // 55 7
    0x36, 0x49, 0x49, 0x49, 0x36,  // 56 8
    0x06, 0x49, 0x49, 0x29, 0x1E,  // 57 9
    0x00, 0x36, 0x36, 0x00, 0x00,  // 58 :
    0x00, 0x56, 0x36, 0x00, 0x00,  // 59 ;
    0x00, 0x08, 0x14, 0x22, 0x41,  // 60 <
    0x14, 0x14, 0x14, 0x14, 0x14,  // 61 =
    0x41, 0x22, 0x14, 0x08, 0x00,  // 62 >
    0x02, 0x01, 0x51, 0x09, 0x06,  // 63 ?
    0x32, 0x49, 0x79, 0x41, 0x3E,  // 64 @
    0x7E, 0x11, 0x11, 0x11, 0x7E,  // 65 A
    0x7F, 0x49, 0x49, 0x49, 0x36,  // 66 B
    0x3E, 0x41, 0x41, 0x41, 0x22,  // 67 C
    0x7F, 0x41, 0x41, 0x22, 0x1C,  // 68 D
    0x7F, 0x49, 0x49, 0x49, 0x41,  // 69 E
    0x7F, 0x09, 0x09, 0x01, 0x01,  // 70 F
    0x3E, 0x41, 0x41, 0x51, 0x32,  // 71 G
    0x7F, 0x08, 0x08, 0x08, 0x7F,  // 72 H
    0x00, 0x41, 0x7F, 0x41, 0x00,  // 73 I
    0x20, 0x40, 0x41, 0x3F, 0x01,  // 74 J
    0x7F, 0x08, 0x14, 0x22, 0x41,  // 75 K
    0x7F, 0x40, 0x40, 0x40, 0x40,  // 76 L
    0x7F, 0x02, 0x04, 0x02, 0x7F,  // 77 M
    0x7F, 0x04, 0x08, 0x10, 0x7F,  // 78 N
    0x3E, 0x41, 0x41, 0x41, 0x3E,  // 79 O
    0x7F, 0x09, 0x09, 0x09, 0x06,  // 80 P
    0x3E, 0x41, 0x51, 0x21, 0x5E,  // 81 Q
    0x7F, 0x09, 0x19, 0x29, 0x46,  // 82 R
    0x46, 0x49, 0x49, 0x49, 0x31,  // 83 S
    0x01, 0x01, 0x7F, 0x01, 0x01,  // 84 T
    0x3F, 0x40, 0x40, 0x40, 0x3F,  // 85 U
    0x1F, 0x20, 0x40, 0x20, 0x1F,  // 86 V
    0x7F, 0x20, 0x18, 0x20, 0x7F,  // 87 W
    0x63, 0x14, 0x08, 0x14, 0x63,  // 88 X
    0x03, 0x04, 0x78, 0x04, 0x03,  // 89 Y
    0x61, 0x51, 0x49, 0x45, 0x43,  // 90 Z
    0x00, 0x00, 0x7F, 0x41, 0x41,  // 91 [
    0x02, 0x04, 0x08, 0x10, 0x20,  // 92 backslash
    0x41, 0x41, 0x7F, 0x00, 0x00,  // 93 ]
    0x04, 0x02, 0x01, 0x02, 0x04,  // 94 ^
    0x40, 0x40, 0x40, 0x40, 0x40,  // 95 _
    0x00, 0x01, 0x02, 0x04, 0x00,  // 96 `
    0x20, 0x54, 0x54, 0x54, 0x78,  // 97 a
    0x7F, 0x48, 0x44, 0x44, 0x38,  // 98 b
    0x38, 0x44, 0x44, 0x44, 0x20,  // 99 c
    0x38, 0x44, 0x44, 0x48, 0x7F,  // 100 d
    0x38, 0x54, 0x54, 0x54, 0x18,  // 101 e
    0x08, 0x7E, 0x09, 0x01, 0x02,  // 102 f
    0x08, 0x14, 0x54, 0x54, 0x3C,  // 103 g
    0x7F, 0x08, 0x04, 0x04, 0x78,  // 104 h
    0x00, 0x44, 0x7D, 0x40, 0x00,  // 105 i
    0x20, 0x40, 0x44, 0x3D, 0x00,  // 106 j
    0x00, 0x7F, 0x10, 0x28, 0x44,  // 107 k
    0x00, 0x41, 0x7F, 0x40, 0x00,  // 108 l
    0x7C, 0x04, 0x18, 0x04, 0x78,  // 109 m
    0x7C, 0x08, 0x04, 0x04, 0x78,  // 110 n
    0x38, 0x44, 0x44, 0x44, 0x38,  // 111 o
    0x7C, 0x14, 0x14, 0x14, 0x08,  // 112 p
    0x08, 0x14, 0x14, 0x18, 0x7C,  // 113 q
    0x7C, 0x08, 0x04, 0x04, 0x08,  // 114 r
    0x48, 0x54, 0x54, 0x54, 0x20,  // 115 s
    0x04, 0x3F, 0x44, 0x40, 0x20,  // 116 t
    0x3C, 0x40, 0x40, 0x20, 0x7C,  // 117 u
    0x1C, 0x20, 0x40, 0x20, 0x1C,  // 118 v
    0x3C, 0x40, 0x30, 0x40, 0x3C,  // 119 w
    0x44, 0x28, 0x10, 0x28, 0x44,  // 120 x
    0x0C, 0x50, 0x50, 0x50, 0x3C,  // 121 y
    0x44, 0x64, 0x54, 0x4C, 0x44,  // 122 z
    0x00, 0x08, 0x36, 0x41, 0x00,  // 123 {
    0x00, 0x00, 0x7F, 0x00, 0x00,  // 124 |
    0x00, 0x41, 0x36, 0x08, 0x00,  // 125 }
    0x08, 0x08, 0x2A, 0x1C, 0x08,  // 126 ~
    0x08, 0x1C, 0x2A, 0x08, 0x08   // 127
};

// ============================================================================
// Drawing Helpers
// ============================================================================

void ui_draw_char(int x, int y, char c)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = &font5x7[(c - 32) * 5];
    
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                display_set_pixel(x + col, y + row, 1);
            }
        }
    }
}

void ui_draw_string(int x, int y, const char *str)
{
    while (*str) {
        ui_draw_char(x, y, *str++);
        x += 6;  // 5 pixel char + 1 pixel spacing
    }
}

void ui_draw_hline(int x, int y, int w)
{
    for (int i = 0; i < w; i++) {
        display_set_pixel(x + i, y, 1);
    }
}

void ui_draw_vline(int x, int y, int h)
{
    for (int i = 0; i < h; i++) {
        display_set_pixel(x, y + i, 1);
    }
}

// ============================================================================
// Page Indicator
// ============================================================================

void ui_draw_page_indicator(void)
{
    ui_draw_page_indicator_with_cursor(false);
}

void ui_draw_page_indicator_with_cursor(bool show_cursor)
{
    // Draw dots at bottom for each page (excluding settings)
    int dot_spacing = 10;
    int total_width = (UI_PAGE_COUNT - 1) * dot_spacing;
    int start_x = (128 - total_width) / 2;
    int y = 62;
    
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        int x = start_x + i * dot_spacing;
        if (i == (int)current_page) {
            // Filled circle for current page
            display_set_pixel(x, y, 1);
            display_set_pixel(x-1, y, 1);
            display_set_pixel(x+1, y, 1);
            display_set_pixel(x, y-1, 1);
            display_set_pixel(x, y+1, 1);
        } else {
            // Single dot for other pages
            display_set_pixel(x, y, 1);
        }
    }
    
    if (show_cursor) {
        // Show small arrow above current page dot
        int x = start_x + (int)current_page * dot_spacing;
        display_set_pixel(x, y - 4, 1);
        display_set_pixel(x - 1, y - 3, 1);
        display_set_pixel(x + 1, y - 3, 1);
    }
}

// ============================================================================
// Risk String Helper
// ============================================================================

const char* get_risk_string(storm_risk_t risk)
{
    switch (risk) {
        case STORM_RISK_LOW:      return "Low";
        case STORM_RISK_MODERATE: return "Mod";
        case STORM_RISK_HIGH:     return "High";
        case STORM_RISK_SEVERE:   return "Severe";
        default:                  return "None";
    }
}

// ============================================================================
// Internal Accessors (for ui_pages.c and ui_info.c)
// ============================================================================

const weather_data_t* ui_get_cached_weather(void)
{
    return &cached_weather;
}

ui_page_t ui_get_current_page_internal(void)
{
    return current_page;
}

const ui_settings_t* ui_get_settings_internal(void)
{
    return &settings;
}

settings_item_t ui_get_settings_cursor(void)
{
    return settings_cursor;
}

bool ui_is_settings_edit(void)
{
    return in_settings_edit;
}

bool ui_is_about_view(void)
{
    return in_about_view;
}

// ============================================================================
// Display Refresh
// ============================================================================

static void ui_refresh_internal(void)
{
    display_clear();
    
    if (in_lora_confirm) {
        // Draw LoRa high power confirmation dialog
        ui_draw_lora_confirm();
    } else if (in_info_view) {
        // Draw info page for current main page
        ui_draw_info_page(info_page_num);
    } else if (in_about_view) {
        ui_draw_about();
    } else {
        // Draw current main page
        switch (current_page) {
            case UI_PAGE_MAIN:
                ui_draw_main_page();
                break;
            case UI_PAGE_LIGHTNING_MAP:
                ui_draw_lightning_map();
                break;
            case UI_PAGE_CALCULATED:
                ui_draw_calculated_page();
                break;
            case UI_PAGE_STORM_TRACKER:
                ui_draw_storm_tracker();
                break;
            case UI_PAGE_SENSOR_STATUS:
                ui_draw_sensor_status();
                break;
            case UI_PAGE_SETTINGS:
                ui_draw_settings_page();
                break;
            default:
                break;
        }
    }
    
    display_refresh();
}

void ui_check_refresh(void)
{
    // Handle deferred brightness change
    if (brightness_pending) {
        brightness_pending = false;
        display_set_brightness(settings.brightness);
    }
    
    // Handle deferred refresh
    if (refresh_pending) {
        refresh_pending = false;
        ui_refresh_internal();
    }
}

// ============================================================================
// Navigation Functions
// ============================================================================

void ui_cycle(void)
{
    if (in_lora_confirm) {
        // LEFT in confirm dialog = Cancel
        in_lora_confirm = false;
        // Stay in settings edit mode
    } else if (in_info_view) {
        // Cycle through info pages
        int max_pages = ui_info_get_page_count();
        info_page_num = (info_page_num + 1) % max_pages;
    } else if (in_about_view) {
        // No cycling in about view
    } else if (in_settings_edit) {
        // In edit mode: LEFT cycles through menu items
        settings_cursor = (settings_cursor + 1) % SETTINGS_COUNT;
    } else {
        // Not in any special mode: LEFT cycles main pages
        current_page = (current_page + 1) % UI_PAGE_COUNT;
    }
    refresh_pending = true;
}

void ui_select(void)
{
    if (in_lora_confirm) {
        // RIGHT in confirm dialog = Enable high power
        settings.sensor_high_power = true;
        in_lora_confirm = false;
        ESP_LOGW(TAG, "Sensor HIGH POWER enabled - ensure antenna is connected!");
        // TODO: Send config to sensor
    } else if (in_info_view) {
        // Exit info view
        in_info_view = false;
        info_page_num = 0;
    } else if (in_about_view) {
        // Exit about view
        in_about_view = false;
    } else if (current_page == UI_PAGE_SETTINGS) {
        if (in_settings_edit) {
            // Already in edit mode: RIGHT activates current item
            switch (settings_cursor) {
                case SETTINGS_BRIGHTNESS:
                    // Cycle brightness value
                    settings.brightness += 25;
                    if (settings.brightness > 100) settings.brightness = 25;
                    brightness_pending = true;
                    break;
                case SETTINGS_UNITS:
                    // Toggle units
                    settings.units = (settings.units == UNITS_METRIC) ? 
                                     UNITS_IMPERIAL : UNITS_METRIC;
                    break;
                case SETTINGS_REFRESH_RATE:
                    // Cycle refresh rate: 5, 10, 15, 30, 60 seconds
                    if (settings.refresh_rate < 10) settings.refresh_rate = 10;
                    else if (settings.refresh_rate < 15) settings.refresh_rate = 15;
                    else if (settings.refresh_rate < 30) settings.refresh_rate = 30;
                    else if (settings.refresh_rate < 60) settings.refresh_rate = 60;
                    else settings.refresh_rate = 5;
                    ESP_LOGI(TAG, "Refresh rate: %ds", settings.refresh_rate);
                    // TODO: Send config to sensor
                    break;
                case SETTINGS_LOCATE:
                    // Toggle locate - send command to sensor
                    settings.locate_enabled = !settings.locate_enabled;
                    locate_pending = true;
                    ESP_LOGI(TAG, "Locate: %s", settings.locate_enabled ? "ON" : "OFF");
                    break;
                case SETTINGS_HIGH_POWER:
                    if (settings.sensor_high_power) {
                        // Turning OFF - no confirmation needed
                        settings.sensor_high_power = false;
                        ESP_LOGI(TAG, "Sensor power: LOW");
                        // TODO: Send config to sensor
                    } else {
                        // Turning ON - show confirmation dialog
                        in_lora_confirm = true;
                    }
                    break;
                case SETTINGS_ABOUT:
                    // Enter about view
                    in_about_view = true;
                    in_settings_edit = false;
                    break;
                default:
                    break;
            }
        } else {
            // Not in edit mode: RIGHT enters edit mode
            in_settings_edit = true;
            settings_cursor = SETTINGS_BRIGHTNESS;  // Start at first item
        }
    }
    // Other pages: select does nothing for now
    refresh_pending = true;
}

void ui_back(void)
{
    if (in_lora_confirm) {
        // Cancel confirmation
        in_lora_confirm = false;
    } else if (in_info_view) {
        // Exit info view
        in_info_view = false;
        info_page_num = 0;
    } else if (in_about_view) {
        // Exit about view, back to settings edit
        in_about_view = false;
        in_settings_edit = true;
    } else if (in_settings_edit) {
        // Exit edit mode - save settings to flash
        in_settings_edit = false;
        ui_save_settings();
    } else if (current_page == UI_PAGE_SETTINGS) {
        // Not in edit mode on settings: go back to main page
        current_page = UI_PAGE_MAIN;
    }
    // Other contexts: back does nothing
    refresh_pending = true;
}

void ui_show_info(void)
{
    // Info is only available for non-settings pages
    if (current_page != UI_PAGE_SETTINGS && !in_about_view) {
        in_info_view = true;
        info_page_num = 0;
        refresh_pending = true;
    }
}

// ============================================================================
// Weather Update
// ============================================================================

void ui_update_weather(const weather_data_t *data)
{
    if (data) {
        memcpy(&cached_weather, data, sizeof(weather_data_t));
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t ui_init(void)
{
    current_page = UI_PAGE_MAIN;
    settings_cursor = SETTINGS_BRIGHTNESS;
    in_settings_edit = false;
    in_about_view = false;
    in_info_view = false;
    info_page_num = 0;
    refresh_pending = false;
    brightness_pending = false;
    
    // Initialize with default weather data
    cached_weather.temperature = 0.0f;
    cached_weather.humidity = 0.0f;
    cached_weather.pressure = 0.0f;
    cached_weather.lightning_dist = -1.0f;
    cached_weather.sensor_connected = false;
    
    // Initial display
    ui_refresh_internal();
    
    return ESP_OK;
}

ui_page_t ui_get_current_page(void)
{
    return current_page;
}

void ui_set_page(ui_page_t page)
{
    if (page < UI_PAGE_COUNT) {
        current_page = page;
        in_info_view = false;
        in_about_view = false;
        refresh_pending = true;
    }
}

void ui_refresh(void)
{
    ui_refresh_internal();
}

const ui_settings_t* ui_get_settings(void)
{
    return &settings;
}

bool ui_is_sensor_high_power(void)
{
    return settings.sensor_high_power;
}

bool ui_is_locate_enabled(void)
{
    return settings.locate_enabled;
}

uint8_t ui_get_refresh_rate(void)
{
    return settings.refresh_rate;
}

bool ui_check_locate_pending(void)
{
    bool pending = locate_pending;
    locate_pending = false;
    return pending;
}

// ============================================================================
// Settings Persistence (NVS)
// ============================================================================

esp_err_t ui_save_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_u8(handle, NVS_KEY_BRIGHTNESS, settings.brightness);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save brightness: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    err = nvs_set_u8(handle, NVS_KEY_UNITS, (uint8_t)settings.units);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save units: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Settings saved: brightness=%d, units=%d", 
                 settings.brightness, settings.units);
    }
    
    nvs_close(handle);
    return err;
}

esp_err_t ui_load_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No saved settings found, using defaults");
        } else {
            ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        }
        return err;
    }
    
    uint8_t brightness = 100;
    uint8_t units = UNITS_METRIC;
    
    err = nvs_get_u8(handle, NVS_KEY_BRIGHTNESS, &brightness);
    if (err == ESP_OK) {
        settings.brightness = brightness;
    }
    
    err = nvs_get_u8(handle, NVS_KEY_UNITS, &units);
    if (err == ESP_OK) {
        settings.units = (unit_system_t)units;
    }
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Settings loaded: brightness=%d, units=%d", 
             settings.brightness, settings.units);
    
    return ESP_OK;
}

// ============================================================================
// Standby Mode
// ============================================================================

void ui_enter_standby(void)
{
    if (standby_mode) return;
    
    standby_mode = true;
    display_sleep();
    ESP_LOGI(TAG, "Entering standby mode");
}

void ui_exit_standby(void)
{
    if (!standby_mode) return;
    
    standby_mode = false;
    display_wake();
    
    // Restore brightness
    display_set_brightness(settings.brightness);
    
    // Force refresh to redraw screen
    refresh_pending = true;
    ESP_LOGI(TAG, "Exiting standby mode");
}

bool ui_is_standby(void)
{
    return standby_mode;
}

void ui_toggle_standby(void)
{
    if (standby_mode) {
        ui_exit_standby();
    } else {
        ui_enter_standby();
    }
}