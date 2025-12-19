#include "ui_info.h"
#include "ui_common.h"
#include "ui.h"
#include "display.h"
#include <stdio.h>

// Info page counts per main page
static const int info_page_counts[] = {
    2,  // UI_PAGE_MAIN - 2 pages
    2,  // UI_PAGE_LIGHTNING_MAP - 2 pages
    2,  // UI_PAGE_CALCULATED - 2 pages
    3,  // UI_PAGE_STORM_TRACKER - 3 pages (algorithm explanation)
    2,  // UI_PAGE_SENSOR_STATUS - 2 pages (sensors + LoRa info)
    0,  // UI_PAGE_SETTINGS - no info (has About already)
};

/**
 * @brief Draw info page header and page indicator
 */
static void ui_draw_info_header(const char *title, int page_num, int total_pages)
{
    ui_draw_string(4, 2, title);
    
    // Page indicator on right (only if multiple pages)
    if (total_pages > 1 && total_pages <= 9) {
        char buf[8];
        buf[0] = '0' + (char)(page_num + 1);
        buf[1] = '/';
        buf[2] = '0' + (char)total_pages;
        buf[3] = '\0';
        ui_draw_string(110, 2, buf);
    }
    
    ui_draw_hline(0, 12, 128);
}

/**
 * @brief Draw info footer with navigation hint
 */
static void ui_draw_info_footer(int total_pages)
{
    if (total_pages > 1) {
        ui_draw_string(4, 57, "R:next  L:exit");
    } else {
        ui_draw_string(4, 57, "L:exit");
    }
}

// ============================================================================
// Main Page Info
// ============================================================================

static void ui_draw_main_info_page0(void)
{
    ui_draw_info_header("Main Info", 0, 2);
    
    ui_draw_string(4, 16, "Temp: Temperature");
    ui_draw_string(4, 26, "Humid: Relative %");
    ui_draw_string(4, 36, "Press: Barometric");
    ui_draw_string(4, 46, "Zap: Nearest strike");
    
    ui_draw_info_footer(2);
}

static void ui_draw_main_info_page1(void)
{
    ui_draw_info_header("Main Info", 1, 2);
    
    ui_draw_string(4, 16, "Risk: Storm level");
    ui_draw_string(4, 26, "LOW/MOD/HIGH/SEVERE");
    ui_draw_string(4, 36, "Based on pressure,");
    ui_draw_string(4, 46, "humidity & strikes.");
    
    ui_draw_info_footer(2);
}

// ============================================================================
// Lightning Map Info
// ============================================================================

static void ui_draw_lightning_info_page0(void)
{
    ui_draw_info_header("Lightning", 0, 2);
    
    ui_draw_string(4, 16, "Shows recent strikes");
    ui_draw_string(4, 26, "on distance scale");
    ui_draw_string(4, 36, "(0-40km or 0-25mi).");
    ui_draw_string(4, 46, "Cnt: Total tracked");
    
    ui_draw_info_footer(2);
}

static void ui_draw_lightning_info_page1(void)
{
    ui_draw_info_header("Lightning", 1, 2);
    
    ui_draw_string(4, 16, "Now: Most recent");
    ui_draw_string(4, 26, "strike distance.");
    ui_draw_string(4, 36, "Markers show where");
    ui_draw_string(4, 46, "strikes occurred.");
    
    ui_draw_info_footer(2);
}

// ============================================================================
// Calculated Values Info
// ============================================================================

static void ui_draw_calc_info_page0(void)
{
    ui_draw_info_header("Calculated", 0, 2);
    
    ui_draw_string(4, 16, "Dew Pt: Temperature");
    ui_draw_string(4, 26, "where condensation");
    ui_draw_string(4, 36, "forms. Uses Magnus");
    ui_draw_string(4, 46, "approximation.");
    
    ui_draw_info_footer(2);
}

static void ui_draw_calc_info_page1(void)
{
    ui_draw_info_header("Calculated", 1, 2);
    
    ui_draw_string(4, 16, "Heat Idx: Feels-like");
    ui_draw_string(4, 26, "temp with humidity.");
    ui_draw_string(4, 36, "AbsHum: Water vapor");
    ui_draw_string(4, 46, "density in g/m3.");
    
    ui_draw_info_footer(2);
}

// ============================================================================
// Storm Tracker Info
// ============================================================================

static void ui_draw_storm_info_page0(void)
{
    ui_draw_info_header("Storm Algo", 0, 3);
    
    ui_draw_string(4, 16, "Risk calculated from:");
    ui_draw_string(4, 26, "1.Pressure drop 3h");
    ui_draw_string(4, 36, "2.Humidity rise 3h");
    ui_draw_string(4, 46, "3.Lightning activity");
    
    ui_draw_info_footer(3);
}

static void ui_draw_storm_info_page1(void)
{
    ui_draw_info_header("Storm Algo", 1, 3);
    
    ui_draw_string(4, 16, "Pressure thresholds:");
    ui_draw_string(4, 26, ">3hPa drop = MOD");
    ui_draw_string(4, 36, ">6hPa drop = HIGH");
    ui_draw_string(4, 46, ">10hPa = SEVERE");
    
    ui_draw_info_footer(3);
}

static void ui_draw_storm_info_page2(void)
{
    ui_draw_info_header("Storm Algo", 2, 3);
    
    ui_draw_string(4, 16, "COMING! shown when:");
    ui_draw_string(4, 26, "Lightning getting");
    ui_draw_string(4, 36, "closer over time");
    ui_draw_string(4, 46, "(decreasing dist).");
    
    ui_draw_info_footer(3);
}

// ============================================================================
// Sensor Status Info
// ============================================================================

static void ui_draw_sensor_info_page0(void)
{
    ui_draw_info_header("Sensors", 0, 2);
    
    ui_draw_string(4, 16, "Remote: External");
    ui_draw_string(4, 26, "sensor via LoRa.");
    ui_draw_string(4, 36, "RSSI: Signal strength");
    ui_draw_string(4, 46, "SNR: Signal quality.");
    
    ui_draw_info_footer(2);
}

static void ui_draw_sensor_info_page1(void)
{
    ui_draw_info_header("Sensors", 1, 2);
    
    ui_draw_string(4, 16, "LoRa: 915MHz radio");
    ui_draw_string(4, 26, "Power: Lo(safe) or");
    ui_draw_string(4, 36, "Hi(needs antenna!)");
    ui_draw_string(4, 46, "TX/RX: packet counts");
    
    ui_draw_info_footer(2);
}

// ============================================================================
// Public Functions
// ============================================================================

int ui_info_get_page_count(void)
{
    ui_page_t page = ui_get_current_page_internal();
    if (page < sizeof(info_page_counts) / sizeof(info_page_counts[0])) {
        return info_page_counts[page];
    }
    return 0;
}

void ui_draw_info_page(int info_page_num)
{
    ui_page_t page = ui_get_current_page_internal();
    
    display_clear();
    
    switch (page) {
        case UI_PAGE_MAIN:
            if (info_page_num == 0) ui_draw_main_info_page0();
            else ui_draw_main_info_page1();
            break;
            
        case UI_PAGE_LIGHTNING_MAP:
            if (info_page_num == 0) ui_draw_lightning_info_page0();
            else ui_draw_lightning_info_page1();
            break;
            
        case UI_PAGE_CALCULATED:
            if (info_page_num == 0) ui_draw_calc_info_page0();
            else ui_draw_calc_info_page1();
            break;
            
        case UI_PAGE_STORM_TRACKER:
            if (info_page_num == 0) ui_draw_storm_info_page0();
            else if (info_page_num == 1) ui_draw_storm_info_page1();
            else ui_draw_storm_info_page2();
            break;
            
        case UI_PAGE_SENSOR_STATUS:
            if (info_page_num == 0) ui_draw_sensor_info_page0();
            else ui_draw_sensor_info_page1();
            break;
            
        default:
            // Settings has no info pages
            break;
    }
    
    display_refresh();
}
