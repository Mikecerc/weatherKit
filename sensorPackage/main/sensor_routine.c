/**
 * @file sensor_routine.c
 * @brief Sensor routine for WeatherKit Sensor Package
 * 
 * Handles periodic sensor reading and transmission to base station.
 * Supports both real sensor data and fake data for testing.
 */

#include "sensor_routine.h"
#include "drivers/lora.h"
#include "drivers/lora_protocol.h"
#include "drivers/led.h"
#include "drivers/i2c_init.h"
#include "drivers/temp_humidity_driver.h"
#include "drivers/pressure_driver.h"
#include "drivers/lightning_driver.h"
#include "pinout.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "sensor";

// =============================================================================
// State Variables
// =============================================================================

static sensor_config_t current_config;
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t led_status_task_handle = NULL;
static bool is_running = false;
static SemaphoreHandle_t config_mutex = NULL;

// LED status state
static volatile bool led_flash_packet_sent = false;
static volatile bool led_flash_packet_failed = false;
static volatile bool led_flash_packet_received = false;
static volatile bool led_using_fake_data = false;  // Track if using fake or real data

// Statistics
static uint32_t packets_sent = 0;
static uint32_t packets_acked = 0;
static uint32_t packets_failed = 0;

// Timing
static int64_t start_time_us = 0;
static int64_t last_weather_send_us = 0;
static int64_t last_heartbeat_send_us = 0;

// Fake sensor state (for simulating realistic data when use_fake_data=true)
static float fake_temp = 22.5f;          // Starting temperature (°C)
static float fake_humidity = 45.0f;      // Starting humidity (%)
static float fake_pressure = 1013.25f;   // Starting pressure (hPa)

// Sensor error flags
static uint8_t sensor_error_flags = 0;

// Lightning data accumulator (collects strikes between transmissions)
static lightning_data_t accumulated_lightning = {0};

// Forward declaration for LED signal function
static void led_signal_packet_received(void);

// =============================================================================
// Callbacks
// =============================================================================

/**
 * @brief Called when configuration message received from base
 */
static void on_config_received(const config_payload_t *config)
{
    ESP_LOGI(TAG, "Config received: interval=%ds, heartbeat=%ds, power=%d",
             config->update_interval, config->heartbeat_interval, config->tx_power);
    
    // Signal LED for received packet
    led_signal_packet_received();
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update intervals if valid
        if (config->update_interval >= MIN_UPDATE_INTERVAL_SEC && 
            config->update_interval <= MAX_UPDATE_INTERVAL_SEC) {
            current_config.update_interval_sec = config->update_interval;
        }
        
        if (config->heartbeat_interval > 0) {
            current_config.heartbeat_interval_sec = config->heartbeat_interval;
        }
        
        // Update adaptive power setting
        current_config.adaptive_power = (config->flags & CFG_ADAPTIVE_POWER) != 0;
        lora_set_adaptive_power(current_config.adaptive_power);
        
        // If adaptive power is off, use the specified power level
        if (!current_config.adaptive_power && config->tx_power >= TX_POWER_MIN) {
            lora_set_tx_power(config->tx_power);
        }
        
        xSemaphoreGive(config_mutex);
    }
}

/**
 * @brief Called when ACK received from base
 */
static void on_ack_received(const ack_payload_t *ack)
{
    ESP_LOGD(TAG, "ACK received: seq=%d, rssi=%d, suggested_power=%d",
             ack->acked_sequence, ack->rssi, ack->suggested_power);
    
    // Signal LED for received packet
    led_signal_packet_received();
    
    packets_acked++;
}

/**
 * @brief Called when ping received from base
 */
static void on_ping_received(uint8_t ping_type)
{
    ESP_LOGI(TAG, "Ping received: type=%d", ping_type);
    
    if (ping_type == 1) {
        // Locate request - flash LED blue
        ESP_LOGI(TAG, "LOCATE request - flashing LED");
        led_flash_locate(3000);  // Flash for 3 seconds
    }
}

// =============================================================================
// LED Status Task
// =============================================================================

/**
 * @brief LED status indicator task
 * 
 * Manages LED state based on LoRa connection and packet status:
 * - Flash red: LoRa offline
 * - Solid green: LoRa online, using real sensor data
 * - Solid blue: LoRa online, using fake data
 * - Bright green/blue flash: Packet sent successfully (color matches data mode)
 * - Red flash: Packet send failed
 * - Purple flash: Packet/config received from base
 */
static void led_status_task(void *arg)
{
    ESP_LOGI(TAG, "LED status task started");
    
    bool led_on = true;  // Start with LED on
    int flash_counter = 0;
    
    // Start with LED off until we know the state
    led_off();
    
    while (is_running) {
        // Check for one-shot packet status flashes (highest priority)
        if (led_flash_packet_sent) {
            led_flash_packet_sent = false;
            // Bright flash for successful send - green for real data, blue for fake
            if (led_using_fake_data) {
                led_set_color(0, 0, 255);  // Bright blue (fake data)
            } else {
                led_set_color(0, 255, 0);  // Bright green (real data)
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            // Continue to set normal state below
        }
        
        if (led_flash_packet_failed) {
            led_flash_packet_failed = false;
            // Red flash for failed send
            led_set_color(255, 0, 0);  // Red
            vTaskDelay(pdMS_TO_TICKS(150));
            // Continue to set normal state below
        }
        
        if (led_flash_packet_received) {
            led_flash_packet_received = false;
            // Purple flash for received packet/config
            led_set_color(128, 0, 255);  // Purple
            vTaskDelay(pdMS_TO_TICKS(100));
            // Continue to set normal state below
        }
        
        // Handle base LED status based on LoRa state
        if (lora_is_initialized()) {
            // LoRa online - solid color (green for real data, blue for fake)
            if (led_using_fake_data) {
                led_set_color(0, 0, 50);  // Dim blue (fake data mode)
            } else {
                led_set_color(0, 50, 0);  // Dim green (real data mode)
            }
            flash_counter = 0;
            led_on = true;
        } else {
            // LoRa offline - flash red (500ms on, 500ms off)
            flash_counter++;
            if (flash_counter >= 10) {  // Toggle every 500ms (10 * 50ms)
                flash_counter = 0;
                led_on = !led_on;
            }
            
            // Always set the LED state for offline mode
            if (led_on) {
                led_set_color(255, 0, 0);  // Red
            } else {
                led_set_color(0, 0, 0);    // Off (explicit black, not led_off)
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Turn off LED when stopping
    led_off();
    ESP_LOGI(TAG, "LED status task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Signal that a packet was sent successfully
 */
static void led_signal_packet_sent(void)
{
    led_flash_packet_sent = true;
}

/**
 * @brief Signal that a packet failed to send
 */
static void led_signal_packet_failed(void)
{
    led_flash_packet_failed = true;
}

/**
 * @brief Signal that a packet was received
 */
static void led_signal_packet_received(void)
{
    led_flash_packet_received = true;
}

// =============================================================================
// Fake Sensor Data Generation (for testing without hardware)
// =============================================================================

/**
 * @brief Generate fake but realistic-looking weather data
 * 
 * Simulates gradual changes with some randomness to look like
 * real sensor readings.
 */
static void generate_fake_weather_data(weather_payload_t *weather)
{
    // Add small random variations to simulate real sensor drift
    // Temperature: varies ±0.1°C per reading, bounded to reasonable range
    fake_temp += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_temp < -20.0f) fake_temp = -20.0f;
    if (fake_temp > 50.0f) fake_temp = 50.0f;
    
    // Humidity: varies ±0.5% per reading, bounded 0-100%
    fake_humidity += ((float)(esp_random() % 100) - 50.0f) / 100.0f;
    if (fake_humidity < 0.0f) fake_humidity = 0.0f;
    if (fake_humidity > 100.0f) fake_humidity = 100.0f;
    
    // Pressure: varies ±0.1 hPa per reading, bounded to reasonable range
    fake_pressure += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_pressure < 950.0f) fake_pressure = 950.0f;
    if (fake_pressure > 1050.0f) fake_pressure = 1050.0f;
    
    // Encode values
    weather->temperature = encode_temperature(fake_temp);
    weather->humidity = encode_humidity(fake_humidity);
    weather->pressure = encode_pressure(fake_pressure);
    
    // No lightning for fake data
    lightning_data_init(&weather->lightning);
    
    // Include link quality info
    weather->rssi = lora_get_last_rssi();
    weather->tx_power = lora_get_current_power();
}

// =============================================================================
// Real Sensor Data Reading
// =============================================================================

/**
 * @brief Read real weather data from sensors
 * 
 * Reads from AHT20 (temp/humidity), HX710B (pressure), and AS3935 (lightning)
 */
static void read_real_weather_data(weather_payload_t *weather)
{
    float temperature = 0.0f;
    float humidity = 0.0f;
    float pressure = 0.0f;  // Default to 0 on error (not fake data)
    
    // Clear error flags for this reading
    sensor_error_flags = 0;
    
    // Read temperature and humidity from AHT20
    aht20_data_t aht20_data = {0};
    esp_err_t err = aht20_read(&aht20_data);
    if (err == ESP_OK && aht20_data.valid) {
        temperature = aht20_data.temperature_c;
        humidity = aht20_data.humidity_rh;
        ESP_LOGI(TAG, "AHT20: T=%.2f°C, RH=%.2f%%", temperature, humidity);
    } else {
        ESP_LOGE(TAG, "AHT20 read failed: %s - sending 0 values", esp_err_to_name(err));
        sensor_error_flags |= ERR_TEMP_SENSOR | ERR_HUMIDITY_SENSOR;
        // Leave temperature and humidity at 0 - don't use fake data
    }
    
    // Read pressure from HX710B
    hx710b_data_t hx710b_data = {0};
    err = hx710b_read(&hx710b_data);
    if (err == ESP_OK && hx710b_data.valid) {
        pressure = hx710b_data.pressure_hpa;
        ESP_LOGI(TAG, "HX710B: P=%.2f hPa", pressure);
    } else {
        ESP_LOGE(TAG, "HX710B read failed: %s - sending 0 value", esp_err_to_name(err));
        sensor_error_flags |= ERR_PRESSURE_SENSOR;
        // Leave pressure at 0 - don't use fake data
    }
    
    // Encode values into packet (will be 0 if sensor failed)
    weather->temperature = encode_temperature(temperature);
    weather->humidity = encode_humidity(humidity);
    weather->pressure = encode_pressure(pressure);
    
    // Copy accumulated lightning data and reset
    memcpy(&weather->lightning, &accumulated_lightning, sizeof(lightning_data_t));
    lightning_data_init(&accumulated_lightning);
    
    // Include link quality info
    weather->rssi = lora_get_last_rssi();
    weather->tx_power = lora_get_current_power();
    
    ESP_LOGI(TAG, "Weather: T=%.2f°C, RH=%.2f%%, P=%.2f hPa, Lightning=%d strikes",
             decode_temperature(weather->temperature),
             decode_humidity(weather->humidity),
             decode_pressure(weather->pressure),
             weather->lightning.count);
}

/**
 * @brief Log weather data to monitor (for debugging without LoRa)
 */
static void log_weather_data(const weather_payload_t *weather)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "         SENSOR DATA OUTPUT");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Temperature:  %.2f °C", decode_temperature(weather->temperature));
    ESP_LOGI(TAG, "Humidity:     %.2f %%", decode_humidity(weather->humidity));
    ESP_LOGI(TAG, "Pressure:     %.2f hPa", decode_pressure(weather->pressure));
    ESP_LOGI(TAG, "Lightning:    %d strikes", weather->lightning.count);
    if (weather->lightning.count > 0) {
        ESP_LOGI(TAG, "  Closest:    %d km", lightning_data_closest(&weather->lightning));
    }
    ESP_LOGI(TAG, "RSSI:         %d dBm", weather->rssi);
    ESP_LOGI(TAG, "TX Power:     %d dBm", weather->tx_power);
    ESP_LOGI(TAG, "Errors:       0x%02X", sensor_error_flags);
    ESP_LOGI(TAG, "========================================");
}

/**
 * @brief Generate sensor status payload
 */
static void generate_status_data(status_payload_t *status_data)
{
    // Fake battery data (would come from ADC in real implementation)
    status_data->battery_mv = 3700 + (esp_random() % 200);  // 3.7-3.9V
    status_data->battery_percent = 85 + (esp_random() % 15); // 85-100%
    
    // Link quality
    status_data->rssi = lora_get_last_rssi();
    status_data->snr = 0;  // TODO: Get from LoRa status
    status_data->tx_power = lora_get_current_power();
    
    // Uptime
    status_data->uptime_min = sensor_routine_get_uptime_min();
    
    // Current configuration
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status_data->update_interval = current_config.update_interval_sec;
        xSemaphoreGive(config_mutex);
    }
    
    // Report actual sensor errors
    status_data->error_flags = sensor_error_flags;
    
    ESP_LOGI(TAG, "Status: battery=%dmV (%d%%), uptime=%d min, errors=0x%02X",
             status_data->battery_mv, status_data->battery_percent, 
             status_data->uptime_min, status_data->error_flags);
}

// =============================================================================
// Sensor Task
// =============================================================================

/**
 * @brief Main sensor routine task
 */
static void sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor task started");
    
    // Initialize timing
    start_time_us = esp_timer_get_time();
    last_weather_send_us = 0;
    last_heartbeat_send_us = 0;
    
    // Put LoRa in receive mode initially
    lora_receive();
    
    while (is_running) {
        int64_t now_us = esp_timer_get_time();
        
        // Get current intervals
        uint16_t weather_interval_sec;
        uint16_t heartbeat_interval_sec;
        
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            weather_interval_sec = current_config.update_interval_sec;
            heartbeat_interval_sec = current_config.heartbeat_interval_sec;
            xSemaphoreGive(config_mutex);
        } else {
            weather_interval_sec = DEFAULT_UPDATE_INTERVAL_SEC;
            heartbeat_interval_sec = DEFAULT_HEARTBEAT_INTERVAL_SEC;
        }
        
        // Check for incoming packets (only if LoRa is initialized)
        if (lora_is_initialized()) {
            if (lora_received()) {
                // Only log if we actually got a valid packet
                if (lora_process_rx()) {
                    ESP_LOGI(TAG, ">>> Valid packet received from base!");
                }
                lora_receive();  // Go back to RX mode
            }
        }
        
        // Check if it's time to send weather data
        int64_t weather_interval_us = (int64_t)weather_interval_sec * 1000000LL;
        if ((now_us - last_weather_send_us) >= weather_interval_us) {
            ESP_LOGI(TAG, "Reading sensors...");
            
            weather_payload_t weather = {0};
            
            // Choose between real and fake data based on config
            bool use_fake = false;
            if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                use_fake = current_config.use_fake_data;
                xSemaphoreGive(config_mutex);
            }
            
            // Update LED color mode based on data source
            led_using_fake_data = use_fake;
            
            if (use_fake) {
                generate_fake_weather_data(&weather);
            } else {
                read_real_weather_data(&weather);
            }
            
            // Log sensor data to monitor (for debugging without LoRa)
            log_weather_data(&weather);
            
            // Attempt to send via LoRa (only if initialized)
            if (lora_is_initialized()) {
                esp_err_t err = lora_send_weather(&weather);
                if (err == ESP_OK) {
                    packets_sent++;
                    led_signal_packet_sent();  // Flash bright green/blue based on data mode
                    ESP_LOGI(TAG, "Weather packet sent (total=%lu)", (unsigned long)packets_sent);
                } else {
                    packets_failed++;
                    led_signal_packet_failed();  // Flash red
                    ESP_LOGW(TAG, "LoRa send failed: %s", esp_err_to_name(err));
                }
                
                // Go back to receive mode to listen for ACK
                lora_receive();
            } else {
                ESP_LOGI(TAG, "(LoRa not available - data logged only)");
            }
            
            last_weather_send_us = now_us;
        }
        
        // Check if it's time to send heartbeat/status
        int64_t heartbeat_interval_us = (int64_t)heartbeat_interval_sec * 1000000LL;
        if ((now_us - last_heartbeat_send_us) >= heartbeat_interval_us) {
            ESP_LOGI(TAG, "Status heartbeat...");
            
            status_payload_t status_data = {0};
            generate_status_data(&status_data);
            
            if (lora_is_initialized()) {
                esp_err_t err = lora_send_sensor_status(&status_data);
                if (err == ESP_OK) {
                    led_signal_packet_sent();  // Flash bright blue
                    ESP_LOGI(TAG, "Status packet sent");
                } else {
                    led_signal_packet_failed();  // Flash red
                    ESP_LOGW(TAG, "Status packet send failed: %s", esp_err_to_name(err));
                }
                
                // Go back to receive mode
                lora_receive();
            }
            
            last_heartbeat_send_us = now_us;
        }
        
        // Ensure we're in receive mode (only if LoRa is initialized)
        if (lora_is_initialized()) {
            lora_receive();
        }
        
        // Small delay - sensor spends most time listening for base commands
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "Sensor task stopped");
    lora_sleep();
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t sensor_routine_init(const sensor_config_t *config)
{
    ESP_LOGI(TAG, "Initializing sensor routine...");
    
    // Create mutex for config access
    if (config_mutex == NULL) {
        config_mutex = xSemaphoreCreateMutex();
        if (config_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create config mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Set configuration
    if (config != NULL) {
        memcpy(&current_config, config, sizeof(sensor_config_t));
    } else {
        sensor_config_t default_config = SENSOR_CONFIG_DEFAULT();
        memcpy(&current_config, &default_config, sizeof(sensor_config_t));
    }
    
    // Initialize LED for locate function
    esp_err_t err = led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize LED: %s (continuing anyway)", esp_err_to_name(err));
        // Don't fail - LED is optional
    }
    
    // Initialize sensors if not using fake data
    if (!current_config.use_fake_data) {
        ESP_LOGI(TAG, "Initializing real sensors...");
        
        // Initialize I2C bus (shared by AHT20 and AS3935)
        err = i2c_bus_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(err));
            return err;
        }
        
        // Scan I2C bus to see what's connected
        i2c_bus_scan();
        
        // Initialize AHT20 temperature/humidity sensor
        err = aht20_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize AHT20: %s (sensor may be missing)", esp_err_to_name(err));
            sensor_error_flags |= ERR_TEMP_SENSOR | ERR_HUMIDITY_SENSOR;
            // Continue - we can still try to use the other sensors
        }
        
        // Initialize HX710B pressure sensor
        err = hx710b_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize HX710B: %s (sensor may be missing)", esp_err_to_name(err));
            sensor_error_flags |= ERR_PRESSURE_SENSOR;
            // Continue - we can still try to use the other sensors
        }
        
        // Initialize AS3935 lightning sensor
        as3935_config_t lightning_config = AS3935_CONFIG_DEFAULT();
        lightning_config.indoor = false;  // Outdoor mode for weather station
        err = as3935_init(&lightning_config);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize AS3935: %s (sensor may be missing)", esp_err_to_name(err));
            sensor_error_flags |= ERR_LIGHTNING_SENSOR;
            // Recover I2C bus in case AS3935 failure left it in bad state
            i2c_bus_recover();
            // Re-initialize AHT20 since bus recovery invalidates device handles
            err = aht20_reinit();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to reinit AHT20 after bus recovery: %s", esp_err_to_name(err));
                sensor_error_flags |= ERR_HUMIDITY_SENSOR;
            }
            // Continue - we can still try to use the other sensors
        }
        
        // Initialize lightning data accumulator
        lightning_data_init(&accumulated_lightning);
        
        ESP_LOGI(TAG, "Sensors initialized (error_flags=0x%02X)", sensor_error_flags);
    } else {
        ESP_LOGI(TAG, "Using fake sensor data (no hardware initialization)");
    }
    
    // Initialize LoRa module (optional - may not be connected)
    err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LoRa not available: %s (continuing without wireless)", esp_err_to_name(err));
        // Don't return error - we can still log sensor data locally
    } else {
        // Set up callbacks only if LoRa initialized
        lora_set_config_callback(on_config_received);
        lora_set_ack_callback(on_ack_received);
        lora_set_ping_callback(on_ping_received);
        
        // Enable adaptive power if configured
        lora_set_adaptive_power(current_config.adaptive_power);
        
        // Run diagnostics
        lora_run_diagnostics();
    }
    
    ESP_LOGI(TAG, "Sensor routine initialized (interval=%ds, heartbeat=%ds)",
             current_config.update_interval_sec, current_config.heartbeat_interval_sec);
    
    return ESP_OK;
}

esp_err_t sensor_routine_start(void)
{
    if (is_running) {
        ESP_LOGW(TAG, "Sensor routine already running");
        return ESP_OK;
    }
    
    is_running = true;
    
    // Initialize LED color mode from config before starting LED task
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        led_using_fake_data = current_config.use_fake_data;
        xSemaphoreGive(config_mutex);
    }
    
    // Create LED status task
    BaseType_t ret = xTaskCreate(
        led_status_task,
        "led_status_task",
        3072,           // Stack size (needs extra for RMT LED driver)
        NULL,
        4,              // Priority (slightly lower than sensor task)
        &led_status_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create LED status task (continuing without LED status)");
        led_status_task_handle = NULL;
    }
    
    // Create sensor task
    ret = xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,           // Stack size
        NULL,
        5,              // Priority
        &sensor_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        is_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Sensor routine started");
    return ESP_OK;
}

void sensor_routine_stop(void)
{
    if (!is_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping sensor routine...");
    is_running = false;
    
    // Wait for tasks to finish (they will delete themselves)
    vTaskDelay(pdMS_TO_TICKS(200));
    sensor_task_handle = NULL;
    led_status_task_handle = NULL;
}

bool sensor_routine_is_running(void)
{
    return is_running;
}

void sensor_routine_update_config(const sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&current_config, config, sizeof(sensor_config_t));
        lora_set_adaptive_power(current_config.adaptive_power);
        xSemaphoreGive(config_mutex);
        
        ESP_LOGI(TAG, "Config updated: interval=%ds, heartbeat=%ds",
                 current_config.update_interval_sec, current_config.heartbeat_interval_sec);
    }
}

void sensor_routine_get_config(sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &current_config, sizeof(sensor_config_t));
        xSemaphoreGive(config_mutex);
    }
}

uint16_t sensor_routine_get_uptime_min(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t uptime_us = now_us - start_time_us;
    uint32_t uptime_min = (uint32_t)(uptime_us / 60000000LL);
    
    // Clamp to uint16 max
    if (uptime_min > 65535) {
        uptime_min = 65535;
    }
    
    return (uint16_t)uptime_min;
}

void sensor_routine_get_stats(uint32_t *sent, uint32_t *acked, uint32_t *failed)
{
    if (sent) *sent = packets_sent;
    if (acked) *acked = packets_acked;
    if (failed) *failed = packets_failed;
}
