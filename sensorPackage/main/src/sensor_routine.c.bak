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
#include "drivers/buzzer.h"
#include "drivers/i2c_init.h"
#include "drivers/aht20.h"
#include "drivers/pressure_driver.h"
#include "drivers/as3935.h"
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
// Lightning data is only cleared when weather ACK is received from base
static lightning_data_t accumulated_lightning = {0};

// Track last sent weather sequence for ACK matching
static uint8_t last_weather_sequence = 0;
static bool awaiting_weather_ack = false;

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
    ESP_LOGI(TAG, "Config received: interval=%ds, power=%d, flags=0x%02X",
             config->update_interval, config->tx_power, config->flags);
    
    // Signal LED for received packet
    led_signal_packet_received();
    
    uint8_t applied_flags = 0;
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update interval if valid
        if (config->update_interval >= MIN_UPDATE_INTERVAL_SEC && 
            config->update_interval <= MAX_UPDATE_INTERVAL_SEC) {
            current_config.update_interval_sec = config->update_interval;
        }
        
        // Update adaptive power setting
        if (config->flags & CFG_ADAPTIVE_POWER) {
            current_config.adaptive_power = true;
            applied_flags |= CFG_ADAPTIVE_POWER;
        } else {
            current_config.adaptive_power = false;
        }
        lora_set_adaptive_power(current_config.adaptive_power);
        
        // Update high power mode
        if (config->flags & CFG_HIGH_POWER) {
            current_config.high_power = true;
            applied_flags |= CFG_HIGH_POWER;
        } else {
            current_config.high_power = false;
        }
        
        // Handle locate buzzer
        if (config->flags & CFG_LOCATE_BUZZER) {
            buzzer_set_locate(true);
            applied_flags |= CFG_LOCATE_BUZZER;
            ESP_LOGI(TAG, "Locate mode ENABLED");
        } else {
            // Disable buzzer if locate was previously enabled
            buzzer_set_locate(false);
        }
        
        // If adaptive power is off, use the specified power level
        if (!current_config.adaptive_power && config->tx_power >= TX_POWER_MIN) {
            lora_set_tx_power(config->tx_power);
        }
        
        xSemaphoreGive(config_mutex);
    }
    
    // Send CONFIG_ACK to confirm receipt and applied settings
    lora_send_config_ack(config->config_sequence, applied_flags);
}

/**
 * @brief Called when weather ACK received from base
 * This clears the pending lightning data and sends ACK-ACK
 */
static void on_weather_ack_received(const weather_ack_payload_t *ack)
{
    ESP_LOGI(TAG, "Weather ACK received: seq=%d, base_rssi=%d, suggested_power=%d",
             ack->acked_sequence, ack->base_rssi, ack->suggested_power);
    
    // Signal LED for received packet
    led_signal_packet_received();
    
    // Only clear lightning if this ACK matches our last sent weather packet
    if (ack->acked_sequence == last_weather_sequence) {
        // Clear pending lightning strikes (keeps total_count)
        lightning_data_clear_pending(&accumulated_lightning);
        ESP_LOGI(TAG, "Lightning data cleared (ACK seq matched)");
        awaiting_weather_ack = false;
        
        // Send ACK-ACK to confirm we cleared the lightning data
        // This tells base it's safe to record the lightning without double-counting
        lora_send_weather_ack_ack(ack->acked_sequence, accumulated_lightning.total_count);
    } else {
        ESP_LOGW(TAG, "ACK sequence mismatch: got %d, expected %d (ignoring duplicate)",
                 ack->acked_sequence, last_weather_sequence);
        // Don't send ACK-ACK for mismatched sequence - already processed
    }
    
    packets_acked++;
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
    
    // No lightning for fake data (but preserve existing accumulated data)
    memcpy(&weather->lightning, &accumulated_lightning, sizeof(lightning_data_t));
    
    // Include current sensor config in weather packet
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        weather->config.update_interval = current_config.update_interval_sec;
        weather->config.tx_power = lora_get_current_power();
        weather->config.flags = 0;
        if (current_config.adaptive_power) weather->config.flags |= CFG_ADAPTIVE_POWER;
        if (current_config.high_power) weather->config.flags |= CFG_HIGH_POWER;
        xSemaphoreGive(config_mutex);
    }
    
    // Include link quality info
    weather->last_base_rssi = lora_get_last_rssi();
    weather->sensor_tx_power = lora_get_current_power();
    weather->error_flags = 0;  // No errors for fake data
    
    // Include uptime
    weather->uptime_sec = (uint32_t)((esp_timer_get_time() - start_time_us) / 1000000);
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
      //  ESP_LOGI(TAG, "AHT20: T=%.2f°C, RH=%.2f%%", temperature, humidity);
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
       // ESP_LOGI(TAG, "HX710B: P=%.2f hPa", pressure);
    } else {
        ESP_LOGE(TAG, "HX710B read failed: %s - sending 0 value", esp_err_to_name(err));
        sensor_error_flags |= ERR_PRESSURE_SENSOR;
        // Leave pressure at 0 - don't use fake data
    }
    
    // Encode values into packet (will be 0 if sensor failed)
    weather->temperature = encode_temperature(temperature);
    weather->humidity = encode_humidity(humidity);
    weather->pressure = encode_pressure(pressure);
    
    // Copy accumulated lightning data (DO NOT clear - only cleared on ACK receipt)
    memcpy(&weather->lightning, &accumulated_lightning, sizeof(lightning_data_t));
    // Note: lightning_data_clear_pending() is called in on_weather_ack_received()
    
    // Include current sensor config in weather packet
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        weather->config.update_interval = current_config.update_interval_sec;
        weather->config.tx_power = lora_get_current_power();
        weather->config.flags = 0;
        if (current_config.adaptive_power) weather->config.flags |= CFG_ADAPTIVE_POWER;
        if (current_config.high_power) weather->config.flags |= CFG_HIGH_POWER;
        xSemaphoreGive(config_mutex);
    }
    
    // Include link quality info
    weather->last_base_rssi = lora_get_last_rssi();
    weather->sensor_tx_power = lora_get_current_power();
    weather->error_flags = sensor_error_flags;
    
    // Include uptime
    weather->uptime_sec = (uint32_t)((esp_timer_get_time() - start_time_us) / 1000000);
    
    ESP_LOGI(TAG, "Weather: T=%.2f°C, RH=%.2f%%, P=%.2f hPa, Lightning=%d strikes since ACK",
             decode_temperature(weather->temperature),
             decode_humidity(weather->humidity),
             decode_pressure(weather->pressure),
             weather->lightning.strikes_since_ack);
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
    ESP_LOGI(TAG, "Lightning:    %d strikes (total: %lu)", 
             weather->lightning.strikes_since_ack, (unsigned long)weather->lightning.total_count);
    if (weather->lightning.strikes_since_ack > 0) {
        ESP_LOGI(TAG, "  Closest:    %d km", lightning_data_closest(&weather->lightning));
    }
    ESP_LOGI(TAG, "Sensor RSSI:  %d dBm", weather->last_base_rssi);
    ESP_LOGI(TAG, "TX Power:     %d dBm", weather->sensor_tx_power);
    ESP_LOGI(TAG, "Errors:       0x%02X", weather->error_flags);
    ESP_LOGI(TAG, "========================================");
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
        
        // Get current interval
        uint16_t weather_interval_sec;
        
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            weather_interval_sec = current_config.update_interval_sec;
            xSemaphoreGive(config_mutex);
        } else {
            weather_interval_sec = DEFAULT_UPDATE_INTERVAL_SEC;
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
            
            // Set sequence number for ACK matching
            last_weather_sequence++;
            weather.sequence = last_weather_sequence;
            
            // Log sensor data to monitor (for debugging without LoRa)
            log_weather_data(&weather);
            
            // Attempt to send via LoRa (only if initialized)
            if (lora_is_initialized()) {
                esp_err_t err = lora_send_weather(&weather);
                if (err == ESP_OK) {
                    packets_sent++;
                    awaiting_weather_ack = true;  // Wait for ACK to clear lightning
                    led_signal_packet_sent();  // Flash bright green/blue based on data mode

                    ESP_LOGI(TAG, "Weather packet sent seq=%d (total=%lu)", 
                             last_weather_sequence, (unsigned long)packets_sent);
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
        
        // Weather packets serve as heartbeat - no separate status packet needed
        
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

        // Initialize buzzer (optional hardware)
        err = buzzer_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize buzzer: %s (continuing without buzzer)", esp_err_to_name(err));
            // Don't fail - buzzer optional
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
    i2c_bus_log_scan();
        
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
        /*
        
        */
       
        
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
        lora_set_weather_ack_callback(on_weather_ack_received);
        
        // Enable adaptive power if configured
        lora_set_adaptive_power(current_config.adaptive_power);
        
        // Run diagnostics
        lora_run_diagnostics();
    }

    // Initialize lightning data
    lightning_data_init(&accumulated_lightning);
     
    ESP_LOGI(TAG, "Sensor routine initialized (interval=%ds, high_power=%d)",
             current_config.update_interval_sec, current_config.high_power);
    

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
        
        ESP_LOGI(TAG, "Config updated: interval=%ds, high_power=%d",
                 current_config.update_interval_sec, current_config.high_power);
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
