/**
 * @file lora_protocol.h
 * @brief WeatherKit LoRa Protocol Definition
 * 
 * Defines packet structures for communication between:
 * - Base Station (indoor display unit)
 * - Remote Sensor (outdoor weather sensors)
 * 
 * Power Conservation Features:
 * - Adaptive TX power based on link quality
 * - Configurable sensor update interval
 * - Remote sleep/wake commands
 * 
 * Security Features:
 * - Unique sync word per installation
 * - Device ID filtering
 * - CRC16 integrity check
 * - Optional AES-128 encryption
 */

#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Protocol Version
// =============================================================================
#define LORA_PROTOCOL_VERSION   1

// =============================================================================
// Network Configuration
// =============================================================================

// Sync word - CHANGE THIS to a unique value for your installation (0x00-0xFF)
// Both base and remote must use the same value
#define LORA_SYNC_WORD          0x77

// Device IDs
#define DEVICE_ID_BROADCAST     0xFF    // Broadcast to all devices
#define DEVICE_ID_BASE          0x01    // Base station (display unit)
#define DEVICE_ID_REMOTE_1      0x02    // Remote sensor #1
#define DEVICE_ID_REMOTE_2      0x03    // Remote sensor #2 (future)
#define DEVICE_ID_REMOTE_3      0x04    // Remote sensor #3 (future)

// =============================================================================
// Message Types
// =============================================================================
#define MSG_TYPE_WEATHER_DATA       0x01    // Remote -> Base: Sensor readings + config echo
#define MSG_TYPE_WEATHER_ACK        0x02    // Base -> Remote: Weather received ACK (clears lightning)
#define MSG_TYPE_CONFIG             0x03    // Base -> Remote: Configuration update
#define MSG_TYPE_CONFIG_ACK         0x04    // Remote -> Base: Config received acknowledgment
#define MSG_TYPE_WEATHER_ACK_ACK    0x05    // Remote -> Base: Confirms lightning cleared (three-way handshake)

// Legacy types (deprecated - kept for compatibility during transition)
#define MSG_TYPE_SENSOR_STATUS  0x10    // DEPRECATED: Use weather packets as heartbeat
#define MSG_TYPE_ACK            0x11    // DEPRECATED: Use MSG_TYPE_WEATHER_ACK
#define MSG_TYPE_PING           0x12    // DEPRECATED: Use config with locate flag
#define MSG_TYPE_PONG           0x13    // DEPRECATED: Removed

// =============================================================================
// TX Power Levels (dBm)
// =============================================================================
#define TX_POWER_MIN            2       // Minimum power (safe, short range)
#define TX_POWER_LOW            2       // Low power mode limit (2 dBm)
#define TX_POWER_MED            7       // Medium power
#define TX_POWER_HIGH           12      // High power  
#define TX_POWER_MAX            17      // Maximum power (PA_BOOST, requires antenna!)

// High power mode allows TX above 2dBm
#define TX_POWER_LOW_MODE_MAX   2       // Max power when HIGH_POWER disabled
#define TX_POWER_HIGH_MODE_MAX  17      // Max power when HIGH_POWER enabled

// =============================================================================
// Adaptive Power Thresholds
// =============================================================================
// RSSI thresholds for automatic power adjustment
#define RSSI_EXCELLENT          -50     // Very strong signal, can reduce power
#define RSSI_GOOD               -70     // Good signal
#define RSSI_FAIR               -85     // Fair signal
#define RSSI_POOR               -100    // Poor signal, increase power
#define RSSI_CRITICAL           -110    // Critical, use max power

// =============================================================================
// Timing Configuration (defaults, can be changed via config message)
// =============================================================================
#define DEFAULT_UPDATE_INTERVAL_SEC     30      // Send weather data every 30s
#define MIN_UPDATE_INTERVAL_SEC         5       // Minimum 5 seconds
#define MAX_UPDATE_INTERVAL_SEC         3600    // Maximum 1 hour

#define ACK_TIMEOUT_MS                  1000    // Wait 1s for ACK
#define MAX_RETRIES                     3       // Retry count before giving up
#define CONFIG_RETRY_INTERVAL_MS        2000    // Retry config every 2s until ACK

// =============================================================================
// Packet Structures
// =============================================================================

/**
 * @brief Common packet header (6 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t version;            // Protocol version
    uint8_t src_id;             // Source device ID
    uint8_t dest_id;            // Destination device ID (0xFF = broadcast)
    uint8_t msg_type;           // Message type
    uint8_t sequence;           // Sequence number (0-255, wraps)
    uint8_t flags;              // Flags (encrypted, ack_required, etc.)
} packet_header_t;

// Header flags
#define FLAG_ENCRYPTED          0x01    // Payload is AES encrypted
#define FLAG_ACK_REQUIRED       0x02    // Sender expects ACK
#define FLAG_IS_RETRY           0x04    // This is a retransmission
#define FLAG_LOW_BATTERY        0x08    // Sender has low battery

#define PACKET_HEADER_SIZE      sizeof(packet_header_t)

// =============================================================================
// Lightning Data
// =============================================================================

// Maximum lightning strikes we can report in one packet
// Limited by LoRa packet size (each distance is 1 byte = 0-63 km range)
#define MAX_LIGHTNING_STRIKES   8

/**
 * @brief Lightning strike data
 * Records strikes since last ACKNOWLEDGED weather report
 * Data is only cleared when weather ACK is received from base
 */
typedef struct __attribute__((packed)) {
    uint32_t total_count;                       // Total strike count since power-on
    uint8_t strikes_since_ack;                  // Number of strikes since last ACK (0-255)
    uint8_t distances[MAX_LIGHTNING_STRIKES];   // Distance of each recent strike in km (0-63)
                                                // Only first 'strikes_since_ack' entries valid (up to MAX)
                                                // 0xFF = out of range / unknown
} lightning_data_t;

// =============================================================================
// Sensor Configuration (echoed in weather packets)
// =============================================================================

/**
 * @brief Sensor configuration state
 * Included in weather packets so base knows actual sensor state
 */
typedef struct __attribute__((packed)) {
    uint16_t update_interval;   // Current weather update interval in seconds
    uint8_t tx_power;           // Current TX power level (dBm)
    uint8_t flags;              // Configuration flags (CFG_*)
} sensor_config_echo_t;

// =============================================================================
// Weather Payload
// =============================================================================

/**
 * @brief Weather data payload (Remote -> Base)
 * Sent periodically with sensor readings
 * This is the primary packet - acts as heartbeat
 * 
 * Sensors: BME280/AHT20 (temp/humidity/pressure) + AS3935 (lightning)
 */
typedef struct __attribute__((packed)) {
    // BME280/AHT20 data
    int16_t temperature;        // Temperature in 0.01°C (e.g., 2250 = 22.50°C)
    uint16_t humidity;          // Humidity in 0.01% (e.g., 4500 = 45.00%)
    uint16_t pressure;          // Pressure in 0.1 hPa (e.g., 10132 = 1013.2 hPa)
    
    // AS3935 lightning data (only cleared on ACK receipt)
    lightning_data_t lightning; // Lightning strikes since last ACK
    
    // Current sensor configuration (so base knows actual state)
    sensor_config_echo_t config;// Current config state
    
    // Link quality (for adaptive power)
    int8_t last_base_rssi;      // RSSI of last received packet from base
    uint8_t sensor_tx_power;    // Current sensor TX power level
    
    // Status
    uint8_t error_flags;        // Sensor error flags (ERR_*)
    uint8_t sequence;           // Packet sequence for ACK matching
    uint32_t uptime_sec;        // Sensor uptime in seconds
} weather_payload_t;

/**
 * @brief Sensor status payload (DEPRECATED - use weather packets)
 * Kept for backward compatibility
 */
typedef struct __attribute__((packed)) {
    uint16_t battery_mv;        // Battery voltage in mV
    uint8_t battery_percent;    // Battery percentage (0-100)
    int8_t rssi;                // RSSI of last received packet
    int8_t snr;                 // SNR of last received packet (in 0.25 dB units)
    uint8_t tx_power;           // Current TX power level
    uint16_t uptime_min;        // Uptime in minutes (wraps at 65535)
    uint16_t update_interval;   // Current update interval in seconds
    uint8_t error_flags;        // Sensor error flags
} status_payload_t;

// Error flags for status_payload_t
#define ERR_TEMP_SENSOR         0x01    // Temperature sensor error
#define ERR_HUMIDITY_SENSOR     0x02    // Humidity sensor error
#define ERR_PRESSURE_SENSOR     0x04    // Pressure sensor error
#define ERR_LIGHTNING_SENSOR    0x08    // Lightning sensor error
#define ERR_LOW_BATTERY         0x10    // Battery critically low
#define ERR_RTC_ERROR           0x20    // RTC error

/**
 * @brief Configuration payload (Base -> Remote)
 * Sent to change remote sensor settings
 * Retransmitted until CONFIG_ACK received
 */
typedef struct __attribute__((packed)) {
    uint16_t update_interval;   // Weather update interval in seconds
    uint8_t tx_power;           // TX power level to use (only if adaptive off)
    uint8_t flags;              // Configuration flags
    uint8_t config_sequence;    // Sequence number for ACK matching
} config_payload_t;

// Configuration flags
#define CFG_ADAPTIVE_POWER      0x01    // Enable adaptive TX power
#define CFG_HIGH_POWER          0x02    // Allow TX power above 2dBm
#define CFG_LOCATE_BUZZER       0x04    // Enable locate buzzer on sensor

// Legacy flags (deprecated)
#define CFG_SLEEP_BETWEEN       0x10    // DEPRECATED: Sleep between transmissions
#define CFG_ACK_REQUIRED        0x20    // DEPRECATED: Always require ACK for weather

/**
 * @brief Weather acknowledgment payload (Base -> Remote)
 * Sent in response to weather packets
 * Receipt clears lightning data on sensor
 */
typedef struct __attribute__((packed)) {
    uint8_t acked_sequence;     // Sequence number being acknowledged
    int8_t base_rssi;           // RSSI of weather packet at base (for sensor power adjust)
    int8_t base_snr;            // SNR of weather packet at base
    uint8_t base_tx_power;      // Base's current TX power
    uint8_t suggested_power;    // Suggested TX power for sensor
} weather_ack_payload_t;

/**
 * @brief Weather ACK-ACK payload (Remote -> Base)
 * Confirms sensor received ACK and cleared lightning data
 * Three-way handshake: Weather -> ACK -> ACK-ACK
 */
typedef struct __attribute__((packed)) {
    uint8_t acked_sequence;     // ACK sequence being acknowledged
    uint32_t lightning_total;   // Running total of lightning strikes (confirms sync)
} weather_ack_ack_payload_t;

/**
 * @brief Config acknowledgment payload (Remote -> Base)
 * Sent in response to config packets
 */
typedef struct __attribute__((packed)) {
    uint8_t acked_sequence;     // Config sequence being acknowledged
    int8_t sensor_rssi;         // RSSI of config packet at sensor
    uint8_t sensor_tx_power;    // Sensor's current TX power
    uint8_t applied_flags;      // Flags that were actually applied
} config_ack_payload_t;

/**
 * @brief Legacy ACK payload (DEPRECATED)
 */
typedef struct __attribute__((packed)) {
    uint8_t acked_sequence;     // Sequence number being acknowledged
    int8_t rssi;                // RSSI of received packet (for power adjustment)
    int8_t snr;                 // SNR of received packet
    uint8_t suggested_power;    // Suggested TX power based on link quality
} ack_payload_t;



// =============================================================================
// Complete Packet with CRC
// =============================================================================

/**
 * @brief Maximum packet size
 */
#define MAX_PACKET_SIZE         64
#define MAX_PAYLOAD_SIZE        (MAX_PACKET_SIZE - PACKET_HEADER_SIZE - 2)  // -2 for CRC

/**
 * @brief Full packet structure (header + payload + CRC)
 * CRC16 is appended at the end of the actual data
 */
typedef struct __attribute__((packed)) {
    packet_header_t header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    // CRC16 follows payload (not in struct, calculated dynamically)
} lora_packet_t;

// =============================================================================
// Helper Functions
// =============================================================================

/**
 * @brief Calculate suggested TX power based on RSSI
 * @param rssi Received signal strength
 * @return Suggested TX power level
 */
static inline uint8_t calculate_adaptive_power(int8_t rssi)
{
    if (rssi > RSSI_EXCELLENT) return TX_POWER_MIN;
    if (rssi > RSSI_GOOD) return TX_POWER_LOW;
    if (rssi > RSSI_FAIR) return TX_POWER_MED;
    if (rssi > RSSI_POOR) return TX_POWER_HIGH;
    return TX_POWER_MAX;
}

// -----------------------------------------------------------------------------
// Temperature encoding (0.01°C resolution)
// -----------------------------------------------------------------------------

/**
 * @brief Convert float temperature to int16 format
 * @param temp_c Temperature in degrees Celsius
 * @return Encoded value (temp * 100)
 */
static inline int16_t encode_temperature(float temp_c)
{
    return (int16_t)(temp_c * 100.0f);
}

/**
 * @brief Convert int16 format to float temperature
 * @param encoded Encoded temperature value
 * @return Temperature in degrees Celsius
 */
static inline float decode_temperature(int16_t encoded)
{
    return (float)encoded / 100.0f;
}

// -----------------------------------------------------------------------------
// Humidity encoding (0.01% resolution)
// -----------------------------------------------------------------------------

/**
 * @brief Convert float humidity to uint16 format
 * @param humidity Relative humidity percentage (0-100)
 * @return Encoded value (humidity * 100)
 */
static inline uint16_t encode_humidity(float humidity)
{
    return (uint16_t)(humidity * 100.0f);
}

/**
 * @brief Convert uint16 format to float humidity
 * @param encoded Encoded humidity value
 * @return Relative humidity percentage
 */
static inline float decode_humidity(uint16_t encoded)
{
    return (float)encoded / 100.0f;
}

// -----------------------------------------------------------------------------
// Pressure encoding (0.1 hPa resolution)
// -----------------------------------------------------------------------------

/**
 * @brief Convert float pressure to uint16 format
 * @param pressure_hpa Pressure in hPa (hectopascals / millibars)
 * @return Encoded value (pressure * 10)
 */
static inline uint16_t encode_pressure(float pressure_hpa)
{
    return (uint16_t)(pressure_hpa * 10.0f);
}

/**
 * @brief Convert uint16 format to float pressure
 * @param encoded Encoded pressure value
 * @return Pressure in hPa
 */
static inline float decode_pressure(uint16_t encoded)
{
    return (float)encoded / 10.0f;
}

// -----------------------------------------------------------------------------
// Lightning data helpers
// -----------------------------------------------------------------------------

/**
 * @brief Initialize lightning data structure
 * @param data Pointer to lightning data
 */
static inline void lightning_data_init(lightning_data_t *data)
{
    data->total_count = 0;
    data->strikes_since_ack = 0;
    for (int i = 0; i < MAX_LIGHTNING_STRIKES; i++) {
        data->distances[i] = 0xFF;  // Invalid/unused
    }
}

/**
 * @brief Clear strikes since ACK (called when ACK received)
 * Does NOT clear total_count
 * @param data Pointer to lightning data
 */
static inline void lightning_data_clear_pending(lightning_data_t *data)
{
    data->strikes_since_ack = 0;
    for (int i = 0; i < MAX_LIGHTNING_STRIKES; i++) {
        data->distances[i] = 0xFF;
    }
}

/**
 * @brief Add a lightning strike to the data
 * Keeps the 8 closest strikes (most dangerous for user safety)
 * @param data Pointer to lightning data
 * @param distance_km Distance in km (0-63, or 0xFF for out of range)
 * @return true if stored in array, false if farther than all stored strikes
 */
static inline bool lightning_data_add(lightning_data_t *data, uint8_t distance_km)
{
    data->total_count++;
    
    // If array not full yet, just add it
    if (data->strikes_since_ack < MAX_LIGHTNING_STRIKES) {
        data->distances[data->strikes_since_ack] = distance_km;
        data->strikes_since_ack++;
        return true;
    }
    
    // Array is full - find the farthest strike and replace if new one is closer
    uint8_t farthest_idx = 0;
    uint8_t farthest_dist = data->distances[0];
    for (int i = 1; i < MAX_LIGHTNING_STRIKES; i++) {
        if (data->distances[i] > farthest_dist) {
            farthest_dist = data->distances[i];
            farthest_idx = i;
        }
    }
    
    // Replace farthest if new strike is closer
    if (distance_km < farthest_dist) {
        data->distances[farthest_idx] = distance_km;
        return true;
    }
    
    return false;  // New strike is farther than all stored, but still counted in total
}

/**
 * @brief Get the closest lightning strike distance
 * @param data Pointer to lightning data
 * @return Closest distance in km, or 0xFF if no strikes
 */
static inline uint8_t lightning_data_closest(const lightning_data_t *data)
{
    if (data->strikes_since_ack == 0) return 0xFF;
    
    uint8_t closest = 0xFF;
    int count = (data->strikes_since_ack < MAX_LIGHTNING_STRIKES) ? 
                data->strikes_since_ack : MAX_LIGHTNING_STRIKES;
    for (int i = 0; i < count; i++) {
        if (data->distances[i] < closest) {
            closest = data->distances[i];
        }
    }
    return closest;
}

/**
 * @brief Calculate adaptive power with high power mode awareness
 * @param rssi Received signal strength
 * @param high_power_enabled Whether high power mode is allowed
 * @return Suggested TX power level (capped if high power disabled)
 */
static inline uint8_t calculate_adaptive_power_limited(int8_t rssi, bool high_power_enabled)
{
    uint8_t power;
    if (rssi > RSSI_EXCELLENT) power = TX_POWER_MIN;
    else if (rssi > RSSI_GOOD) power = TX_POWER_MIN;
    else if (rssi > RSSI_FAIR) power = TX_POWER_MED;
    else if (rssi > RSSI_POOR) power = TX_POWER_HIGH;
    else power = TX_POWER_MAX;
    
    // Cap power if high power mode is disabled
    uint8_t max_power = high_power_enabled ? TX_POWER_HIGH_MODE_MAX : TX_POWER_LOW_MODE_MAX;
    return (power > max_power) ? max_power : power;
}

#endif // LORA_PROTOCOL_H
