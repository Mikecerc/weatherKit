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
#define MSG_TYPE_WEATHER_DATA   0x01    // Remote -> Base: Sensor readings
#define MSG_TYPE_SENSOR_STATUS  0x02    // Remote -> Base: Battery, RSSI, etc.
#define MSG_TYPE_ACK            0x03    // Base -> Remote: Acknowledgment
#define MSG_TYPE_CONFIG         0x04    // Base -> Remote: Configuration update
#define MSG_TYPE_PING           0x05    // Base -> Remote: Locate/wake signal
#define MSG_TYPE_PONG           0x06    // Remote -> Base: Response to ping

// =============================================================================
// TX Power Levels (dBm)
// =============================================================================
#define TX_POWER_MIN            2       // Minimum power (safe, short range)
#define TX_POWER_LOW            7       // Low power
#define TX_POWER_MED            12      // Medium power
#define TX_POWER_HIGH           17      // High power (requires antenna!)
#define TX_POWER_MAX            20      // Maximum power (PA_HP, requires antenna!)

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

#define DEFAULT_HEARTBEAT_INTERVAL_SEC  300     // Status heartbeat every 5 min
#define ACK_TIMEOUT_MS                  1000    // Wait 1s for ACK
#define MAX_RETRIES                     3       // Retry count before giving up

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
#define MAX_LIGHTNING_STRIKES   16

/**
 * @brief Lightning strike data
 * Records strikes since last weather report
 */
typedef struct __attribute__((packed)) {
    uint8_t count;                              // Number of strikes (0-255)
    uint8_t distances[MAX_LIGHTNING_STRIKES];   // Distance of each strike in km (0-63)
                                                // Only first 'count' entries are valid
                                                // 0xFF = out of range / unknown
} lightning_data_t;

// =============================================================================
// Weather Payload
// =============================================================================

/**
 * @brief Weather data payload (Remote -> Base)
 * Sent periodically with sensor readings
 * 
 * Sensors: BME280 (temp/humidity/pressure) + AS3935 (lightning)
 */
typedef struct __attribute__((packed)) {
    // BME280 data
    int16_t temperature;        // Temperature in 0.01°C (e.g., 2250 = 22.50°C)
    uint16_t humidity;          // Humidity in 0.01% (e.g., 4500 = 45.00%)
    uint16_t pressure;          // Pressure in 0.1 hPa (e.g., 10132 = 1013.2 hPa)
    
    // AS3935 lightning data
    lightning_data_t lightning; // Lightning strikes since last report
    
    // Link quality (for adaptive power)
    int8_t rssi;                // RSSI of last received packet from base
    uint8_t tx_power;           // Current TX power level
} weather_payload_t;

/**
 * @brief Sensor status payload (Remote -> Base)
 * Sent periodically as heartbeat
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
 */
typedef struct __attribute__((packed)) {
    uint16_t update_interval;   // Weather update interval in seconds
    uint16_t heartbeat_interval;// Status heartbeat interval in seconds
    uint8_t tx_power;           // TX power level to use
    uint8_t flags;              // Configuration flags
} config_payload_t;

// Configuration flags
#define CFG_ADAPTIVE_POWER      0x01    // Enable adaptive TX power
#define CFG_SLEEP_BETWEEN       0x02    // Sleep between transmissions
#define CFG_ACK_REQUIRED        0x04    // Require ACK for weather data

/**
 * @brief Acknowledgment payload (Base -> Remote)
 */
typedef struct __attribute__((packed)) {
    uint8_t acked_sequence;     // Sequence number being acknowledged
    int8_t rssi;                // RSSI of received packet (for power adjustment)
    int8_t snr;                 // SNR of received packet
    uint8_t suggested_power;    // Suggested TX power based on link quality
} ack_payload_t;

/**
 * @brief Ping payload (Base -> Remote)
 * Used to locate sensor or wake from deep sleep
 */
typedef struct __attribute__((packed)) {
    uint8_t ping_type;          // 0=status request, 1=locate (flash LED)
    uint8_t reserved[3];        // Future use
} ping_payload_t;

/**
 * @brief Pong payload (Remote -> Base)
 * Response to ping
 */
typedef struct __attribute__((packed)) {
    uint8_t ping_type;          // Echo of ping type
    int8_t rssi;                // RSSI of ping
    uint8_t battery_percent;    // Quick battery status
    uint8_t tx_power;           // Current TX power
} pong_payload_t;

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
    data->count = 0;
    for (int i = 0; i < MAX_LIGHTNING_STRIKES; i++) {
        data->distances[i] = 0xFF;  // Invalid/unused
    }
}

/**
 * @brief Add a lightning strike to the data
 * @param data Pointer to lightning data
 * @param distance_km Distance in km (0-63, or 0xFF for out of range)
 * @return true if added, false if array full
 */
static inline bool lightning_data_add(lightning_data_t *data, uint8_t distance_km)
{
    if (data->count >= MAX_LIGHTNING_STRIKES) {
        return false;  // Array full
    }
    data->distances[data->count] = distance_km;
    data->count++;
    return true;
}

/**
 * @brief Get the closest lightning strike distance
 * @param data Pointer to lightning data
 * @return Closest distance in km, or 0xFF if no strikes
 */
static inline uint8_t lightning_data_closest(const lightning_data_t *data)
{
    if (data->count == 0) return 0xFF;
    
    uint8_t closest = 0xFF;
    for (int i = 0; i < data->count && i < MAX_LIGHTNING_STRIKES; i++) {
        if (data->distances[i] < closest) {
            closest = data->distances[i];
        }
    }
    return closest;
}

#endif // LORA_PROTOCOL_H
