/**
 * @file lora.h
 * @brief LoRa driver for SX1278/RA-02 module (Sensor Package Version)
 * 
 * Based on Inteform/esp32-lora-library
 * Modified for WeatherKit Sensor Package - no UI dependencies
 */

#ifndef LORA_H
#define LORA_H

#include "esp_err.h"
#include "lora_protocol.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Low-Level Packet Format (used by lora_send_secure/lora_receive_secure)
// =============================================================================

/**
 * @brief Simple packet header for secure messaging
 * This is a lighter-weight header than packet_header_t for low-level use
 */
typedef struct __attribute__((packed)) {
    uint8_t device_id;          // Source device ID
    uint8_t dest_id;            // Destination device ID (0xFF = broadcast)
    uint8_t msg_type;           // Message type
    uint8_t sequence;           // Sequence number
    uint16_t checksum;          // CRC16 checksum
} lora_header_t;

#define LORA_HEADER_SIZE        sizeof(lora_header_t)
#define LORA_MAX_PAYLOAD        (255 - LORA_HEADER_SIZE)

// Device IDs (re-exported from protocol for convenience)
#define LORA_DEVICE_ID_BASE     DEVICE_ID_BASE
#define LORA_DEVICE_ID_REMOTE   DEVICE_ID_REMOTE_1
#define LORA_DEVICE_ID_BROADCAST DEVICE_ID_BROADCAST

// =============================================================================
// LoRa Status
// =============================================================================

/**
 * @brief LoRa module status
 */
typedef struct {
    bool initialized;           // Module initialized successfully
    bool high_power;            // Currently using high power mode
    uint8_t tx_power;           // Current TX power level (dBm)
    int8_t last_rssi;           // RSSI of last received packet (dBm)
    float last_snr;             // SNR of last received packet (dB)
    uint32_t packets_sent;      // Total packets transmitted
    uint32_t packets_received;  // Total packets received  
    uint32_t packets_rejected;  // Packets rejected (wrong ID/checksum)
    uint32_t crc_errors;        // Packets with CRC errors
    uint8_t last_src_id;        // Source ID of last valid packet
    uint8_t suggested_power;    // Suggested power based on link quality
} lora_status_t;

/**
 * @brief Initialize the LoRa module
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lora_init(void);

/**
 * @brief Check if LoRa module is initialized
 * @return true if initialized
 */
bool lora_is_initialized(void);

/**
 * @brief Set the radio into idle (standby) mode
 */
void lora_idle(void);

/**
 * @brief Set the radio into sleep mode (low power)
 */
void lora_sleep(void);

/**
 * @brief Set the radio into continuous receive mode
 */
void lora_receive(void);

/**
 * @brief Set carrier frequency
 * @param frequency Frequency in Hz (e.g., 915000000 for 915 MHz)
 */
void lora_set_frequency(long frequency);

/**
 * @brief Set TX power level
 * @param level Power level 2-17 dBm (or 20 with PA_HP)
 */
void lora_set_tx_power(int level);

/**
 * @brief Set spreading factor
 * @param sf Spreading factor 6-12 (higher = longer range, slower)
 */
void lora_set_spreading_factor(int sf);

/**
 * @brief Set bandwidth
 * @param sbw Bandwidth in Hz (7800 to 500000)
 */
void lora_set_bandwidth(long sbw);

/**
 * @brief Set coding rate
 * @param denominator Coding rate denominator 5-8 (4/5, 4/6, 4/7, 4/8)
 */
void lora_set_coding_rate(int denominator);

/**
 * @brief Enable CRC checking
 */
void lora_enable_crc(void);

/**
 * @brief Disable CRC checking
 */
void lora_disable_crc(void);

/**
 * @brief Set sync word (allows multiple networks)
 * @param sw Sync word (0x12 = private, 0x34 = public LoRaWAN)
 */
void lora_set_sync_word(int sw);

/**
 * @brief Send a packet (blocking)
 * @param buf Data buffer to send
 * @param size Number of bytes to send (max 255)
 * @return ESP_OK on success
 */
esp_err_t lora_send_packet(const uint8_t *buf, int size);

/**
 * @brief Check if a packet has been received
 * @return true if packet available
 */
bool lora_received(void);

/**
 * @brief Wait for a packet to be received (blocking with interrupt)
 * @param timeout_ms Timeout in milliseconds (0 = wait forever)
 * @return true if packet received, false if timeout
 * 
 * This function uses DIO0 interrupt to efficiently wait for packets
 * without busy-polling. The CPU can sleep while waiting.
 */
bool lora_wait_receive(uint32_t timeout_ms);

/**
 * @brief Receive a packet
 * @param buf Buffer to store received data
 * @param size Maximum buffer size
 * @return Number of bytes received (0 if none)
 */
int lora_receive_packet(uint8_t *buf, int size);

/**
 * @brief Get RSSI of last received packet
 * @return RSSI in dBm
 */
int lora_packet_rssi(void);

/**
 * @brief Get SNR of last received packet
 * @return SNR in dB
 */
float lora_packet_snr(void);

/**
 * @brief Get current LoRa status
 * @param status Pointer to status structure to fill
 */
void lora_get_status(lora_status_t *status);

/**
 * @brief Shutdown the LoRa module
 */
void lora_close(void);

/**
 * @brief Dump all registers (for debugging)
 */
void lora_dump_registers(void);

/**
 * @brief Run diagnostic tests to verify LoRa hardware
 * Tests: chip version, frequency registers, RSSI/noise floor
 * Results are printed to console via ESP_LOG
 */
void lora_run_diagnostics(void);

// =============================================================================
// Low-Level Messaging (raw packets)
// =============================================================================

/**
 * @brief Send a raw packet with header and checksum
 * @param dest_id Destination device ID (or 0xFF for broadcast)
 * @param msg_type Message type
 * @param payload Data payload
 * @param payload_len Length of payload
 * @return ESP_OK on success
 */
esp_err_t lora_send_secure(uint8_t dest_id, uint8_t msg_type, 
                           const uint8_t *payload, int payload_len);

/**
 * @brief Receive and validate a packet
 * @param src_id Output: source device ID
 * @param msg_type Output: message type
 * @param payload Buffer for payload data
 * @param max_len Maximum payload buffer size
 * @return Payload length on success, 0 if no packet or validation failed
 */
int lora_receive_secure(uint8_t *src_id, uint8_t *msg_type,
                        uint8_t *payload, int max_len);

/**
 * @brief Calculate CRC16 checksum
 */
uint16_t lora_crc16(const uint8_t *data, int len);

// =============================================================================
// High-Level Protocol API (WeatherKit specific - Sensor Package)
// =============================================================================

/**
 * @brief Send weather data to base station (for remote sensor)
 * @param weather Pointer to weather payload
 * @return ESP_OK on success
 */
esp_err_t lora_send_weather(const weather_payload_t *weather);

/**
 * @brief Send config acknowledgment to base station
 * @param config_seq Config sequence being acknowledged
 * @param applied_flags Flags that were actually applied
 * @return ESP_OK on success
 */
esp_err_t lora_send_config_ack(uint8_t config_seq, uint8_t applied_flags);

/**
 * @brief Send weather ACK-ACK to confirm lightning data was cleared
 * @param acked_seq Sequence number of the weather packet being confirmed
 * @param lightning_total Total lightning count after clearing (for base to sync)
 * @return ESP_OK on success
 */
esp_err_t lora_send_weather_ack_ack(uint8_t acked_seq, uint32_t lightning_total);

/**
 * @brief Send status/heartbeat to base station (for remote sensor)
 * @param status Pointer to status payload
 * @return ESP_OK on success
 * @deprecated Use weather packets as heartbeat instead
 */
esp_err_t lora_send_sensor_status(const status_payload_t *status);

/**
 * @brief Process received packet and dispatch to handler
 * @return true if a packet was processed
 */
bool lora_process_rx(void);

/**
 * @brief Set callback for received config (remote sensor)
 */
typedef void (*lora_config_cb_t)(const config_payload_t *config);
void lora_set_config_callback(lora_config_cb_t callback);

/**
 * @brief Set callback for received weather ACK (remote sensor)
 * ACK contains RSSI for adaptive power, sequence for lightning clearing
 */
typedef void (*lora_weather_ack_cb_t)(const weather_ack_payload_t *ack);
void lora_set_weather_ack_callback(lora_weather_ack_cb_t callback);

/**
 * @brief Enable/disable adaptive TX power
 * @param enable true to automatically adjust power based on link quality
 */
void lora_set_adaptive_power(bool enable);

/**
 * @brief Enable/disable high power mode
 * @param enable true to allow TX power above 2dBm (up to 17dBm)
 */
void lora_set_high_power(bool enable);

/**
 * @brief Get suggested TX power based on recent link quality
 * @return Suggested power level in dBm
 */
uint8_t lora_get_suggested_power(void);

/**
 * @brief Get the current TX power setting
 * @return Current TX power in dBm
 */
uint8_t lora_get_current_power(void);

/**
 * @brief Get the last received RSSI
 * @return RSSI in dBm
 */
int8_t lora_get_last_rssi(void);

#endif // LORA_H

