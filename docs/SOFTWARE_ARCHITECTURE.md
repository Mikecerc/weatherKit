# WeatherKit Software Architecture

This document provides in-depth software architecture flowcharts for both the Base Station (dev) and Sensor Package (sensorPackage) microcontrollers.

---

## System Overview

```mermaid
graph TB
    subgraph "Sensor Package (ESP32-S3)"
        SP_MAIN[app_main]
        SP_LED[LED Status Task]
        SP_TX[Weather TX Task]
        SP_RX[LoRa RX Task]
        SP_SENSORS[Sensors<br/>AHT20, HX710B, AS3935]
        SP_LORA[LoRa Radio<br/>SX1278]
    end
    
    subgraph "Base Station (ESP32-S3)"
        BS_MAIN[app_main]
        BS_RX[LoRa RX Task]
        BS_ACK[Weather ACK Task]
        BS_CFG[Config TX Task]
        BS_UI[UI Task]
        BS_BTN[Button Task]
        BS_DISPLAY[OLED Display]
        BS_LORA[LoRa Radio<br/>SX1278]
    end
    
    SP_LORA <-->|433 MHz LoRa| BS_LORA
    
    SP_SENSORS --> SP_TX
    SP_TX --> SP_LORA
    SP_LORA --> SP_RX
    SP_RX --> SP_TX
    
    BS_LORA --> BS_RX
    BS_RX --> BS_ACK
    BS_RX --> BS_UI
    BS_ACK --> BS_LORA
    BS_CFG --> BS_LORA
    BS_BTN --> BS_UI
    BS_UI --> BS_DISPLAY
```

---

## LoRa Communication Protocol (Three-Way Handshake)

```mermaid
sequenceDiagram
    participant S as Sensor
    participant B as Base Station
    
    Note over S: Read Sensors
    S->>B: WEATHER (temp, humidity, pressure, lightning)
    Note over B: Store pending lightning data
    B->>S: WEATHER_ACK (rssi, suggested_power)
    Note over S: Clear lightning counts
    S->>B: WEATHER_ACK_ACK (confirmed)
    Note over B: Record lightning to storm tracker
```

---

# Base Station (dev)

## Main Initialization Flow

```mermaid
flowchart TD
    START([app_main]) --> NVS[Initialize NVS Flash]
    NVS --> TASK_COMMON[task_common_init<br/>Create semaphores & queues]
    TASK_COMMON --> STORM[storm_tracker_init]
    STORM --> DISPLAY[display_init<br/>I2C OLED setup]
    DISPLAY --> UI_INIT[ui_init<br/>Load default state]
    UI_INIT --> SETTINGS[ui_load_settings<br/>Load from NVS]
    SETTINGS --> BRIGHTNESS[display_set_brightness]
    BRIGHTNESS --> BUTTONS[buttons_init<br/>GPIO config + task]
    BUTTONS --> LORA{lora_init}
    
    LORA -->|Success| LORA_DIAG[lora_run_diagnostics]
    LORA -->|Fail| LORA_ERR[Log Error]
    
    LORA_DIAG --> WEATHER_INIT[Initialize weather_data_t]
    LORA_ERR --> WEATHER_INIT
    
    WEATHER_INIT --> START_TASKS{LoRa OK?}
    START_TASKS -->|Yes| RX_TASK[lora_rx_task_start]
    RX_TASK --> ACK_TASK[weather_ack_task_start]
    ACK_TASK --> CFG_TASK[config_tx_task_start]
    START_TASKS -->|No| UI_TASK
    CFG_TASK --> UI_TASK[ui_task_start]
    
    UI_TASK --> MAIN_LOOP[Main Loop<br/>Status logging every 30s]
    MAIN_LOOP --> MAIN_LOOP
```

---

## Button Task (buttons.c)

```mermaid
flowchart TD
    START([button_task]) --> LOOP_START{g_tasks_running?}
    LOOP_START -->|No| EXIT([Task Exit])
    LOOP_START -->|Yes| READ[Read GPIO states<br/>left_pressed, right_pressed]
    
    READ --> ANY_CHECK{Any button pressed?}
    ANY_CHECK -->|Yes| ANY_SENT{any_press_sent?}
    ANY_SENT -->|No| SEND_ANY[callback BUTTON_ANY_PRESS<br/>any_press_sent = true]
    ANY_SENT -->|Yes| BOTH_CHECK
    SEND_ANY --> BOTH_CHECK
    ANY_CHECK -->|No| RESET_ANY[any_press_sent = false]
    RESET_ANY --> INDIVIDUAL
    
    BOTH_CHECK{Both pressed?}
    BOTH_CHECK -->|Yes| BOTH_START{both_pressed_start == 0?}
    BOTH_START -->|Yes| INIT_BOTH[both_pressed_start = now<br/>Suppress individual events]
    BOTH_START -->|No| BOTH_LONG{Held >= 2000ms?}
    BOTH_LONG -->|Yes| FIRE_BOTH[callback BUTTON_BOTH_LONG]
    BOTH_LONG -->|No| DELAY
    INIT_BOTH --> DELAY
    FIRE_BOTH --> DELAY
    
    BOTH_CHECK -->|No| RESET_BOTH[Reset both tracking]
    RESET_BOTH --> INDIVIDUAL
    
    INDIVIDUAL[Process individual buttons] --> PROCESS_BTN
    
    subgraph PROCESS_BTN[process_button for each]
        BTN_READ[Read debounced state] --> STATE_CHANGE{State changed?}
        STATE_CHANGE -->|No| CHK_LONG{Pressed & not long_fired?}
        STATE_CHANGE -->|Yes| PRESSED{Now pressed?}
        PRESSED -->|Yes| START_TIMER[Record press_start_tick]
        PRESSED -->|No| WAS_LONG{Was long press fired?}
        WAS_LONG -->|Yes| SKIP
        WAS_LONG -->|No| SHORT_PRESS[callback SHORT press event]
        
        CHK_LONG -->|Yes| LONG_TIME{Held >= 2000ms?}
        CHK_LONG -->|No| SKIP[Skip]
        LONG_TIME -->|Yes| LONG_PRESS[callback LONG press event<br/>long_press_fired = true]
        LONG_TIME -->|No| SKIP
    end
    
    PROCESS_BTN --> DELAY[vTaskDelay 20ms]
    DELAY --> LOOP_START
```

---

## LoRa RX Task (lora_rx_task.c)

```mermaid
flowchart TD
    START([lora_rx_task]) --> INIT[Register callbacks<br/>on_weather_received<br/>on_status_received<br/>on_ack_ack_received]
    
    INIT --> LOOP{Loop}
    LOOP --> MUTEX[Take lora_mutex]
    
    MUTEX -->|Got it| RECEIVE[lora_receive_packet]
    MUTEX -->|Timeout| TIMEOUT_CHK
    
    RECEIVE --> GIVE_MUTEX[Give lora_mutex]
    GIVE_MUTEX --> PARSE{Packet received?}
    
    PARSE -->|Yes| DISPATCH[Dispatch to callback based on type]
    PARSE -->|No| TIMEOUT_CHK
    
    subgraph DISPATCH_TYPES[Packet Types]
        WEATHER[WEATHER packet] --> WEATHER_CB[on_weather_received]
        STATUS[STATUS packet] --> STATUS_CB[on_status_received]
        ACK_ACK[ACK_ACK packet] --> ACK_CB[on_ack_ack_received]
    end
    
    DISPATCH --> DISPATCH_TYPES
    DISPATCH_TYPES --> TIMEOUT_CHK
    
    TIMEOUT_CHK[Check sensor timeout] --> SENSOR_TO{Elapsed > 120s?}
    SENSOR_TO -->|Yes| MARK_DISC[Mark sensor disconnected]
    SENSOR_TO -->|No| DELAY
    MARK_DISC --> DELAY
    
    DELAY[vTaskDelay 20ms] --> LOOP
```

### on_weather_received Callback

```mermaid
flowchart TD
    START([on_weather_received]) --> FIRST{First packet?}
    FIRST -->|Yes| SYNC[config_tx_request_sync]
    FIRST -->|No| CONVERT
    SYNC --> CONVERT
    
    CONVERT[Convert protocol format<br/>to weather_data_t] --> MUTEX[Take weather_state_mutex]
    
    MUTEX --> LIGHTNING{New lightning strikes?}
    LIGHTNING -->|Yes| CACHE[Cache pending lightning<br/>pending_total, pending_count, pending_closest]
    LIGHTNING -->|No| SKIP[Skip lightning update]
    CACHE --> GIVE[Give mutex]
    SKIP --> GIVE
    
    GIVE --> STORM[storm_tracker_record_weather]
    STORM --> UPDATE_TIME[task_update_sensor_time]
    UPDATE_TIME --> QUEUE[xQueueOverwrite weather_queue]
    QUEUE --> NOTIFY[weather_ack_queue_ack<br/>Request ACK be sent]
    NOTIFY --> LOG[Log weather data]
    LOG --> END([Return])
```

---

## Weather ACK Task (weather_ack_task.c)

```mermaid
flowchart TD
    START([weather_ack_task]) --> LOOP{Loop}
    
    LOOP --> CHECK_Q[xQueueReceive ack_request_queue<br/>100ms timeout]
    
    CHECK_Q -->|Got request| SEND_ACK[send_weather_ack<br/>Take lora_mutex<br/>lora_send_weather_ack]
    CHECK_Q -->|Timeout| CHECK_RETRY
    
    SEND_ACK --> UPDATE_STATE[Update pending_ack_state<br/>waiting = true<br/>sent_time = now]
    UPDATE_STATE --> CHECK_RETRY
    
    CHECK_RETRY{pending_ack_state.waiting?}
    CHECK_RETRY -->|No| DELAY
    CHECK_RETRY -->|Yes| ELAPSED{Elapsed > ACK_TIMEOUT?}
    
    ELAPSED -->|No| DELAY
    ELAPSED -->|Yes| RETRIES{retries < MAX_RETRIES?}
    
    RETRIES -->|Yes| RETRY[Increment retries<br/>Resend ACK]
    RETRIES -->|No| GIVE_UP[Log failure<br/>Update lightning anyway<br/>Clear pending state]
    
    RETRY --> DELAY
    GIVE_UP --> DELAY
    
    DELAY[vTaskDelay 50ms] --> LOOP
```

---

## Config TX Task (config_tx_task.c)

```mermaid
flowchart TD
    START([config_tx_task]) --> LOOP{Loop}
    
    LOOP --> CHECK_CFG{ui_check_config_pending?}
    CHECK_CFG -->|Yes| BUILD_CFG[Build config_payload_t<br/>interval, power, flags]
    CHECK_CFG -->|No| CHECK_LOC
    
    BUILD_CFG --> SEND_CFG[Take lora_mutex<br/>lora_send_config]
    SEND_CFG --> CHECK_LOC
    
    CHECK_LOC{ui_check_locate_pending?}
    CHECK_LOC -->|Yes| SEND_PING[Take lora_mutex<br/>lora_send_ping]
    CHECK_LOC -->|No| CHECK_QUEUE
    SEND_PING --> CHECK_QUEUE
    
    CHECK_QUEUE[xQueueReceive config_request_queue] --> QUEUE_RESULT{Got request?}
    QUEUE_RESULT -->|Yes| PROCESS_Q[Process queued config/locate]
    QUEUE_RESULT -->|No| RETRY_CHK
    PROCESS_Q --> RETRY_CHK
    
    RETRY_CHK[lora_config_retry_check<br/>Resend if no CONFIG_ACK] --> DELAY
    
    DELAY[vTaskDelay 100ms] --> LOOP
```

---

## UI Task (ui_task.c)

```mermaid
flowchart TD
    START([ui_task]) --> INIT[last_refresh = 0]
    
    INIT --> LOOP{Loop}
    
    LOOP --> CHK_REFRESH[ui_check_refresh<br/>Handle button-triggered refresh]
    
    CHK_REFRESH --> RECV_Q[xQueueReceive weather_queue<br/>50ms timeout]
    
    RECV_Q -->|Got data| UPDATE[ui_update_weather<br/>Copy to cached_weather<br/>last_refresh = now]
    RECV_Q -->|Timeout| PERIODIC
    
    UPDATE --> DELAY
    
    PERIODIC{now - last_refresh >= 500ms?}
    PERIODIC -->|Yes| REFRESH[ui_refresh<br/>last_refresh = now]
    PERIODIC -->|No| DELAY
    
    REFRESH --> DELAY
    
    DELAY[vTaskDelay 20ms] --> LOOP
```

---

## UI Navigation State Machine

```mermaid
stateDiagram-v2
    [*] --> MainPages
    
    MainPages --> MainPages: RIGHT_SHORT (cycle)
    MainPages --> InfoView: RIGHT_LONG (show info)
    MainPages --> SettingsEdit: LEFT_SHORT on Settings page
    MainPages --> SensorStatusScroll: LEFT_SHORT on Sensor page
    
    InfoView --> InfoView: RIGHT_SHORT (cycle info pages)
    InfoView --> MainPages: LEFT_SHORT (exit)
    
    SettingsEdit --> SettingsEdit: RIGHT_SHORT (next item)
    SettingsEdit --> SettingsEdit: LEFT_SHORT (activate/toggle)
    SettingsEdit --> LoRaConfirm: LEFT_SHORT on HighPwr (if enabling)
    SettingsEdit --> AboutView: LEFT_SHORT on About
    SettingsEdit --> MainPages: LEFT_LONG (exit, save)
    
    LoRaConfirm --> SettingsEdit: RIGHT_SHORT (cancel)
    LoRaConfirm --> SettingsEdit: LEFT_SHORT (confirm high power)
    
    AboutView --> SettingsEdit: LEFT_LONG (back)
    
    SensorStatusScroll --> SensorStatusScroll: LEFT_SHORT (toggle scroll)
    
    note right of MainPages
        Pages: Main, Lightning Map,
        Calculated, Storm Tracker,
        Sensor Status, Settings
    end note
```

---

# Sensor Package (sensorPackage)

## Main Initialization Flow

```mermaid
flowchart TD
    START([app_main]) --> NVS[Initialize NVS Flash]
    NVS --> SENSOR_INIT[sensor_routine_init NULL]
    
    subgraph sensor_routine_init
        TASK_COMMON[task_common_init<br/>Create mutexes<br/>Load config from NVS]
        TASK_COMMON --> LED_INIT[led_init]
        LED_INIT --> BUZZER_INIT[buzzer_init]
        BUZZER_INIT --> FAKE_CHK{use_fake_data?}
        
        FAKE_CHK -->|No| I2C[i2c_bus_init]
        I2C --> I2C_SCAN[i2c_bus_log_scan]
        I2C_SCAN --> AHT20[aht20_init]
        AHT20 --> HX710B[hx710b_init]
        HX710B --> LORA_INIT
        
        FAKE_CHK -->|Yes| LORA_INIT[lora_init]
        
        LORA_INIT --> ADAPTIVE[lora_set_adaptive_power]
        ADAPTIVE --> DIAG[lora_run_diagnostics]
    end
    
    SENSOR_INIT --> START_ROUTINE[sensor_routine_start]
    
    subgraph sensor_routine_start
        LED_TASK[led_status_task_start]
        LED_TASK --> RX_TASK[lora_rx_task_start]
        RX_TASK --> TX_TASK[weather_tx_task_start]
    end
    
    START_ROUTINE --> MAIN_LOOP[Main Loop<br/>Print stats every 60s]
    MAIN_LOOP --> MAIN_LOOP
```

---

## Task Common Init (with NVS Load)

```mermaid
flowchart TD
    START([task_common_init]) --> MUTEX1[Create g_config_mutex]
    MUTEX1 --> MUTEX2[Create g_lora_mutex]
    MUTEX2 --> MUTEX3[Create g_lightning_mutex]
    
    MUTEX3 --> DEFAULT[Set g_current_config to defaults<br/>interval=30s, adaptive=true, high_power=false]
    
    DEFAULT --> NVS_LOAD[task_config_load]
    
    subgraph task_config_load
        NVS_OPEN[nvs_open sensor_cfg] --> OPEN_OK{Success?}
        OPEN_OK -->|No| USE_DEFAULT[Use default config]
        OPEN_OK -->|Yes| READ_INT[nvs_get_u16 interval]
        READ_INT --> READ_FLAGS[nvs_get_u8 flags]
        READ_FLAGS --> APPLY[Apply to g_current_config<br/>interval, adaptive, high_power]
        APPLY --> NVS_CLOSE[nvs_close]
    end
    
    NVS_LOAD --> LIGHTNING[lightning_data_init]
    LIGHTNING --> TIME[g_start_time_us = now]
    TIME --> END([Return ESP_OK])
```

---

## LED Status Task (led_status_task.c)

```mermaid
flowchart TD
    START([led_status_task]) --> LED_OFF[led_off]
    LED_OFF --> LOOP{g_tasks_running?}
    
    LOOP -->|No| EXIT[led_off<br/>vTaskDelete]
    LOOP -->|Yes| CHK_SENT{g_led_flash_packet_sent?}
    
    CHK_SENT -->|Yes| FLASH_SENT[Clear flag<br/>Bright green/blue 100ms]
    CHK_SENT -->|No| CHK_FAIL
    FLASH_SENT --> CHK_FAIL
    
    CHK_FAIL{g_led_flash_packet_failed?}
    CHK_FAIL -->|Yes| FLASH_FAIL[Clear flag<br/>Red 150ms]
    CHK_FAIL -->|No| CHK_RECV
    FLASH_FAIL --> CHK_RECV
    
    CHK_RECV{g_led_flash_packet_received?}
    CHK_RECV -->|Yes| FLASH_RECV[Clear flag<br/>Purple 100ms]
    CHK_RECV -->|No| BASE_LED
    FLASH_RECV --> BASE_LED
    
    BASE_LED{lora_is_initialized?}
    BASE_LED -->|Yes| LORA_ON{g_led_using_fake_data?}
    BASE_LED -->|No| LORA_OFF[Flash red 500ms cycle]
    
    LORA_ON -->|Yes| DIM_BLUE[Solid dim blue]
    LORA_ON -->|No| DIM_GREEN[Solid dim green]
    
    DIM_BLUE --> DELAY
    DIM_GREEN --> DELAY
    LORA_OFF --> DELAY
    
    DELAY[vTaskDelay 50ms] --> LOOP
```

---

## LoRa RX Task - Sensor (lora_rx_task.c)

```mermaid
flowchart TD
    START([lora_rx_task]) --> REG_CB[Register callbacks<br/>on_config_received<br/>on_weather_ack_received]
    
    REG_CB --> LOOP{g_tasks_running?}
    
    LOOP -->|No| EXIT([Task Exit])
    LOOP -->|Yes| MUTEX[Take g_lora_mutex<br/>50ms timeout]
    
    MUTEX -->|Got it| RECEIVE[lora_process_rx<br/>Check for packets]
    MUTEX -->|Timeout| DELAY
    
    RECEIVE --> GIVE[Give g_lora_mutex]
    GIVE --> DELAY[vTaskDelay 10ms]
    DELAY --> LOOP
```

### on_config_received Callback

```mermaid
flowchart TD
    START([on_config_received]) --> LOG[Log config details]
    LOG --> SIGNAL[task_signal_packet_received]
    
    SIGNAL --> MUTEX[Take g_config_mutex]
    MUTEX --> VALID{Interval valid?}
    VALID -->|Yes| SET_INT[g_current_config.update_interval_sec = interval]
    VALID -->|No| SET_FLAGS
    SET_INT --> SET_FLAGS
    
    SET_FLAGS[Set adaptive_power<br/>Set high_power] --> GIVE[Give g_config_mutex]
    
    GIVE --> LOCATE{CFG_LOCATE_BUZZER set?}
    LOCATE -->|Yes| BUZZER_ON[buzzer_set_locate true]
    LOCATE -->|No| BUZZER_OFF[buzzer_set_locate false]
    BUZZER_ON --> APPLY
    BUZZER_OFF --> APPLY
    
    APPLY[lora_set_adaptive_power<br/>lora_set_high_power<br/>lora_set_tx_power] --> SAVE[task_config_save to NVS]
    
    SAVE --> ACK[lora_send_config_ack]
    ACK --> END([Return])
```

### on_weather_ack_received Callback

```mermaid
flowchart TD
    START([on_weather_ack_received]) --> LOG[Log ACK details]
    LOG --> SIGNAL[task_signal_packet_received]
    
    SIGNAL --> SEQ_CHK{acked_sequence == g_last_weather_sequence?}
    
    SEQ_CHK -->|No| DUP[Log sequence mismatch<br/>Ignore duplicate]
    SEQ_CHK -->|Yes| MUTEX[Take g_lightning_mutex]
    
    DUP --> END([Return])
    
    MUTEX --> CLEAR[lightning_data_clear_pending<br/>Save total_count]
    CLEAR --> GIVE[Give g_lightning_mutex]
    
    GIVE --> UPDATE[g_awaiting_weather_ack = false<br/>g_packets_acked++]
    
    UPDATE --> ACK_ACK[lora_send_weather_ack_ack<br/>sequence, total_count]
    
    ACK_ACK --> END
```

---

## Weather TX Task (weather_tx_task.c)

```mermaid
flowchart TD
    START([weather_tx_task]) --> INIT[last_weather_send_us = 0]
    
    INIT --> LOOP{g_tasks_running?}
    
    LOOP -->|No| EXIT[vTaskDelete]
    LOOP -->|Yes| NOW[now_us = esp_timer_get_time]
    
    NOW --> GET_CFG[Take g_config_mutex<br/>Get interval, use_fake]
    
    GET_CFG --> TIME_CHK{now - last_send >= interval?}
    
    TIME_CHK -->|No| DELAY
    TIME_CHK -->|Yes| READ_SENSORS
    
    subgraph READ_SENSORS[Read/Generate Data]
        FAKE_CHK{use_fake_data?}
        FAKE_CHK -->|Yes| GEN_FAKE[generate_fake_weather_data]
        FAKE_CHK -->|No| READ_REAL[read_real_weather_data]
        
        READ_REAL --> AHT20[aht20_read]
        AHT20 --> HX710B[hx710b_read]
        HX710B --> COPY_LIGHT[Copy g_accumulated_lightning]
        
        GEN_FAKE --> COPY_LIGHT
    end
    
    READ_SENSORS --> SEQ[g_last_weather_sequence++<br/>weather.sequence = seq]
    
    SEQ --> LOG_DATA[log_weather_data]
    
    LOG_DATA --> LORA_CHK{lora_is_initialized?}
    
    LORA_CHK -->|No| LOG_ONLY[Log data only mode]
    LORA_CHK -->|Yes| MUTEX[Take g_lora_mutex]
    
    LOG_ONLY --> UPDATE_TIME
    
    MUTEX --> SEND[lora_send_weather]
    
    SEND --> GIVE[Give g_lora_mutex]
    
    GIVE --> RESULT{Send OK?}
    RESULT -->|Yes| SUCCESS[g_packets_sent++<br/>g_awaiting_weather_ack = true<br/>task_signal_packet_sent]
    RESULT -->|No| FAIL[g_packets_failed++<br/>task_signal_packet_failed]
    
    SUCCESS --> SMALL_DELAY[vTaskDelay 5ms<br/>Let RX task catch ACK]
    FAIL --> UPDATE_TIME
    SMALL_DELAY --> UPDATE_TIME
    
    UPDATE_TIME[last_weather_send_us = now] --> DELAY
    
    DELAY[vTaskDelay 100ms] --> LOOP
```

---

## Shared State & Synchronization

```mermaid
graph TB
    subgraph Mutexes
        CONFIG[g_config_mutex<br/>Protects: g_current_config]
        LORA[g_lora_mutex<br/>Protects: LoRa radio access]
        LIGHT[g_lightning_mutex<br/>Protects: g_accumulated_lightning]
    end
    
    subgraph "Shared State (Sensor)"
        S_CFG[g_current_config<br/>interval, adaptive, high_power, use_fake]
        S_LIGHT[g_accumulated_lightning<br/>strikes, distances, total]
        S_FLAGS[g_led_flash_* flags]
        S_STATS[g_packets_sent/acked/failed]
        S_SEQ[g_last_weather_sequence]
    end
    
    subgraph "Shared State (Base)"
        B_LIGHT[lightning_state<br/>pending, confirmed]
        B_ACK[pending_ack_state<br/>sequence, retries, waiting]
        B_SENSOR[last_sensor_data_time]
    end
    
    subgraph "Queues (Base)"
        WEATHER_Q[weather_queue<br/>1 item, weather_data_t]
        ACK_REQ_Q[ack_request_queue<br/>1 item]
        CONFIG_Q[config_tx_queue]
    end
    
    CONFIG --> S_CFG
    LIGHT --> S_LIGHT
    
    WEATHER_Q --> B_LIGHT
    ACK_REQ_Q --> B_ACK
```

---

## NVS Config Persistence (Sensor)

```mermaid
flowchart TD
    subgraph task_config_save
        OPEN[nvs_open sensor_cfg RW] --> WRITE_INT[nvs_set_u16 interval]
        WRITE_INT --> BUILD_FLAGS["flags = adaptive OR high_power << 1"]
        BUILD_FLAGS --> WRITE_FLAGS[nvs_set_u8 flags]
        WRITE_FLAGS --> COMMIT[nvs_commit]
        COMMIT --> CLOSE[nvs_close]
    end
    
    subgraph task_config_load
        OPEN2[nvs_open sensor_cfg RO] --> READ_INT[nvs_get_u16 interval]
        READ_INT --> VALID{5 <= interval <= 300?}
        VALID -->|Yes| APPLY_INT[g_current_config.interval = value]
        VALID -->|No| SKIP_INT[Keep default]
        APPLY_INT --> READ_FLAGS[nvs_get_u8 flags]
        SKIP_INT --> READ_FLAGS
        READ_FLAGS --> APPLY_FLAGS["adaptive = flags AND 1<br/>high_power = flags AND 2"]
        APPLY_FLAGS --> CLOSE2[nvs_close]
    end
```

---

## Task Priorities

| Task | Priority | Stack Size | Purpose |
|------|----------|------------|---------|
| **Base Station** |
| button_task | 5 | 3072 | Button polling and events |
| lora_rx_task | 6 | 4096 | LoRa packet reception |
| weather_ack_task | 5 | 3072 | ACK sending and retry |
| config_tx_task | 4 | 3072 | Config/locate transmission |
| ui_task | 4 | 4096 | Display updates |
| **Sensor Package** |
| lora_rx_task | 5 | 4096 | LoRa packet reception |
| weather_tx_task | 4 | 4096 | Sensor reading and TX |
| led_status_task | 3 | 3072 | LED indicators |

---

## Adaptive TX Power Algorithm (Sensor)

```mermaid
flowchart TD
    START([calculate_adaptive_power]) --> HIGH{high_power_enabled?}
    
    HIGH -->|Yes| MAX_17[max_power = 17 dBm]
    HIGH -->|No| MAX_10[max_power = 10 dBm]
    
    MAX_17 --> RSSI_CHK
    MAX_10 --> RSSI_CHK
    
    RSSI_CHK{base_rssi value?}
    
    RSSI_CHK -->|>= -50 dBm| PWR_2[target = 2 dBm<br/>Excellent signal]
    RSSI_CHK -->|-50 to -60| PWR_5[target = 5 dBm]
    RSSI_CHK -->|-60 to -70| PWR_8[target = 8 dBm]
    RSSI_CHK -->|-70 to -80| PWR_10[target = 10 dBm]
    RSSI_CHK -->|-80 to -90| PWR_12[target = 12 dBm<br/>if high_power]
    RSSI_CHK -->|-90 to -100| PWR_14[target = 14 dBm<br/>if high_power]
    RSSI_CHK -->|< -100 dBm| PWR_MAX[target = max_power]
    
    PWR_2 --> CLAMP
    PWR_5 --> CLAMP
    PWR_8 --> CLAMP
    PWR_10 --> CLAMP
    PWR_12 --> CLAMP
    PWR_14 --> CLAMP
    PWR_MAX --> CLAMP
    
    CLAMP[Clamp to 2..max_power] --> RETURN([Return target power])
```

---

## Error Handling

```mermaid
flowchart TD
    subgraph "Sensor Errors (g_sensor_error_flags)"
        TEMP[ERR_TEMP_SENSOR 0x01]
        HUMID[ERR_HUMIDITY_SENSOR 0x02]
        PRESSURE[ERR_PRESSURE_SENSOR 0x04]
        LIGHTNING[ERR_LIGHTNING_SENSOR 0x08]
        LORA[ERR_LORA 0x10]
    end
    
    subgraph "Recovery Actions"
        RETRY[Retry sensor read]
        CONTINUE[Continue with partial data]
        FALLBACK[Use fake data mode]
    end
    
    TEMP --> CONTINUE
    HUMID --> CONTINUE
    PRESSURE --> CONTINUE
    LORA --> FALLBACK
```

---

## Data Flow Summary

```mermaid
graph LR
    subgraph Sensor
        AHT20[AHT20 Temp/Humid] --> WTX[Weather TX Task]
        HX710B[HX710B Pressure] --> WTX
        AS3935[AS3935 Lightning] --> WTX
        WTX --> LORA_S[LoRa Radio]
    end
    
    LORA_S -->|WEATHER| LORA_B
    
    subgraph Base
        LORA_B[LoRa Radio] --> RX[LoRa RX Task]
        RX --> QUEUE[Weather Queue]
        QUEUE --> UI[UI Task]
        UI --> OLED[OLED Display]
        RX --> ACK_TASK[ACK Task]
        ACK_TASK --> LORA_B
        RX --> STORM[Storm Tracker]
        BTN[Buttons] --> UI
        UI --> CFG_TX[Config TX Task]
        CFG_TX --> LORA_B
    end
    
    LORA_B -->|WEATHER_ACK| LORA_S
    LORA_S -->|ACK_ACK| LORA_B
    LORA_B -->|CONFIG| LORA_S
    LORA_S -->|CONFIG_ACK| LORA_B
```

---

## Memory Map

| Resource | Base Station | Sensor Package |
|----------|--------------|----------------|
| **Total Heap** | ~300KB | ~300KB |
| **Stack (all tasks)** | ~18KB | ~11KB |
| **Weather Queue** | ~128 bytes | N/A |
| **NVS Storage** | UI settings | Sensor config |
| **LoRa TX Buffer** | 256 bytes | 256 bytes |
| **LoRa RX Buffer** | 256 bytes | 256 bytes |
