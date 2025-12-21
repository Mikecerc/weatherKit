# Details

Date : 2025-12-19 19:05:34

Directory /home/mike/code/weatherKit

Total : 76 files,  8082 codes, 3110 comments, 2350 blanks, all 13542 lines

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [CMakeLists.txt](/CMakeLists.txt) | CMake | 7 | 0 | 1 | 8 |
| [dev/.devcontainer/Dockerfile](/dev/.devcontainer/Dockerfile) | Docker | 8 | 0 | 5 | 13 |
| [dev/.devcontainer/devcontainer.json](/dev/.devcontainer/devcontainer.json) | JSON with Comments | 21 | 0 | 0 | 21 |
| [dev/CMakeLists.txt](/dev/CMakeLists.txt) | CMake | 7 | 0 | 1 | 8 |
| [dev/README.md](/dev/README.md) | Markdown | 22 | 0 | 11 | 33 |
| [dev/main/CMakeLists.txt](/dev/main/CMakeLists.txt) | CMake | 24 | 0 | 0 | 24 |
| [dev/main/idf\_component.yml](/dev/main/idf_component.yml) | YAML | 2 | 0 | 1 | 3 |
| [dev/main/include/drivers/buttons.h](/dev/main/include/drivers/buttons.h) | C++ | 22 | 19 | 8 | 49 |
| [dev/main/include/drivers/display.h](/dev/main/include/drivers/display.h) | C++ | 15 | 29 | 12 | 56 |
| [dev/main/include/drivers/lora.h](/dev/main/include/drivers/lora.h) | C++ | 82 | 220 | 56 | 358 |
| [dev/main/include/drivers/lora\_protocol.h](/dev/main/include/drivers/lora_protocol.h) | C++ | 224 | 204 | 67 | 495 |
| [dev/main/include/pinout.h](/dev/main/include/pinout.h) | C++ | 24 | 27 | 10 | 61 |
| [dev/main/include/tasks/config\_tx\_task.h](/dev/main/include/tasks/config_tx_task.h) | C++ | 13 | 30 | 11 | 54 |
| [dev/main/include/tasks/lora\_rx\_task.h](/dev/main/include/tasks/lora_rx_task.h) | C++ | 6 | 11 | 6 | 23 |
| [dev/main/include/tasks/task\_common.h](/dev/main/include/tasks/task_common.h) | C++ | 39 | 63 | 22 | 124 |
| [dev/main/include/tasks/ui\_task.h](/dev/main/include/tasks/ui_task.h) | C++ | 6 | 11 | 6 | 23 |
| [dev/main/include/tasks/weather\_ack\_task.h](/dev/main/include/tasks/weather_ack_task.h) | C++ | 9 | 21 | 8 | 38 |
| [dev/main/include/ui/ui.h](/dev/main/include/ui/ui.h) | C++ | 74 | 98 | 33 | 205 |
| [dev/main/include/ui/ui\_common.h](/dev/main/include/ui/ui_common.h) | C++ | 23 | 42 | 17 | 82 |
| [dev/main/include/ui/ui\_info.h](/dev/main/include/ui/ui_info.h) | C++ | 6 | 8 | 5 | 19 |
| [dev/main/include/ui/ui\_pages.h](/dev/main/include/ui/ui_pages.h) | C++ | 11 | 24 | 10 | 45 |
| [dev/main/include/weather/storm\_tracker.h](/dev/main/include/weather/storm_tracker.h) | C++ | 29 | 36 | 12 | 77 |
| [dev/main/include/weather/weather\_calc.h](/dev/main/include/weather/weather_calc.h) | C++ | 18 | 38 | 11 | 67 |
| [dev/main/src/drivers/buttons.c](/dev/main/src/drivers/buttons.c) | C | 152 | 31 | 29 | 212 |
| [dev/main/src/drivers/buttons\_new.c](/dev/main/src/drivers/buttons_new.c) | C | 193 | 56 | 41 | 290 |
| [dev/main/src/drivers/display.c](/dev/main/src/drivers/display.c) | C | 155 | 34 | 49 | 238 |
| [dev/main/src/drivers/lora.c](/dev/main/src/drivers/lora.c) | C | 871 | 171 | 219 | 1,261 |
| [dev/main/src/main.c](/dev/main/src/main.c) | C | 114 | 40 | 24 | 178 |
| [dev/main/src/tasks/config\_tx\_task.c](/dev/main/src/tasks/config_tx_task.c) | C | 127 | 24 | 33 | 184 |
| [dev/main/src/tasks/lora\_rx\_task.c](/dev/main/src/tasks/lora_rx_task.c) | C | 158 | 51 | 46 | 255 |
| [dev/main/src/tasks/task\_common.c](/dev/main/src/tasks/task_common.c) | C | 81 | 23 | 23 | 127 |
| [dev/main/src/tasks/ui\_task.c](/dev/main/src/tasks/ui_task.c) | C | 32 | 13 | 9 | 54 |
| [dev/main/src/tasks/weather\_ack\_task.c](/dev/main/src/tasks/weather_ack_task.c) | C | 102 | 28 | 27 | 157 |
| [dev/main/src/ui/ui.c](/dev/main/src/ui/ui.c) | C | 538 | 81 | 79 | 698 |
| [dev/main/src/ui/ui\_info.c](/dev/main/src/ui/ui_info.c) | C | 172 | 27 | 55 | 254 |
| [dev/main/src/ui/ui\_pages.c](/dev/main/src/ui/ui_pages.c) | C | 328 | 71 | 65 | 464 |
| [dev/main/src/weather/storm\_tracker.c](/dev/main/src/weather/storm_tracker.c) | C | 188 | 23 | 51 | 262 |
| [dev/main/src/weather/weather\_calc.c](/dev/main/src/weather/weather_calc.c) | C | 61 | 10 | 16 | 87 |
| [docs/SOFTWARE\_ARCHITECTURE.md](/docs/SOFTWARE_ARCHITECTURE.md) | Markdown | 576 | 0 | 207 | 783 |
| [sensorPackage/.devcontainer/Dockerfile](/sensorPackage/.devcontainer/Dockerfile) | Docker | 8 | 0 | 5 | 13 |
| [sensorPackage/.devcontainer/devcontainer.json](/sensorPackage/.devcontainer/devcontainer.json) | JSON with Comments | 21 | 0 | 0 | 21 |
| [sensorPackage/CMakeLists.txt](/sensorPackage/CMakeLists.txt) | CMake | 7 | 0 | 1 | 8 |
| [sensorPackage/README.md](/sensorPackage/README.md) | Markdown | 22 | 0 | 11 | 33 |
| [sensorPackage/esp\_idf\_project\_configuration.json](/sensorPackage/esp_idf_project_configuration.json) | JSON | 26 | 0 | 1 | 27 |
| [sensorPackage/idf\_component.yml](/sensorPackage/idf_component.yml) | YAML | 3 | 0 | 2 | 5 |
| [sensorPackage/main/CMakeLists.txt](/sensorPackage/main/CMakeLists.txt) | CMake | 21 | 0 | 1 | 22 |
| [sensorPackage/main/idf\_component.yml](/sensorPackage/main/idf_component.yml) | YAML | 2 | 0 | 2 | 4 |
| [sensorPackage/main/include/drivers/aht20.h](/sensorPackage/main/include/drivers/aht20.h) | C++ | 10 | 7 | 4 | 21 |
| [sensorPackage/main/include/drivers/as3935.h](/sensorPackage/main/include/drivers/as3935.h) | C++ | 42 | 85 | 19 | 146 |
| [sensorPackage/main/include/drivers/buzzer.h](/sensorPackage/main/include/drivers/buzzer.h) | C++ | 10 | 27 | 9 | 46 |
| [sensorPackage/main/include/drivers/i2c\_init.h](/sensorPackage/main/include/drivers/i2c_init.h) | C++ | 10 | 26 | 9 | 45 |
| [sensorPackage/main/include/drivers/led.h](/sensorPackage/main/include/drivers/led.h) | C++ | 9 | 19 | 7 | 35 |
| [sensorPackage/main/include/drivers/lora.h](/sensorPackage/main/include/drivers/lora.h) | C++ | 74 | 194 | 51 | 319 |
| [sensorPackage/main/include/drivers/lora\_protocol.h](/sensorPackage/main/include/drivers/lora_protocol.h) | C++ | 224 | 205 | 67 | 496 |
| [sensorPackage/main/include/drivers/pressure\_driver.h](/sensorPackage/main/include/drivers/pressure_driver.h) | C++ | 16 | 46 | 10 | 72 |
| [sensorPackage/main/include/pinout.h](/sensorPackage/main/include/pinout.h) | C++ | 27 | 52 | 17 | 96 |
| [sensorPackage/main/include/sensor\_routine.h](/sensorPackage/main/include/sensor_routine.h) | C++ | 15 | 57 | 13 | 85 |
| [sensorPackage/main/include/tasks/led\_status\_task.h](/sensorPackage/main/include/tasks/led_status_task.h) | C++ | 8 | 12 | 6 | 26 |
| [sensorPackage/main/include/tasks/lora\_rx\_task.h](/sensorPackage/main/include/tasks/lora_rx_task.h) | C++ | 8 | 14 | 6 | 28 |
| [sensorPackage/main/include/tasks/task\_common.h](/sensorPackage/main/include/tasks/task_common.h) | C++ | 47 | 90 | 32 | 169 |
| [sensorPackage/main/include/tasks/weather\_tx\_task.h](/sensorPackage/main/include/tasks/weather_tx_task.h) | C++ | 8 | 14 | 6 | 28 |
| [sensorPackage/main/main.c](/sensorPackage/main/main.c) | C | 50 | 25 | 13 | 88 |
| [sensorPackage/main/src/drivers/aht20.c](/sensorPackage/main/src/drivers/aht20.c) | C | 87 | 14 | 30 | 131 |
| [sensorPackage/main/src/drivers/as3935.c](/sensorPackage/main/src/drivers/as3935.c) | C | 281 | 41 | 74 | 396 |
| [sensorPackage/main/src/drivers/buzzer.c](/sensorPackage/main/src/drivers/buzzer.c) | C | 131 | 16 | 26 | 173 |
| [sensorPackage/main/src/drivers/i2c\_init.c](/sensorPackage/main/src/drivers/i2c_init.c) | C | 69 | 7 | 20 | 96 |
| [sensorPackage/main/src/drivers/led.c](/sensorPackage/main/src/drivers/led.c) | C | 171 | 16 | 35 | 222 |
| [sensorPackage/main/src/drivers/lora.c](/sensorPackage/main/src/drivers/lora.c) | C | 745 | 169 | 206 | 1,120 |
| [sensorPackage/main/src/drivers/pressure\_driver.c](/sensorPackage/main/src/drivers/pressure_driver.c) | C | 129 | 46 | 45 | 220 |
| [sensorPackage/main/src/main.c](/sensorPackage/main/src/main.c) | C | 44 | 22 | 12 | 78 |
| [sensorPackage/main/src/sensor\_routine.c](/sensorPackage/main/src/sensor_routine.c) | C | 470 | 172 | 130 | 772 |
| [sensorPackage/main/src/tasks/led\_status\_task.c](/sensorPackage/main/src/tasks/led_status_task.c) | C | 87 | 22 | 20 | 129 |
| [sensorPackage/main/src/tasks/lora\_rx\_task.c](/sensorPackage/main/src/tasks/lora_rx_task.c) | C | 144 | 45 | 39 | 228 |
| [sensorPackage/main/src/tasks/sensor\_routine.c](/sensorPackage/main/src/tasks/sensor_routine.c) | C | 167 | 24 | 43 | 234 |
| [sensorPackage/main/src/tasks/task\_common.c](/sensorPackage/main/src/tasks/task_common.c) | C | 161 | 35 | 44 | 240 |
| [sensorPackage/main/src/tasks/weather\_tx\_task.c](/sensorPackage/main/src/tasks/weather_tx_task.c) | C | 188 | 44 | 48 | 280 |

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)