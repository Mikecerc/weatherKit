idf_component_register(SRCS "main.c"
                            "sensor_routine.c"
                            "drivers/lora.c"
                            "drivers/led.c"
                    INCLUDE_DIRS "." "drivers"
                    REQUIRES driver esp_timer nvs_flash esp_driver_rmt)