idf_component_register(
    SRCS 
        "main.c"
        "sensor_routine.c"
        "drivers/lora.c"
        "drivers/led.c"
        "drivers/i2c_init.c"      
        "drivers/lightning_driver.c"
        "drivers/pressure_driver.c"
        "drivers/temp_humidity_driver.c"
    INCLUDE_DIRS 
        "."
        "drivers"
    REQUIRES driver esp_timer nvs_flash
)
