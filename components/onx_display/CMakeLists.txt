idf_component_register(SRCS "onx_display.c" "swipe_gesture.c"
                       INCLUDE_DIRS "include"
                       REQUIRES parameter_manager
                       PRIV_REQUIRES board_config esp_driver_gpio esp_driver_i2c esp_driver_spi esp_lcd esp_timer freertos heap log)
