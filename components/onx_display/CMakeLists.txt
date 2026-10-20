idf_component_register(SRCS "onx_display.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES board_config esp_driver_gpio esp_driver_i2c esp_driver_spi esp_lcd esp_timer freertos log)
