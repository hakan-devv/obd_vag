idf_component_register(SRCS "can_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi
                       PRIV_REQUIRES mcp2515 board_config log)
