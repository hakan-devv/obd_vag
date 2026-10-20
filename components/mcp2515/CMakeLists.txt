idf_component_register(SRCS "mcp2515.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi
                       PRIV_REQUIRES log)
