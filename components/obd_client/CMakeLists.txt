if(CONFIG_OBD_RUN_PROTOCOL_TESTS)
    idf_component_register(SRCS "obd_client.c" "protocol_tests.c"
                           INCLUDE_DIRS "include"
                           PRIV_REQUIRES can_driver isotp parameter_manager uds_client
                                        onx_display esp_timer log)
else()
    idf_component_register(SRCS "obd_client.c"
                           INCLUDE_DIRS "include"
                           PRIV_REQUIRES can_driver isotp parameter_manager uds_client
                                        onx_display esp_timer log)
endif()
