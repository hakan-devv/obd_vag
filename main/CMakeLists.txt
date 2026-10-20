idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES can_driver obd_client onx_display log)
