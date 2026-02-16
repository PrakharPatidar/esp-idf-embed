idf_component_register(SRCS "bldc_esc_constant.c" "ble_service.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_ledc bt nvs_flash)
