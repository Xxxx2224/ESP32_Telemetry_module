idf_component_register(SRCS "src/main3.c" 
                            "src/wifi_manager.c" 
                            "src/uart_manager.c" 
                            "src/mavlink_handler.c" 
                            "src/data_manager.c" 
                            "src/tcp_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES mavlink cjson esp_wifi esp_driver_uart nvs_flash esp_netif)

