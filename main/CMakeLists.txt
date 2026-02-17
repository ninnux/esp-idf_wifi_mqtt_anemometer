
idf_component_register(
    SRCS
        "app_main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        mqtt
        driver
        esp_http_client
        app_update
        freertos
        log
        ninux_esp32_ota
        ninux_sensordata_pb
        qmc5883l
)

