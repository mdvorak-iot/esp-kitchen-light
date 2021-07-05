cmake_minimum_required(VERSION 3.15.0)

idf_component_register(
        SRCS
        app_main.c
        app_status.c
        INCLUDE_DIRS .
        REQUIRES
        freertos
        nvs_flash
        driver
        wifi_provisioning
        double_reset
        status_led
        wifi_reconnect
        app_wifi
        button
)
