idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine esp_timer esp_driver_gpio e220m30s
)
