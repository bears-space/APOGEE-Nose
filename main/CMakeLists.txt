set(main_sources "main.c")

if(CONFIG_GPS_ENABLE)
    list(APPEND main_sources "gps.c" "gps_nmea.c")
endif()

idf_component_register(
    SRCS ${main_sources}
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine esp_driver_uart esp_timer
)
