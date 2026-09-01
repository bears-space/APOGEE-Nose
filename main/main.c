#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "neo_m9n.h"
#include "status_led.h"
#include "vigilant.h"

#if CONFIG_NEO_M9N_ENABLE
static const char* GPS_TAG = "neo_m9n";

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void* event_handler_arg,
                              esp_event_base_t event_base, int32_t event_id,
                              void* event_data) {
    (void)event_handler_arg;
    (void)event_base;
    switch (event_id) {
        case GPS_UPDATE: {
            gps_t* gps = (gps_t*)event_data;
            ESP_LOGI(
                GPS_TAG,
                "%04d/%02d/%02d %02d:%02d:%02d => "
                "latitude = %.05f longitude = %.05f "
                "altitude = %.02fm speed = %.02fm/s fix = %d sats_in_use = %d",
                gps->date.year + 2000, gps->date.month, gps->date.day,
                gps->tim.hour, gps->tim.minute, gps->tim.second, gps->latitude,
                gps->longitude, gps->altitude, gps->speed, gps->fix,
                gps->sats_in_use);
            break;
        }
        case GPS_UNKNOWN:
            ESP_LOGW(GPS_TAG, "Unknown statement: %s", (char*)event_data);
            break;
        default:
            break;
    }
}
#endif

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

#if CONFIG_NEO_M9N_ENABLE
    nmea_parser_config_t gps_config = NEO_M9N_CONFIG_DEFAULT();
    neo_m9n_handle_t gps_hdl = neo_m9n_init(&gps_config);
    if (gps_hdl) {
        neo_m9n_add_handler(gps_hdl, gps_event_handler, NULL);
    }
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
