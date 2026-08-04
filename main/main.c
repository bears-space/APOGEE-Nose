#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "sdkconfig.h"
#include "status_led.h"
#include "vigilant.h"

static const char* TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

#if CONFIG_GPS_ENABLE
    esp_err_t gps_error = gps_init();
    if (gps_error != ESP_OK) {
        ESP_LOGE(TAG, "GPS initialization failed: %s",
                 esp_err_to_name(gps_error));
    }
#endif
}
