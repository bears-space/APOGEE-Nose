#include <unistd.h>
#include "esp_log.h"
#include "vigilant.h"
#include "status_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "message.h"
#include "narrowband.h"

// static const char *TAG = "app_main";

void app_main(void)
{
    VigilantConfig VgConfig = {
        .unique_component_name = "Vigilant ESP Test",
        .network_mode = NW_MODE_APSTA
    };
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    // init narrowband communication
    QueueHandle_t commandQueue = xQueueCreate(10, sizeof(message_t));   // for now we initialize the queues to 10 elements, perhaps subject to change
    QueueHandle_t sensorDataQueue = xQueueCreate(10, sizeof(message_t));
    init_narrowband(commandQueue, sensorDataQueue);

    while(true) {
        // main loop can be used for other tasks, e.g. reading sensors and pushing data to the sensorDataQueue for transmission
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
