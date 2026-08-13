#include <unistd.h>
#include "esp_log.h"
#include "vigilant.h"
#include "status_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "e220m30s.h"
#include <string.h>
#include <stdlib.h>

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
    const char *txt = "Hello from the rocket! ";
    size_t txt_len = strlen(txt); // excludes terminating NUL
    for (int i = 0; i < 5; ++i) {
        uint8_t *buf = (uint8_t *)malloc(txt_len);
        if (buf == NULL) {
            // allocation failed; skip enqueue
            continue;
        }
        memcpy(buf, txt, txt_len);
        message_t msg = { .data = buf, .length = txt_len };
        xQueueSend(sensorDataQueue, &msg, portMAX_DELAY);
    }
    init_narrowband(commandQueue, sensorDataQueue);

    while(true) {
        // main loop can be used for other tasks, e.g. reading sensors and pushing data to the sensorDataQueue for transmission
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
