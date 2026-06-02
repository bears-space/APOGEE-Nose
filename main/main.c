#include <unistd.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "message.h"
#include "narrowband.h"
#include "status_led.h"
#include "vigilant.h"

static const char* TAG = "app_main";

static void set_recovery_as_next_boot(void) {
    const esp_partition_t* factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (factory == NULL) {
        ESP_LOGE(TAG, "factory recovery partition not found");
        return;
    }

    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running != NULL && running->address == factory->address) {
        return;
    }

    esp_err_t err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set recovery as next boot partition: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Next reset will boot recovery partition at 0x%lx",
             (unsigned long)factory->address);
}

void app_main(void) {
    set_recovery_as_next_boot();

    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    // init narrowband communication
    QueueHandle_t commandQueue = xQueueCreate(
        10, sizeof(message_t));  // for now we initialize the queues to 10
                                 // elements, perhaps subject to change
    QueueHandle_t sensorDataQueue = xQueueCreate(10, sizeof(message_t));
    if (commandQueue == NULL || sensorDataQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create narrowband queues");
    } else {
        esp_err_t nb_err = init_narrowband(commandQueue, sensorDataQueue);
        if (nb_err != ESP_OK) {
            ESP_LOGW(TAG, "Narrowband disabled: %s", esp_err_to_name(nb_err));
        }
    }

    while (true) {
        // main loop can be used for other tasks, e.g. reading sensors and
        // pushing data to the sensorDataQueue for transmission
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
