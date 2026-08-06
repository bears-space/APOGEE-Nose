#pragma once
#ifdef __cplusplus
extern "C" {
#endif
 
#include <stddef.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

typedef struct message_t {
    uint8_t *data; // max packet size for LLCC68 is 255 bytes, and 254 with address filtering, but we don't use address filtering
    size_t length;
} message_t;

/*
* Initializes the narrowband communication module, and starts the rxtx task which continuously transmits sensor data and listens for commands.
* @param commandQueue pointer to FreeRTOS queue for data received by narrowband module
* @param sensorDataQueue pointer to FreeRTOS queue for data to be transmitted by narrowband module
*/
void init_narrowband(QueueHandle_t commandQueue, QueueHandle_t sensorDataQueue);

#ifdef __cplusplus
}
#endif