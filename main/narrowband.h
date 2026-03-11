#pragma once
#ifdef __cplusplus
extern "C" {
#endif
 
#include <stddef.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

void init_narrowband(QueueHandle_t* commandQueue, QueueHandle_t* sensorDataQueue);

#ifdef __cplusplus
}
#endif