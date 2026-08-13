#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stddef.h>
#include <stdint.h>

typedef struct message_t {
    uint8_t* data;  // owned by the receiver once sent/queued; free with free()
    size_t length;  // max 254 bytes (LLCC68 packet payload is 255 bytes, one
                    // byte is the length prefix); longer messages are truncated
} message_t;

/*
 * Initializes the narrowband communication module and starts the rxtx task,
 * which continuously transmits messages and listens for incoming packets.
 * The node role (rocket or ground) is selected in menuconfig (NB_RADIO_MODE):
 * - Rocket: sensorDataQueue holds data to transmit; received commands are
 *   delivered to commandQueue.
 * - Ground: commandQueue holds commands to transmit; received sensor data is
 *   delivered to sensorDataQueue.
 * @param commandQueue pointer to FreeRTOS queue for narrowband data
 * @param sensorDataQueue pointer to FreeRTOS queue for narrowband data
 */
void init_narrowband(QueueHandle_t commandQueue, QueueHandle_t sensorDataQueue);

#ifdef __cplusplus
}
#endif