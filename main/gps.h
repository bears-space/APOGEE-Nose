#pragma once

#include "esp_err.h"
#include "gps_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the NEO-M9N UART receiver and NMEA parser task.
 *
 * The UART, baud rate, and GPIO routing are selected in menuconfig. The
 * defaults match this board: GPS TX -> ESP GPIO47 and GPS RX <- ESP GPIO48.
 */
esp_err_t gps_init(void);

/** Stop the parser task and release the UART driver. */
esp_err_t gps_deinit(void);

/** Copy a thread-safe snapshot of the most recently parsed navigation data. */
esp_err_t gps_get_data(gps_data_t* data);

#ifdef __cplusplus
}
#endif
