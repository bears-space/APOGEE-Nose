/*
 * SPDX-FileCopyrightText: 2026 STARSTREAK
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "nmea_parser.h"

/**
 * @brief NEO-M9N handle
 *
 */
typedef nmea_parser_handle_t neo_m9n_handle_t;

/**
 * @brief Default configuration for the NEO-M9N-00B GPS module
 *
 */
#define NEO_M9N_CONFIG_DEFAULT()                     \
    {.uart = {.uart_port = UART_NUM_1,               \
              .rx_pin = CONFIG_NMEA_PARSER_UART_RXD, \
              .baud_rate = CONFIG_NEO_M9N_BAUD_RATE, \
              .data_bits = UART_DATA_8_BITS,         \
              .parity = UART_PARITY_DISABLE,         \
              .stop_bits = UART_STOP_BITS_1,         \
              .event_queue_size = 16}}

/**
 * @brief Init NEO-M9N GPS driver
 *
 * @param config Configuration of the NEO-M9N GPS driver
 * @return neo_m9n_handle_t handle of NEO-M9N driver
 */
neo_m9n_handle_t neo_m9n_init(const nmea_parser_config_t* config);

/**
 * @brief Deinit NEO-M9N GPS driver
 *
 * @param neo_m9n_hdl handle of NEO-M9N driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t neo_m9n_deinit(neo_m9n_handle_t neo_m9n_hdl);

/**
 * @brief Add user defined handler for NEO-M9N GPS driver
 *
 * @param neo_m9n_hdl handle of NEO-M9N driver
 * @param event_handler user defined event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_NO_MEM: Cannot allocate memory for the handler
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t neo_m9n_add_handler(neo_m9n_handle_t neo_m9n_hdl,
                              esp_event_handler_t event_handler,
                              void* handler_args);

/**
 * @brief Remove user defined handler for NEO-M9N GPS driver
 *
 * @param neo_m9n_hdl handle of NEO-M9N driver
 * @param event_handler user defined event handler
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t neo_m9n_remove_handler(neo_m9n_handle_t neo_m9n_hdl,
                                 esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
