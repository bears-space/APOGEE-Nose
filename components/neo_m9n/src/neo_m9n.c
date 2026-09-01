/*
 * SPDX-FileCopyrightText: 2026 STARSTREAK
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "neo_m9n.h"

neo_m9n_handle_t neo_m9n_init(const nmea_parser_config_t* config) {
    return nmea_parser_init(config);
}

esp_err_t neo_m9n_deinit(neo_m9n_handle_t neo_m9n_hdl) {
    return nmea_parser_deinit(neo_m9n_hdl);
}

esp_err_t neo_m9n_add_handler(neo_m9n_handle_t neo_m9n_hdl,
                              esp_event_handler_t event_handler,
                              void* handler_args) {
    return nmea_parser_add_handler(neo_m9n_hdl, event_handler, handler_args);
}

esp_err_t neo_m9n_remove_handler(neo_m9n_handle_t neo_m9n_hdl,
                                 esp_event_handler_t event_handler) {
    return nmea_parser_remove_handler(neo_m9n_hdl, event_handler);
}
