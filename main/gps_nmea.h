#pragma once

#include <stdint.h>

#include "gps_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPS_NMEA_IGNORED = 0,
    GPS_NMEA_GGA,
    GPS_NMEA_GSA,
    GPS_NMEA_GSV_PARTIAL,
    GPS_NMEA_GSV,
    GPS_NMEA_RMC,
    GPS_NMEA_BAD_CHECKSUM,
    GPS_NMEA_MALFORMED,
} gps_nmea_result_t;

typedef struct {
    bool gsv_active;
    char gsv_talker[3];
    uint8_t gsv_signal_id;
    uint8_t gsv_message_count;
    uint8_t gsv_next_message;
    uint16_t gsv_satellites_in_view;
    uint16_t gsv_satellites_with_cno;
    uint16_t gsv_cno_sum;
    uint8_t gsv_strongest_cno;
} gps_nmea_parser_t;

void gps_nmea_parser_init(gps_nmea_parser_t* parser);

/**
 * Validate and parse one mutable NMEA sentence into an existing data snapshot.
 * The sentence must include '$' and '*HH'; trailing CR/LF must be removed.
 */
gps_nmea_result_t gps_nmea_parse(gps_nmea_parser_t* parser, char* sentence,
                                 int64_t received_ms, gps_data_t* data);

#ifdef __cplusplus
}
#endif
