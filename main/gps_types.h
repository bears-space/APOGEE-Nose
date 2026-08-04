#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} gps_utc_t;

typedef struct {
    char talker[3];
    uint8_t signal_id;
    uint16_t satellites_in_view;
    uint16_t satellites_with_cno;
    uint8_t strongest_cno_dbhz;
    double average_cno_dbhz;
    int64_t last_update_ms;
} gps_signal_info_t;

/** Latest navigation data received from the GPS.
 *
 * Values remain available after a fix is lost so callers can inspect the last
 * known position. Check the corresponding validity flag and last_fix_ms before
 * using a value for navigation.
 */
typedef struct {
    bool fix_valid;
    bool time_valid;
    bool date_valid;
    bool altitude_valid;
    bool speed_valid;
    bool course_valid;
    bool hdop_valid;
    bool dop_valid;
    bool signal_valid;

    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double speed_mps;
    double course_deg;
    double hdop;
    double pdop;
    double vdop;

    uint8_t fix_quality;
    uint8_t fix_dimension;
    uint8_t satellites_used;
    gps_utc_t utc;
    gps_signal_info_t signal;

    int64_t last_sentence_ms;
    int64_t last_fix_ms;
} gps_data_t;

#ifdef __cplusplus
}
#endif
