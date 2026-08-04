#include "gps_nmea.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_MAX_FIELDS 24
#define KNOTS_TO_METERS_PER_SECOND 0.5144444444444444

static int hex_value(char value) {
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    value = (char)toupper((unsigned char)value);
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    return -1;
}

static bool checksum_valid(char* sentence, char** payload) {
    if (!sentence || sentence[0] != '$') {
        return false;
    }

    char* checksum = strchr(sentence, '*');
    if (!checksum || checksum[1] == '\0' || checksum[2] == '\0' ||
        checksum[3] != '\0') {
        return false;
    }

    int high = hex_value(checksum[1]);
    int low = hex_value(checksum[2]);
    if (high < 0 || low < 0) {
        return false;
    }

    uint8_t calculated = 0;
    for (char* cursor = sentence + 1; cursor < checksum; ++cursor) {
        calculated ^= (uint8_t)*cursor;
    }
    if (calculated != (uint8_t)((high << 4) | low)) {
        return false;
    }

    *checksum = '\0';
    *payload = sentence + 1;
    return true;
}

static bool split_fields(char* payload, char** fields, size_t* field_count) {
    size_t count = 0;
    char* field = payload;

    while (true) {
        if (count == NMEA_MAX_FIELDS) {
            return false;
        }
        fields[count++] = field;

        char* separator = strchr(field, ',');
        if (!separator) {
            break;
        }
        *separator = '\0';
        field = separator + 1;
    }

    *field_count = count;
    return true;
}

static bool parse_unsigned(const char* field, unsigned int maximum,
                           unsigned int* value) {
    if (!field || field[0] == '\0') {
        return false;
    }

    char* end = NULL;
    unsigned long parsed = strtoul(field, &end, 10);
    if (*end != '\0' || parsed > maximum) {
        return false;
    }

    *value = (unsigned int)parsed;
    return true;
}

static bool parse_hex_unsigned(const char* field, unsigned int maximum,
                               unsigned int* value) {
    if (!field || field[0] == '\0' || isspace((unsigned char)field[0])) {
        return false;
    }

    char* end = NULL;
    unsigned long parsed = strtoul(field, &end, 16);
    if (*end != '\0' || parsed > maximum) {
        return false;
    }

    *value = (unsigned int)parsed;
    return true;
}

static bool parse_decimal(const char* field, double* value) {
    if (!field || field[0] == '\0' || isspace((unsigned char)field[0])) {
        return false;
    }

    char* end = NULL;
    double parsed = strtod(field, &end);
    if (*end != '\0' || !isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool parse_time(const char* field, gps_utc_t* utc) {
    size_t length = strlen(field);
    if (length < 6) {
        return false;
    }
    for (size_t index = 0; index < 6; ++index) {
        if (!isdigit((unsigned char)field[index])) {
            return false;
        }
    }

    unsigned int hour =
        (unsigned int)(field[0] - '0') * 10U + (unsigned int)(field[1] - '0');
    unsigned int minute =
        (unsigned int)(field[2] - '0') * 10U + (unsigned int)(field[3] - '0');
    unsigned int second =
        (unsigned int)(field[4] - '0') * 10U + (unsigned int)(field[5] - '0');
    if (hour > 23 || minute > 59 || second > 60) {
        return false;
    }

    uint16_t millisecond = 0;
    if (length > 6) {
        if (field[6] != '.' || length == 7) {
            return false;
        }
        uint16_t place = 100;
        for (size_t index = 7; index < length; ++index) {
            if (!isdigit((unsigned char)field[index])) {
                return false;
            }
            if (place > 0) {
                millisecond += (uint16_t)(field[index] - '0') * place;
                place /= 10;
            }
        }
    }

    utc->hour = (uint8_t)hour;
    utc->minute = (uint8_t)minute;
    utc->second = (uint8_t)second;
    utc->millisecond = millisecond;
    return true;
}

static bool parse_date(const char* field, gps_utc_t* utc) {
    if (strlen(field) != 6) {
        return false;
    }
    for (size_t index = 0; index < 6; ++index) {
        if (!isdigit((unsigned char)field[index])) {
            return false;
        }
    }

    unsigned int day =
        (unsigned int)(field[0] - '0') * 10U + (unsigned int)(field[1] - '0');
    unsigned int month =
        (unsigned int)(field[2] - '0') * 10U + (unsigned int)(field[3] - '0');
    unsigned int short_year =
        (unsigned int)(field[4] - '0') * 10U + (unsigned int)(field[5] - '0');
    if (day == 0 || day > 31 || month == 0 || month > 12) {
        return false;
    }

    utc->day = (uint8_t)day;
    utc->month = (uint8_t)month;
    utc->year =
        (uint16_t)(short_year < 80 ? 2000 + short_year : 1900 + short_year);
    return true;
}

static bool parse_coordinate(const char* field, const char* hemisphere,
                             bool latitude, double* coordinate) {
    double raw = 0;
    if (!parse_decimal(field, &raw) || raw < 0 || !hemisphere ||
        hemisphere[0] == '\0' || hemisphere[1] != '\0') {
        return false;
    }

    int degrees = (int)(raw / 100.0);
    double minutes = raw - (double)degrees * 100.0;
    int maximum_degrees = latitude ? 90 : 180;
    if (degrees > maximum_degrees || minutes < 0 || minutes >= 60 ||
        (degrees == maximum_degrees && minutes > 0)) {
        return false;
    }

    char direction = (char)toupper((unsigned char)hemisphere[0]);
    if ((latitude && direction != 'N' && direction != 'S') ||
        (!latitude && direction != 'E' && direction != 'W')) {
        return false;
    }

    double decimal_degrees = (double)degrees + minutes / 60.0;
    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    *coordinate = decimal_degrees;
    return true;
}

static bool message_is(const char* identifier, const char* type) {
    size_t length = strlen(identifier);
    return length == 5 && strcmp(identifier + 2, type) == 0;
}

static bool parse_gga(char** fields, size_t count, int64_t received_ms,
                      gps_data_t* data) {
    if (count < 10) {
        return false;
    }

    gps_data_t next = *data;
    unsigned int quality = 0;
    unsigned int satellites = 0;
    if (!parse_unsigned(fields[6], UINT8_MAX, &quality)) {
        return false;
    }

    if (fields[1][0] != '\0') {
        if (!parse_time(fields[1], &next.utc)) {
            return false;
        }
        next.time_valid = true;
    } else {
        next.time_valid = false;
    }

    if (fields[7][0] != '\0') {
        if (!parse_unsigned(fields[7], UINT8_MAX, &satellites)) {
            return false;
        }
        next.satellites_used = (uint8_t)satellites;
    } else {
        next.satellites_used = 0;
    }

    if (fields[8][0] != '\0') {
        if (!parse_decimal(fields[8], &next.hdop) || next.hdop < 0) {
            return false;
        }
        next.hdop_valid = true;
    } else {
        next.hdop_valid = false;
    }

    next.fix_quality = (uint8_t)quality;
    next.last_sentence_ms = received_ms;
    if (quality == 0) {
        next.fix_valid = false;
        next.altitude_valid = false;
        next.speed_valid = false;
        next.course_valid = false;
        *data = next;
        return true;
    }

    if (!parse_coordinate(fields[2], fields[3], true, &next.latitude_deg) ||
        !parse_coordinate(fields[4], fields[5], false, &next.longitude_deg)) {
        return false;
    }

    if (fields[9][0] != '\0') {
        if (!parse_decimal(fields[9], &next.altitude_m)) {
            return false;
        }
        next.altitude_valid = count <= 10 || fields[10][0] == '\0' ||
                              strcmp(fields[10], "M") == 0;
    } else {
        next.altitude_valid = false;
    }

    next.fix_valid = true;
    next.last_fix_ms = received_ms;
    *data = next;
    return true;
}

static bool parse_rmc(char** fields, size_t count, int64_t received_ms,
                      gps_data_t* data) {
    if (count < 10 || fields[2][0] == '\0' || fields[2][1] != '\0') {
        return false;
    }

    char status = (char)toupper((unsigned char)fields[2][0]);
    if (status != 'A' && status != 'V') {
        return false;
    }

    gps_data_t next = *data;
    if (fields[1][0] != '\0') {
        if (!parse_time(fields[1], &next.utc)) {
            return false;
        }
        next.time_valid = true;
    } else {
        next.time_valid = false;
    }
    if (fields[9][0] != '\0') {
        if (!parse_date(fields[9], &next.utc)) {
            return false;
        }
        next.date_valid = true;
    } else {
        next.date_valid = false;
    }

    next.last_sentence_ms = received_ms;
    if (status == 'V') {
        next.fix_valid = false;
        next.altitude_valid = false;
        next.speed_valid = false;
        next.course_valid = false;
        *data = next;
        return true;
    }

    if (!parse_coordinate(fields[3], fields[4], true, &next.latitude_deg) ||
        !parse_coordinate(fields[5], fields[6], false, &next.longitude_deg)) {
        return false;
    }

    if (fields[7][0] != '\0') {
        double speed_knots = 0;
        if (!parse_decimal(fields[7], &speed_knots) || speed_knots < 0) {
            return false;
        }
        next.speed_mps = speed_knots * KNOTS_TO_METERS_PER_SECOND;
        next.speed_valid = true;
    } else {
        next.speed_valid = false;
    }

    if (fields[8][0] != '\0') {
        if (!parse_decimal(fields[8], &next.course_deg) ||
            next.course_deg < 0 || next.course_deg > 360) {
            return false;
        }
        next.course_valid = true;
    } else {
        next.course_valid = false;
    }

    next.fix_valid = true;
    next.last_fix_ms = received_ms;
    *data = next;
    return true;
}

static bool parse_gsa(char** fields, size_t count, int64_t received_ms,
                      gps_data_t* data) {
    if (count < 18) {
        return false;
    }

    unsigned int fix_dimension = 0;
    if (!parse_unsigned(fields[2], 3, &fix_dimension) || fix_dimension == 0) {
        return false;
    }

    gps_data_t next = *data;
    bool has_dop =
        fields[15][0] != '\0' || fields[16][0] != '\0' || fields[17][0] != '\0';
    if (has_dop) {
        if (!parse_decimal(fields[15], &next.pdop) || next.pdop < 0 ||
            !parse_decimal(fields[16], &next.hdop) || next.hdop < 0 ||
            !parse_decimal(fields[17], &next.vdop) || next.vdop < 0) {
            return false;
        }
        next.dop_valid = true;
        next.hdop_valid = true;
    } else {
        next.dop_valid = false;
    }

    next.fix_dimension = (uint8_t)fix_dimension;
    next.last_sentence_ms = received_ms;
    *data = next;
    return true;
}

static void reset_gsv(gps_nmea_parser_t* parser) {
    parser->gsv_active = false;
    parser->gsv_talker[0] = '\0';
    parser->gsv_talker[1] = '\0';
    parser->gsv_talker[2] = '\0';
    parser->gsv_signal_id = 0;
    parser->gsv_message_count = 0;
    parser->gsv_next_message = 0;
    parser->gsv_satellites_in_view = 0;
    parser->gsv_satellites_with_cno = 0;
    parser->gsv_cno_sum = 0;
    parser->gsv_strongest_cno = 0;
}

static gps_nmea_result_t parse_gsv(gps_nmea_parser_t* parser, char** fields,
                                   size_t count, int64_t received_ms,
                                   gps_data_t* data) {
    if (count < 4) {
        return GPS_NMEA_MALFORMED;
    }

    unsigned int message_count = 0;
    unsigned int message_number = 0;
    unsigned int satellites_in_view = 0;
    if (!parse_unsigned(fields[1], UINT8_MAX, &message_count) ||
        message_count == 0 ||
        !parse_unsigned(fields[2], UINT8_MAX, &message_number) ||
        message_number == 0 || message_number > message_count ||
        !parse_unsigned(fields[3], UINT16_MAX, &satellites_in_view)) {
        return GPS_NMEA_MALFORMED;
    }

    size_t detail_count = count - 4;
    bool signal_id_present = detail_count % 4 == 1;
    if (!signal_id_present && detail_count % 4 != 0) {
        return GPS_NMEA_MALFORMED;
    }
    size_t satellite_count = detail_count / 4;
    if (signal_id_present) {
        satellite_count = (detail_count - 1) / 4;
    }
    if (satellite_count > 4) {
        return GPS_NMEA_MALFORMED;
    }

    unsigned int signal_id = 0;
    if (signal_id_present && fields[count - 1][0] != '\0' &&
        !parse_hex_unsigned(fields[count - 1], UINT8_MAX, &signal_id)) {
        return GPS_NMEA_MALFORMED;
    }

    const char* identifier = fields[0];
    if (message_number == 1) {
        reset_gsv(parser);
        parser->gsv_active = true;
        parser->gsv_talker[0] = identifier[0];
        parser->gsv_talker[1] = identifier[1];
        parser->gsv_talker[2] = '\0';
        parser->gsv_signal_id = (uint8_t)signal_id;
        parser->gsv_message_count = (uint8_t)message_count;
        parser->gsv_next_message = 1;
        parser->gsv_satellites_in_view = (uint16_t)satellites_in_view;
    }

    if (!parser->gsv_active || parser->gsv_talker[0] != identifier[0] ||
        parser->gsv_talker[1] != identifier[1] ||
        parser->gsv_signal_id != (uint8_t)signal_id ||
        parser->gsv_message_count != (uint8_t)message_count ||
        parser->gsv_next_message != (uint8_t)message_number ||
        parser->gsv_satellites_in_view != (uint16_t)satellites_in_view) {
        reset_gsv(parser);
        return GPS_NMEA_MALFORMED;
    }

    for (size_t satellite = 0; satellite < satellite_count; ++satellite) {
        const char* cno_field = fields[7 + satellite * 4];
        if (cno_field[0] == '\0') {
            continue;
        }

        unsigned int cno = 0;
        if (!parse_unsigned(cno_field, 99, &cno)) {
            reset_gsv(parser);
            return GPS_NMEA_MALFORMED;
        }
        parser->gsv_satellites_with_cno++;
        parser->gsv_cno_sum += (uint16_t)cno;
        if (cno > parser->gsv_strongest_cno) {
            parser->gsv_strongest_cno = (uint8_t)cno;
        }
    }

    data->last_sentence_ms = received_ms;
    parser->gsv_next_message++;
    if (message_number < message_count) {
        return GPS_NMEA_GSV_PARTIAL;
    }

    data->signal_valid = true;
    data->signal.talker[0] = parser->gsv_talker[0];
    data->signal.talker[1] = parser->gsv_talker[1];
    data->signal.talker[2] = '\0';
    data->signal.signal_id = parser->gsv_signal_id;
    data->signal.satellites_in_view = parser->gsv_satellites_in_view;
    data->signal.satellites_with_cno = parser->gsv_satellites_with_cno;
    data->signal.strongest_cno_dbhz = parser->gsv_strongest_cno;
    data->signal.average_cno_dbhz =
        parser->gsv_satellites_with_cno > 0
            ? (double)parser->gsv_cno_sum /
                  (double)parser->gsv_satellites_with_cno
            : 0;
    data->signal.last_update_ms = received_ms;
    reset_gsv(parser);
    return GPS_NMEA_GSV;
}

void gps_nmea_parser_init(gps_nmea_parser_t* parser) {
    if (parser) {
        memset(parser, 0, sizeof(*parser));
    }
}

gps_nmea_result_t gps_nmea_parse(gps_nmea_parser_t* parser, char* sentence,
                                 int64_t received_ms, gps_data_t* data) {
    if (!parser || !sentence || !data) {
        return GPS_NMEA_MALFORMED;
    }

    char* payload = NULL;
    if (!checksum_valid(sentence, &payload)) {
        return GPS_NMEA_BAD_CHECKSUM;
    }

    char* fields[NMEA_MAX_FIELDS] = {0};
    size_t count = 0;
    if (!split_fields(payload, fields, &count) || count == 0) {
        return GPS_NMEA_MALFORMED;
    }

    if (message_is(fields[0], "GGA")) {
        return parse_gga(fields, count, received_ms, data) ? GPS_NMEA_GGA
                                                           : GPS_NMEA_MALFORMED;
    }
    if (message_is(fields[0], "GSA")) {
        return parse_gsa(fields, count, received_ms, data) ? GPS_NMEA_GSA
                                                           : GPS_NMEA_MALFORMED;
    }
    if (message_is(fields[0], "GSV")) {
        return parse_gsv(parser, fields, count, received_ms, data);
    }
    if (message_is(fields[0], "RMC")) {
        return parse_rmc(fields, count, received_ms, data) ? GPS_NMEA_RMC
                                                           : GPS_NMEA_MALFORMED;
    }
    return GPS_NMEA_IGNORED;
}
