#include "gps_nmea.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void make_sentence(const char* payload, char* sentence,
                          size_t sentence_size) {
    uint8_t checksum = 0;
    for (const char* cursor = payload; *cursor != '\0'; ++cursor) {
        checksum ^= (uint8_t)*cursor;
    }

    int written = snprintf(sentence, sentence_size, "$%s*%02X", payload,
                           (unsigned int)checksum);
    assert(written > 0 && (size_t)written < sentence_size);
}

static void test_gga_fix(void) {
    char sentence[160];
    make_sentence(
        "GNGGA,123519.50,4807.038,N,01131.000,E,1,08,0.9,"
        "545.4,M,46.9,M,,",
        sentence, sizeof(sentence));

    gps_data_t data = {0};
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);
    assert(gps_nmea_parse(&parser, sentence, 1234, &data) == GPS_NMEA_GGA);
    assert(data.fix_valid);
    assert(data.time_valid);
    assert(data.altitude_valid);
    assert(data.hdop_valid);
    assert(fabs(data.latitude_deg - 48.1173) < 0.0000001);
    assert(fabs(data.longitude_deg - 11.5166666667) < 0.0000001);
    assert(fabs(data.altitude_m - 545.4) < 0.0001);
    assert(fabs(data.hdop - 0.9) < 0.0001);
    assert(data.fix_quality == 1);
    assert(data.satellites_used == 8);
    assert(data.utc.hour == 12 && data.utc.minute == 35 &&
           data.utc.second == 19 && data.utc.millisecond == 500);
    assert(data.last_sentence_ms == 1234 && data.last_fix_ms == 1234);
}

static void test_rmc_fix(void) {
    char sentence[160];
    make_sentence(
        "GNRMC,123520.25,A,4807.038,S,01131.000,W,10.0,84.4,"
        "230394,,,A,V",
        sentence, sizeof(sentence));

    gps_data_t data = {0};
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);
    assert(gps_nmea_parse(&parser, sentence, 5678, &data) == GPS_NMEA_RMC);
    assert(data.fix_valid);
    assert(data.date_valid);
    assert(data.speed_valid);
    assert(data.course_valid);
    assert(fabs(data.latitude_deg + 48.1173) < 0.0000001);
    assert(fabs(data.longitude_deg + 11.5166666667) < 0.0000001);
    assert(fabs(data.speed_mps - 5.1444444444) < 0.0000001);
    assert(fabs(data.course_deg - 84.4) < 0.0001);
    assert(data.utc.year == 1994 && data.utc.month == 3 && data.utc.day == 23);
}

static void test_void_fix_and_checksum_rejection(void) {
    char sentence[160];
    gps_data_t data = {.fix_valid = true,
                       .altitude_valid = true,
                       .speed_valid = true,
                       .course_valid = true,
                       .latitude_deg = 12.5,
                       .last_fix_ms = 100};
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);

    make_sentence("GNRMC,123521.00,V,,,,,,,230394,,,N,V", sentence,
                  sizeof(sentence));
    assert(gps_nmea_parse(&parser, sentence, 200, &data) == GPS_NMEA_RMC);
    assert(!data.fix_valid);
    assert(!data.altitude_valid && !data.speed_valid && !data.course_valid);
    assert(data.latitude_deg == 12.5);
    assert(data.last_fix_ms == 100);
    assert(data.last_sentence_ms == 200);

    make_sentence(
        "GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,"
        "545.4,M,46.9,M,,",
        sentence, sizeof(sentence));
    sentence[strlen(sentence) - 1] =
        sentence[strlen(sentence) - 1] == '0' ? '1' : '0';
    gps_data_t before = data;
    assert(gps_nmea_parse(&parser, sentence, 300, &data) ==
           GPS_NMEA_BAD_CHECKSUM);
    assert(memcmp(&data, &before, sizeof(data)) == 0);
}

static void test_gsa_diagnostics(void) {
    char sentence[160];
    make_sentence("GNGSA,A,3,09,10,12,13,15,17,19,24,25,,,,1.94,1.18,1.54,1",
                  sentence, sizeof(sentence));

    gps_data_t data = {0};
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);
    assert(gps_nmea_parse(&parser, sentence, 7000, &data) == GPS_NMEA_GSA);
    assert(data.fix_dimension == 3);
    assert(data.dop_valid && data.hdop_valid);
    assert(fabs(data.pdop - 1.94) < 0.0001);
    assert(fabs(data.hdop - 1.18) < 0.0001);
    assert(fabs(data.vdop - 1.54) < 0.0001);
    assert(data.last_sentence_ms == 7000);
}

static void test_multi_sentence_gsv_diagnostics(void) {
    const char* payloads[] = {
        "GPGSV,3,1,09,09,,,17,10,,,40,12,,,49,13,,,35,1",
        "GPGSV,3,2,09,15,,,44,17,,,45,19,,,44,24,,,50,1",
        "GPGSV,3,3,09,25,,,40,1",
    };
    char sentence[160];
    gps_data_t data = {0};
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);

    make_sentence(payloads[0], sentence, sizeof(sentence));
    assert(gps_nmea_parse(&parser, sentence, 8000, &data) ==
           GPS_NMEA_GSV_PARTIAL);
    assert(!data.signal_valid);
    make_sentence(payloads[1], sentence, sizeof(sentence));
    assert(gps_nmea_parse(&parser, sentence, 8001, &data) ==
           GPS_NMEA_GSV_PARTIAL);
    make_sentence(payloads[2], sentence, sizeof(sentence));
    assert(gps_nmea_parse(&parser, sentence, 8002, &data) == GPS_NMEA_GSV);

    assert(data.signal_valid);
    assert(strcmp(data.signal.talker, "GP") == 0);
    assert(data.signal.signal_id == 1);
    assert(data.signal.satellites_in_view == 9);
    assert(data.signal.satellites_with_cno == 9);
    assert(data.signal.strongest_cno_dbhz == 50);
    assert(fabs(data.signal.average_cno_dbhz - 40.4444444444) < 0.0000001);
    assert(data.signal.last_update_ms == 8002);
}

int main(void) {
    test_gga_fix();
    test_rmc_fix();
    test_void_fix_and_checksum_rejection();
    test_gsa_diagnostics();
    test_multi_sentence_gsv_diagnostics();
    puts("gps_nmea_test: OK");
    return 0;
}
