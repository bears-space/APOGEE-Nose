#include "gps.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gps_nmea.h"
#include "sdkconfig.h"

#define GPS_RX_BUFFER_SIZE 4096
#define GPS_READ_BUFFER_SIZE 256
#define GPS_SENTENCE_SIZE 256
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY 5
#define GPS_NO_DATA_WARNING_MS 5000
#define GPS_FIX_LOG_INTERVAL_MS 10000
#define GPS_SIGNAL_LOG_INTERVAL_MS 10000
#define GPS_SIGNAL_LOG_SLOTS 12
#define GPS_NMEA_SUMMARY_INTERVAL_MS 10000

typedef struct {
    bool used;
    char talker[3];
    uint8_t signal_id;
    int64_t last_log_ms;
} gps_signal_log_slot_t;

typedef struct {
    uint32_t gga;
    uint32_t rmc;
    uint32_t gsa;
    uint32_t gsv;
    uint32_t other;
    uint32_t bad_checksum;
    uint32_t malformed;
    int64_t interval_started_ms;
    bool missing_diagnostics_warned;
} gps_nmea_stats_t;

static const char* TAG = "gps";

static SemaphoreHandle_t s_data_mutex = NULL;
static SemaphoreHandle_t s_task_stopped = NULL;
static TaskHandle_t s_task = NULL;
static gps_data_t s_data = {0};
static bool s_uart_installed = false;
static volatile bool s_running = false;

static int64_t uptime_ms(void) { return esp_timer_get_time() / 1000; }

static const char* gnss_name(const char* talker) {
    if (strcmp(talker, "GP") == 0) return "GPS/SBAS";
    if (strcmp(talker, "GL") == 0) return "GLONASS";
    if (strcmp(talker, "GA") == 0) return "Galileo";
    if (strcmp(talker, "GB") == 0 || strcmp(talker, "BD") == 0) return "BeiDou";
    if (strcmp(talker, "GQ") == 0) return "QZSS";
    if (strcmp(talker, "GN") == 0) return "combined GNSS";
    return talker;
}

static const char* signal_quality(double average_cno_dbhz) {
    if (average_cno_dbhz < 20) return "very weak";
    if (average_cno_dbhz < 30) return "weak";
    if (average_cno_dbhz < 38) return "usable";
    return "strong";
}

static gps_signal_log_slot_t* signal_log_slot(
    gps_signal_log_slot_t slots[GPS_SIGNAL_LOG_SLOTS], const char* talker,
    uint8_t signal_id) {
    gps_signal_log_slot_t* oldest = &slots[0];
    for (size_t index = 0; index < GPS_SIGNAL_LOG_SLOTS; ++index) {
        if (slots[index].used && slots[index].signal_id == signal_id &&
            strcmp(slots[index].talker, talker) == 0) {
            return &slots[index];
        }
        if (!slots[index].used) {
            return &slots[index];
        }
        if (slots[index].last_log_ms < oldest->last_log_ms) {
            oldest = &slots[index];
        }
    }
    return oldest;
}

static void log_signal_update(
    const gps_data_t* data, gps_signal_log_slot_t slots[GPS_SIGNAL_LOG_SLOTS]) {
#if CONFIG_GPS_LOG_DIAGNOSTICS
    if (!data->signal_valid) return;

    gps_signal_log_slot_t* slot =
        signal_log_slot(slots, data->signal.talker, data->signal.signal_id);
    if (slot->used && data->signal.last_update_ms - slot->last_log_ms <
                          GPS_SIGNAL_LOG_INTERVAL_MS) {
        return;
    }

    slot->used = true;
    memcpy(slot->talker, data->signal.talker, sizeof(slot->talker));
    slot->signal_id = data->signal.signal_id;
    slot->last_log_ms = data->signal.last_update_ms;

    if (data->signal.satellites_with_cno == 0) {
        ESP_LOGI(TAG,
                 "Signal %s (signal %X): %u satellites in view, none with "
                 "C/N0",
                 gnss_name(data->signal.talker),
                 (unsigned int)data->signal.signal_id,
                 (unsigned int)data->signal.satellites_in_view);
        return;
    }

    ESP_LOGI(TAG,
             "Signal %s (signal %X): view=%u tracked=%u avg=%.1f max=%u "
             "dB-Hz (%s)",
             gnss_name(data->signal.talker),
             (unsigned int)data->signal.signal_id,
             (unsigned int)data->signal.satellites_in_view,
             (unsigned int)data->signal.satellites_with_cno,
             data->signal.average_cno_dbhz,
             (unsigned int)data->signal.strongest_cno_dbhz,
             signal_quality(data->signal.average_cno_dbhz));
#else
    (void)data;
    (void)slots;
#endif
}

static void log_fix_dimension(const gps_data_t* data, bool* dimension_known,
                              uint8_t* last_dimension) {
#if CONFIG_GPS_LOG_DIAGNOSTICS
    if (*dimension_known && data->fix_dimension == *last_dimension) return;

    const char* mode = "unknown";
    if (data->fix_dimension == 1) mode = "no fix";
    if (data->fix_dimension == 2) mode = "2D fix";
    if (data->fix_dimension == 3) mode = "3D fix";

    if (data->dop_valid) {
        ESP_LOGI(TAG, "Navigation mode: %s, PDOP=%.2f HDOP=%.2f VDOP=%.2f",
                 mode, data->pdop, data->hdop, data->vdop);
    } else {
        ESP_LOGI(TAG, "Navigation mode: %s, DOP unavailable", mode);
    }
    *dimension_known = true;
    *last_dimension = data->fix_dimension;
#else
    (void)data;
    (void)dimension_known;
    (void)last_dimension;
#endif
}

static void log_navigation_update(const gps_data_t* data, bool* state_known,
                                  bool* last_fix_valid,
                                  int64_t* last_fix_log_ms) {
#if CONFIG_GPS_LOG_FIXES
    if (!*state_known || data->fix_valid != *last_fix_valid) {
        if (data->fix_valid) {
            ESP_LOGI(TAG, "GNSS fix acquired");
        } else {
            ESP_LOGI(TAG, "NMEA data received; waiting for a GNSS fix");
        }
        *state_known = true;
        *last_fix_valid = data->fix_valid;
    }

    if (data->fix_valid &&
        (*last_fix_log_ms == 0 ||
         data->last_fix_ms - *last_fix_log_ms >= GPS_FIX_LOG_INTERVAL_MS)) {
        ESP_LOGI(TAG, "Fix: lat=%.7f lon=%.7f alt=%.2f m sats=%u hdop=%.2f",
                 data->latitude_deg, data->longitude_deg,
                 data->altitude_valid ? data->altitude_m : 0.0,
                 (unsigned int)data->satellites_used,
                 data->hdop_valid ? data->hdop : 0.0);
        *last_fix_log_ms = data->last_fix_ms;
    }
#else
    (void)data;
    (void)state_known;
    (void)last_fix_valid;
    (void)last_fix_log_ms;
#endif
}

static void count_nmea_result(gps_nmea_stats_t* stats,
                              gps_nmea_result_t result) {
    switch (result) {
        case GPS_NMEA_GGA:
            stats->gga++;
            break;
        case GPS_NMEA_RMC:
            stats->rmc++;
            break;
        case GPS_NMEA_GSA:
            stats->gsa++;
            break;
        case GPS_NMEA_GSV_PARTIAL:
        case GPS_NMEA_GSV:
            stats->gsv++;
            break;
        case GPS_NMEA_BAD_CHECKSUM:
            stats->bad_checksum++;
            break;
        case GPS_NMEA_MALFORMED:
            stats->malformed++;
            break;
        case GPS_NMEA_IGNORED:
        default:
            stats->other++;
            break;
    }
}

static void log_nmea_summary(gps_nmea_stats_t* stats, int64_t now_ms) {
#if CONFIG_GPS_LOG_DIAGNOSTICS
    if (now_ms - stats->interval_started_ms < GPS_NMEA_SUMMARY_INTERVAL_MS) {
        return;
    }

    ESP_LOGI(TAG,
             "NMEA/10s: GGA=%" PRIu32 " RMC=%" PRIu32 " GSA=%" PRIu32
             " GSV=%" PRIu32 " other=%" PRIu32 " bad_checksum=%" PRIu32
             " malformed=%" PRIu32,
             stats->gga, stats->rmc, stats->gsa, stats->gsv, stats->other,
             stats->bad_checksum, stats->malformed);

    if (!stats->missing_diagnostics_warned && stats->gsa == 0 &&
        stats->gsv == 0) {
        ESP_LOGW(TAG,
                 "No GSA/GSV sentences received; C/N0 signal diagnostics are "
                 "unavailable until NMEA-GSA and NMEA-GSV are enabled on the "
                 "GPS UART output");
        stats->missing_diagnostics_warned = true;
    }

    stats->gga = 0;
    stats->rmc = 0;
    stats->gsa = 0;
    stats->gsv = 0;
    stats->other = 0;
    stats->bad_checksum = 0;
    stats->malformed = 0;
    stats->interval_started_ms = now_ms;
#else
    (void)stats;
    (void)now_ms;
#endif
}

static void process_sentence(
    gps_nmea_parser_t* parser, char* sentence, bool* valid_nmea_seen,
    bool* state_known, bool* last_fix_valid, int64_t* last_fix_log_ms,
    bool* dimension_known, uint8_t* last_dimension,
    gps_signal_log_slot_t signal_slots[GPS_SIGNAL_LOG_SLOTS],
    gps_nmea_stats_t* stats) {
    gps_data_t snapshot = {0};
    gps_nmea_result_t result = GPS_NMEA_IGNORED;

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "GPS data lock timed out; dropping NMEA sentence");
        return;
    }
    result = gps_nmea_parse(parser, sentence, uptime_ms(), &s_data);
    snapshot = s_data;
    xSemaphoreGive(s_data_mutex);
    count_nmea_result(stats, result);

    if (result == GPS_NMEA_GGA || result == GPS_NMEA_GSA ||
        result == GPS_NMEA_GSV_PARTIAL || result == GPS_NMEA_GSV ||
        result == GPS_NMEA_RMC) {
        *valid_nmea_seen = true;
    }
    if (result == GPS_NMEA_GGA || result == GPS_NMEA_RMC) {
        log_navigation_update(&snapshot, state_known, last_fix_valid,
                              last_fix_log_ms);
    } else if (result == GPS_NMEA_GSA) {
        log_fix_dimension(&snapshot, dimension_known, last_dimension);
    } else if (result == GPS_NMEA_GSV) {
        log_signal_update(&snapshot, signal_slots);
    } else if (result == GPS_NMEA_BAD_CHECKSUM) {
        ESP_LOGD(TAG, "Discarded NMEA sentence with an invalid checksum");
    } else if (result == GPS_NMEA_MALFORMED) {
        ESP_LOGD(TAG, "Discarded malformed NMEA sentence");
    }
}

static void gps_task(void* argument) {
    (void)argument;
    uint8_t read_buffer[GPS_READ_BUFFER_SIZE];
    char sentence[GPS_SENTENCE_SIZE];
    size_t sentence_length = 0;
    bool collecting = false;
    bool valid_nmea_seen = false;
    bool no_data_warning_logged = false;
    bool state_known = false;
    bool last_fix_valid = false;
    int64_t last_fix_log_ms = 0;
    int64_t started_ms = uptime_ms();
    bool dimension_known = false;
    uint8_t last_dimension = 0;
    gps_nmea_parser_t parser;
    gps_nmea_parser_init(&parser);
    gps_signal_log_slot_t signal_slots[GPS_SIGNAL_LOG_SLOTS] = {0};
    gps_nmea_stats_t stats = {.interval_started_ms = started_ms};

    while (s_running) {
        int received =
            uart_read_bytes((uart_port_t)CONFIG_GPS_UART_PORT, read_buffer,
                            sizeof(read_buffer), pdMS_TO_TICKS(200));
        if (received < 0) {
            ESP_LOGE(TAG, "Failed to read GPS UART");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        for (int index = 0; index < received; ++index) {
            char value = (char)read_buffer[index];
            if (value == '$') {
                collecting = true;
                sentence_length = 0;
                sentence[sentence_length++] = value;
            } else if (value == '\n') {
                if (collecting && sentence_length > 0) {
                    sentence[sentence_length] = '\0';
                    process_sentence(&parser, sentence, &valid_nmea_seen,
                                     &state_known, &last_fix_valid,
                                     &last_fix_log_ms, &dimension_known,
                                     &last_dimension, signal_slots, &stats);
                }
                collecting = false;
                sentence_length = 0;
            } else if (value != '\r' && collecting) {
                if (sentence_length < sizeof(sentence) - 1) {
                    sentence[sentence_length++] = value;
                } else {
                    collecting = false;
                    sentence_length = 0;
                    ESP_LOGW(TAG, "Discarded overlong NMEA sentence");
                }
            }
        }

        if (!valid_nmea_seen && !no_data_warning_logged &&
            uptime_ms() - started_ms >= GPS_NO_DATA_WARNING_MS) {
            ESP_LOGW(TAG,
                     "No valid NMEA received; check GPS power, D_SEL, wiring, "
                     "and baud rate");
            no_data_warning_logged = true;
        }
        log_nmea_summary(&stats, uptime_ms());
    }

    s_task = NULL;
    xSemaphoreGive(s_task_stopped);
    vTaskDelete(NULL);
}

static void cleanup_resources(void) {
    if (s_uart_installed) {
        uart_driver_delete((uart_port_t)CONFIG_GPS_UART_PORT);
        s_uart_installed = false;
    }
    if (s_task_stopped) {
        vSemaphoreDelete(s_task_stopped);
        s_task_stopped = NULL;
    }
    if (s_data_mutex) {
        vSemaphoreDelete(s_data_mutex);
        s_data_mutex = NULL;
    }
}

esp_err_t gps_init(void) {
    if (s_running || s_task || s_uart_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (CONFIG_GPS_UART_PORT < 0 || CONFIG_GPS_UART_PORT >= UART_NUM_MAX ||
        !GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_GPS_UART_TX_GPIO) ||
        !GPIO_IS_VALID_GPIO(CONFIG_GPS_UART_RX_GPIO)) {
        ESP_LOGE(TAG, "Invalid GPS UART port or GPIO configuration");
        return ESP_ERR_INVALID_ARG;
    }

    s_data_mutex = xSemaphoreCreateMutex();
    s_task_stopped = xSemaphoreCreateBinary();
    if (!s_data_mutex || !s_task_stopped) {
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }
    memset(&s_data, 0, sizeof(s_data));

    uart_config_t uart_config = {
        .baud_rate = CONFIG_GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t error = uart_driver_install((uart_port_t)CONFIG_GPS_UART_PORT,
                                          GPS_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (error != ESP_OK) {
        cleanup_resources();
        return error;
    }
    s_uart_installed = true;

    error = uart_param_config((uart_port_t)CONFIG_GPS_UART_PORT, &uart_config);
    if (error == ESP_OK) {
        error = uart_set_pin((uart_port_t)CONFIG_GPS_UART_PORT,
                             CONFIG_GPS_UART_TX_GPIO, CONFIG_GPS_UART_RX_GPIO,
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (error != ESP_OK) {
        cleanup_resources();
        return error;
    }
    uart_flush_input((uart_port_t)CONFIG_GPS_UART_PORT);

    s_running = true;
    if (xTaskCreate(gps_task, "gps_nmea", GPS_TASK_STACK_SIZE, NULL,
                    GPS_TASK_PRIORITY, &s_task) != pdPASS) {
        s_running = false;
        cleanup_resources();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "NEO-M9N on UART%d at %d baud: GPS TX -> GPIO%d, GPS RX <- "
             "GPIO%d",
             CONFIG_GPS_UART_PORT, CONFIG_GPS_UART_BAUD_RATE,
             CONFIG_GPS_UART_RX_GPIO, CONFIG_GPS_UART_TX_GPIO);
#if CONFIG_GPS_LOG_DIAGNOSTICS
    ESP_LOGI(TAG,
             "Diagnostics enabled: GSA/GSV signal data and NMEA counters "
             "every 10 seconds");
#else
    ESP_LOGW(TAG,
             "GPS diagnostics disabled; enable GPS_LOG_DIAGNOSTICS in "
             "menuconfig");
#endif
    return ESP_OK;
}

esp_err_t gps_deinit(void) {
    if (!s_running || !s_task || !s_uart_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    TaskHandle_t task = s_task;
    s_running = false;
    if (xSemaphoreTake(s_task_stopped, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "GPS task did not stop in time; stopping it now");
        vTaskDelete(task);
        s_task = NULL;
    }
    cleanup_resources();
    memset(&s_data, 0, sizeof(s_data));
    return ESP_OK;
}

esp_err_t gps_get_data(gps_data_t* data) {
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_data_mutex || !s_running) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    *data = s_data;
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}
