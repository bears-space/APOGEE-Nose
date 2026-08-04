# u-blox NEO-M9N GPS

The main firmware reads the NEO-M9N over a dedicated hardware UART and parses
the default NMEA GGA and RMC messages in a background FreeRTOS task. It exposes
position, altitude, speed, course, fix quality, satellite count, dilution of
precision, per-constellation C/N0 signal diagnostics, and UTC date/time as a
thread-safe snapshot.

## Wiring

UART signals are crossed between the GPS and ESP:

| NEO-M9N signal | ESP32-S3 signal | GPIO |
| --- | --- | ---: |
| TXD (GPS output) | UART RX | 47 |
| RXD (GPS input) | UART TX | 48 |
| GND | GND | - |

Leave the NEO-M9N `D_SEL` pin open or high to enable UART. The module's digital
I/O voltage follows its VCC supply, so the GPS and ESP must use compatible logic
levels and share ground.

## UART defaults

The firmware matches the NEO-M9N-00B defaults from the data sheet:

- 38400 baud
- 8 data bits, no parity, 1 stop bit
- no hardware flow control
- NMEA output enabled

These settings can be changed under **APOGEE GPS (u-blox NEO-M9N)** in
`idf.py menuconfig`. If the receiver was previously reconfigured and its saved
baud rate differs from 38400, set `GPS_UART_BAUD_RATE` to match it.

The project defaults to the ESP32-S3 target because GPIO47 and GPIO48 are S3
pins. If an existing local `sdkconfig` was created for another chip, run
`idf.py set-target esp32s3` once before building.

## Reading the latest fix

```c
#include "gps.h"

gps_data_t position;
if (gps_get_data(&position) == ESP_OK && position.fix_valid) {
    ESP_LOGI("location", "%.7f, %.7f", position.latitude_deg,
             position.longitude_deg);
}
```

`gps_get_data()` never waits for the receiver to acquire a fix. It copies the
latest data under a mutex. When `fix_valid` is false, coordinates remain the
last known values; use `last_fix_ms` to enforce the maximum acceptable age for
your application.

The driver validates every NMEA checksum and discards malformed or overlong
sentences. It logs a wiring/baud warning if no supported, valid NMEA sentence
arrives within five seconds of startup.

## Signal diagnostics

The default GSA and GSV messages provide the fix dimension, dilution of
precision, satellites in view, and carrier-to-noise density (C/N0). Diagnostics
are logged once per GNSS/signal when first received and then every ten seconds:

```text
I (...) gps: Navigation mode: no fix, DOP unavailable
I (...) gps: Signal GPS/SBAS (signal 1): view=9 tracked=6 avg=31.8 max=42 dB-Hz (usable)
I (...) gps: Signal GLONASS (signal 1): 5 satellites in view, none with C/N0
I (...) gps: NMEA/10s: GGA=10 RMC=10 GSA=10 GSV=18 other=20 bad_checksum=0 malformed=0
```

The quality labels are intentionally approximate: below 20 dB-Hz is reported
as very weak, 20–29 as weak, 30–37 as usable, and 38 or higher as strong. Use
the numeric values when diagnosing antenna placement. `GPS_LOG_DIAGNOSTICS`
can disable these messages independently from periodic position logs. If both
GSA and GSV remain at zero, the driver emits a warning because the receiver's
UART output must include those sentence types before DOP or C/N0 can be shown.

Hardware reference: [u-blox NEO-M9N-00B data sheet](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf).
