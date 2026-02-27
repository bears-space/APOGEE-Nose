#include "narrowband.h"
#include <RadioLib.h>
#include <queue>
#include <vector>
#include <cstdint>
#include "EspHal.h"
#include "esp_log.h"

// radio comms interval
constexpr int RADIO_INTERVAL_MS = 500;

// HAL and radio module pin definitions
constexpr int SCLK_PIN = 14;
constexpr int MISO_PIN = 12;
constexpr int MOSI_PIN = 13;
constexpr int NSS_PIN = 19;
constexpr int DIO1_PIN = 26;
constexpr int NRST_PIN = 18;
constexpr int BUSY_PIN = 21;
constexpr int RXEN_PIN = 16;


static EspHal* s_hal = nullptr;
static LLCC68* s_radio = nullptr;

static const char* TAG = "Narrowband";

void init_narrowband() {

    ESP_LOGI(TAG, "[LLCC68] Initializing narrowband radio...");
    
    // create a new instance of the HAL class
    s_hal = new EspHal(SCLK_PIN, MISO_PIN, MOSI_PIN);

    // now we can create the radio module
    s_radio = new LLCC68(new Module(s_hal, NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN));

    // freq 434 Mhz, bitrate 2.4 kHz, frequency deviation 2.4 kHz, receiver bandwidth DSB 11.7 kHz, power 22 dBm, preamble length 32 bit, TCXO voltage 0 V, useRegulatorLDO false
    int state = s_radio->beginFSK(434, 2.4, 2.4, 11.7, 22, 32, 0, false);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "failed, code %d\n", state);
        return;
    }

    ESP_LOGI(TAG, "success!\n");

  // RXEN pin: 16
  // TXEN pin controlled via dio2
  s_radio->setRfSwitchPins(RXEN_PIN, RADIOLIB_NC);
  s_radio->setDio2AsRfSwitch(true);
  

  state = s_radio->setPaConfig(0x04, 0x00, 0x07, 0x01);
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "failed pa config, code %d\n", state);
    return;
  }
  ESP_LOGI(TAG, "[LLCC68] PA config configured!\n");
}


void parse_command() {

}

// interrupt function
// needs IRAM_ATTR to be used as interrupt func, which makes the func be stored in IRAM instead of flash, to ensure it can be executed when flash is not accessible (e.g. during sleep)
void IRAM_ATTR handle_receive(void) {
    if (s_hal == nullptr || s_radio == nullptr) {
        return;
    }

    size_t len = s_radio->getPacketLength();
    std::vector<uint8_t> buf(len);
    int state = s_radio->readData(buf.data(), buf.size());

    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Received packet: %s\n", buf.data());
        // TODO: parse_command(), put received packet into cmd queue

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        ESP_LOGI(TAG, "Received packet with CRC mismatch!\n");
    } else {
        ESP_LOGI(TAG, "Failed to read received packet, code %d\n", state);
    }
}

void narrowband_thread() {

    if (s_hal == nullptr || s_radio == nullptr) {
        ESP_LOGI(TAG, "HAL or radio not initialized!\n");
        return;
    }

    ESP_LOGI(TAG, "[LLCC68] Started narrowband thread!\n");

    int state = RADIOLIB_ERR_NONE;
    while (true) {

        ESP_LOGI(TAG, "[LLCC68] Sending packet...");

        state = s_radio->transmit("Hello world!");
        if(state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!\n");
        } else {
            ESP_LOGI(TAG, "failed, code %d\n", state);
        }
        ESP_LOGI(TAG, "Datarate measured: %f\n", s_radio->getDataRate());


        state = s_radio->startReceive();
        if(state == RADIOLIB_ERR_NONE) {
            // the module is now in receive mode, waiting for a packet
            ESP_LOGI(TAG, "Waiting for a packet...\n");
        } else {
            ESP_LOGI(TAG, "failed to start receiver, code %d\n", state);
        }

        // wait 0.5 s
        s_hal->delay(RADIO_INTERVAL_MS);
        
        // if no packet was received, we just start sending again
    }
}

void destroy_narrowband() {
    // free hal
    delete s_hal;
    s_hal = nullptr;

    // free radio
    delete s_radio;
    s_radio = nullptr;
}
