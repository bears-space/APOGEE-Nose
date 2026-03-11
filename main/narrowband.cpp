#include "narrowband.h"
#include <RadioLib.h>
#include <vector>
#include <cstdint>
#include "EspHal.h"
#include "esp_log.h"
#include <cstring>
#include <sys/types.h>
#include <atomic>  // for ISR flag
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace {

    // flag set in ISR when packet arrives
    // atomic operation to avoid data race between interrupt and main thread
    static std::atomic<bool> s_received_packet{false};

    static const char* TAG = "Narrowband";

    // CLASS DEFINITION
    class NarrowbandRadio {
    private:

        // pin definitions
        static constexpr int SCLK_PIN = 14;
        static constexpr int MISO_PIN = 12;
        static constexpr int MOSI_PIN = 13;
        static constexpr int NSS_PIN  = 19;
        static constexpr int DIO1_PIN = 26;
        static constexpr int NRST_PIN = 18;
        static constexpr int BUSY_PIN = 21;
        static constexpr int RXEN_PIN = 16;

        int rxtx_interval_ms = 500;

        EspHal hal;
        Module module;
        LLCC68 radio;

        QueueHandle_t* commandQueue;
        QueueHandle_t* sensorDataQueue;
        
        void handleReceive();
        static void IRAM_ATTR receive_isr(void);
        static void rxtx_task_trampoline(void* param);
        void rxtx_task();
        
    public:
        NarrowbandRadio();
        void init(QueueHandle_t* commandQueue, QueueHandle_t* sensorDataQueue);
    };


    // CLASS IMPLEMENTATION

    NarrowbandRadio::NarrowbandRadio()
        : hal(SCLK_PIN, MISO_PIN, MOSI_PIN),
        module(&hal, NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN),
        radio(&module),
        commandQueue(nullptr),
        sensorDataQueue(nullptr) {}

    void NarrowbandRadio::init(QueueHandle_t* commandQueue, QueueHandle_t* sensorDataQueue) {
        ESP_LOGI(TAG, "[LLCC68] Initializing narrowband radio...");
        
        // TODO: remove magic numbers, use config values instead
        // freq 434 Mhz, bitrate 2.4 kHz, frequency deviation 2.4 kHz, receiver bandwidth DSB 11.7 kHz, power 22 dBm, preamble length 32 bit, TCXO voltage 0 V, useRegulatorLDO false
        int state = radio.beginFSK(434, 2.4, 2.4, 11.7, 22, 32, 0, false);
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "beginFSK failed, code %d (fatal)\n", state);
            abort(); // fatal error, cannot continue without radio
        }

        // RXEN pin: 16
        // TXEN pin controlled via dio2
        radio.setRfSwitchPins(RXEN_PIN, RADIOLIB_NC);
        radio.setDio2AsRfSwitch(true);

        ESP_LOGI(TAG, "success!\n");

        // for more details, see LLCC68 datasheet, this is the highest power setting, with 22 dBm set in beginFSK
        state = radio.setPaConfig(0x04, 0x00, 0x07, 0x01);
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "PA config failed, code %d (fatal)\n", state);
            abort();
        }
        ESP_LOGI(TAG, "[LLCC68] PA config configured!\n");

        // configure callback for received packet; must be a free/IRAM-safe function
        radio.setPacketReceivedAction(receive_isr);

        // create the rx/tx task
        // xTaskCreate(rxtx_task_trampoline, "narrowband_rxtx", 4096, this, 1, NULL);

    }

    // ISR callback stored in IRAM; just sets the atomic flag
    void IRAM_ATTR NarrowbandRadio::receive_isr(void) {
        s_received_packet.store(true, std::memory_order_relaxed);
    }

    void NarrowbandRadio::handleReceive() {

        // check and clear ISR flag
        if (!s_received_packet.load(std::memory_order_relaxed)) {
            return;
        }
        s_received_packet.store(false, std::memory_order_relaxed);

        size_t len = radio.getPacketLength();
        uint8_t* buf = (uint8_t*)malloc(len);

        int state = radio.readData(buf, len);

        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG, "Received packet: %s\n", buf);
            // maybe parse command here, currently we just enqueue the raw packet data for processing in the main thread
            // enqueueCommand(buf);
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            ESP_LOGI(TAG, "Received packet with CRC mismatch!\n");
        } else {
            ESP_LOGI(TAG, "Failed to read received packet, code %d\n", state);
        }

        free(buf);
    }

    void NarrowbandRadio::rxtx_task_trampoline(void* param) {
        static_cast<NarrowbandRadio*>(param)->rxtx_task();
    }

    void NarrowbandRadio::rxtx_task() {
        ESP_LOGI(TAG, "[LLCC68] Started rxtx task!\n");

        int state = RADIOLIB_ERR_NONE;
        while (true) {
            ESP_LOGI(TAG, "[LLCC68] Sending packet...");

            // TODO: replace with sensor data from sensor queue
            state = radio.transmit("Hello world!");
            if (state == RADIOLIB_ERR_NONE) {
                // the packet was successfully transmitted
                ESP_LOGI(TAG, "success!\n");
            } else {
                ESP_LOGI(TAG, "failed, code %d\n", state);
            }

            state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE) {
                // the module is now in receive mode, waiting for a packet
                ESP_LOGI(TAG, "Waiting for a packet...\n");
            } else {
                ESP_LOGI(TAG, "failed to start receiver, code %d\n", state);
            }

            // wait 0.5 s
            hal.delay(rxtx_interval_ms);
            
            // if no packet was received, we just start sending again
            handleReceive();
        }
    }

    // static instance of the narrowband class
    NarrowbandRadio nb_radio;
}

// C COMPATIBILITY WRAPPERS

extern "C" {
    void init_narrowband(QueueHandle_t* commandQueue, QueueHandle_t* sensorDataQueue) {
        nb_radio.init(commandQueue, sensorDataQueue);
    }
}
