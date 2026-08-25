#include "e220m30s.h"

#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>

#include "esp_log.h"

namespace {

static const char* TAG = "Narrowband";

// radio role is selected in menuconfig (main/Kconfig.projbuild, NB_RADIO_MODE)
#if !defined(CONFIG_NB_RADIO_MODE_ROCKET) && \
    !defined(CONFIG_NB_RADIO_MODE_GROUND)
#error "NB_RADIO_MODE is not set. Select rocket or ground mode in menuconfig."
#endif

template <typename RadioType>
class NarrowbandRadio {
   private:
    // pin definitions
    static constexpr int SCLK_PIN = 12;
    static constexpr int MISO_PIN = 13;
    static constexpr int MOSI_PIN = 11;
    static constexpr int NSS_PIN = 10;
    static constexpr int DIO1_PIN = 45;
    static constexpr int NRST_PIN = RADIOLIB_NC;
    static constexpr int BUSY_PIN = 2;
    static constexpr int RXEN_PIN = RADIOLIB_NC;

    EspHal hal;
    Module module;
    LLCC68 radio;

    TaskHandle_t rxtxTaskHandle;
    // index of the task notification used by the radio ISR callback
    static constexpr UBaseType_t rxtxTaskNotifyIndex = 0;

    void handle_receive(QueueHandle_t rxQueue);
    static void IRAM_ATTR radio_isr(void);
    static void rxtx_task_trampoline(void* param);

   protected:
    // radio settings; must match what beginFSK is configured with in init()
    static constexpr float frequency_mhz = 434.0f;
    static constexpr uint32_t bitrate_bps = 2400;  // 2.4 kbps
    static constexpr float frequency_deviation_khz = 2.4f;
    static constexpr float rx_bandwidth_khz = 11.7f;
    static constexpr int8_t tx_power_dbm = 22;
    static constexpr uint16_t preamble_length_bits = 32;

    // max payload of a single FSK packet is 255 bytes; one byte is used as the
    // per-message length prefix, so a single message is limited to 254 bytes
    static constexpr size_t max_payload_size =
        RADIOLIB_SX126X_MAX_PACKET_LENGTH;
    static constexpr size_t max_message_length = max_payload_size - 1;

    static constexpr uint32_t tx_timeout_ms =
        500;  // base timeout, extended by airtime in transmit_data()

    // a full-size packet takes ~880 ms on air at 2.4 kbps; the listen window
    // must be at least that long, otherwise the radio's hardware timeout would
    // abort a packet that is still being received when the window ends
    static constexpr uint16_t listen_window_ms = 1200;

    QueueHandle_t commandQueue;
    QueueHandle_t sensorDataQueue;

    size_t pack_messages(std::span<uint8_t> buffer, QueueHandle_t queue);
    void unpack_messages(std::span<const uint8_t> buffer, QueueHandle_t queue);
    void transmit_data(std::span<uint8_t> buffer);
    bool listen(uint16_t timeout_ms, bool return_on_receive,
                QueueHandle_t rxQueue);

   public:
    NarrowbandRadio();
    void init(QueueHandle_t commandQueue, QueueHandle_t sensorDataQueue);
};

#ifdef CONFIG_NB_RADIO_MODE_ROCKET

class RocketRadio : public NarrowbandRadio<RocketRadio> {
   public:
    void rxtx_task() {
        ESP_LOGI(TAG, "Rocket rxtx task started");
        std::array<uint8_t, max_payload_size> tx_buffer;

        while (true) {
            // transmit all queued sensor data (whole messages, batched)
            size_t bytes_copied = pack_messages(tx_buffer, sensorDataQueue);
            transmit_data(std::span<uint8_t>(tx_buffer.data(), bytes_copied));

            // listen for commands until the next transmission interval
            listen(listen_window_ms, false, commandQueue);
        }
    }
};

RocketRadio nb_radio;

#endif  // CONFIG_NB_RADIO_MODE_ROCKET

#ifdef CONFIG_NB_RADIO_MODE_GROUND

class GroundRadio : public NarrowbandRadio<GroundRadio> {
   public:
    void rxtx_task() {
        ESP_LOGI(TAG, "Ground rxtx task started");
        std::array<uint8_t, max_payload_size> tx_buffer;

        while (true) {
            // wait for a rocket transmission; received messages land in
            // sensorDataQueue
            bool packet_received =
                listen(listen_window_ms, true, sensorDataQueue);
            if (!packet_received) {
                ESP_LOGD(TAG, "No packet received, rocket not available");
                continue;
            }

            // respond with all queued commands
            size_t bytes_copied = pack_messages(tx_buffer, commandQueue);
            transmit_data(std::span<uint8_t>(tx_buffer.data(), bytes_copied));
        }
    }
};

GroundRadio nb_radio;

#endif  // CONFIG_NB_RADIO_MODE_GROUND

// CLASS IMPLEMENTATION

template <typename RadioType>
NarrowbandRadio<RadioType>::NarrowbandRadio()
    : hal(SCLK_PIN, MISO_PIN, MOSI_PIN),
      module(&hal, NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN),
      radio(&module),
      rxtxTaskHandle(nullptr),
      commandQueue(nullptr),
      sensorDataQueue(nullptr) {}

template <typename RadioType>
void NarrowbandRadio<RadioType>::init(QueueHandle_t commandQueue,
                                      QueueHandle_t sensorDataQueue) {
    ESP_LOGI(TAG, "[LLCC68] Initializing narrowband radio...");

    this->commandQueue = commandQueue;
    this->sensorDataQueue = sensorDataQueue;

    // 434 MHz, 2.4 kbps FSK, 2.4 kHz deviation, 11.7 kHz receive bandwidth,
    // 22 dBm, 32-bit preamble; beginFSK also configures DIO2 to control the
    // TX/RX RF switch
    int state = radio.beginFSK(frequency_mhz, bitrate_bps / 1000.0f,
                               frequency_deviation_khz, rx_bandwidth_khz,
                               tx_power_dbm, preamble_length_bits, 0, false);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "beginFSK failed, code %d (fatal)", state);
        abort();
    }

    // highest-power PA configuration for the LLCC68, see datasheet
    state = radio.setPaConfig(0x04, 0x00, 0x07, 0x01);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "PA config failed, code %d (fatal)", state);
        abort();
    }

    radio.setDio2AsRfSwitch(true);

    // register the DIO1 callback; it must be IRAM-safe and must not touch the
    // radio. RX and TX events share DIO1, so the same handler is used for both
    // (RadioLib attaches the same pin for both callbacks, the second one wins)
    radio.setPacketReceivedAction(radio_isr);
    radio.setPacketSentAction(radio_isr);

    BaseType_t task_created = xTaskCreate(rxtx_task_trampoline, "rxtx", 4096,
                                          this, 1, &rxtxTaskHandle);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create rxtx task (fatal)");
        abort();
    }

    ESP_LOGI(TAG, "[LLCC68] Initialized successfully");
}

// RadioLib calls this from the GPIO ISR; it only wakes the rxtx task. A
// notification alone is not proof of a radio event: RX and TX share one
// notification index, and notifications may be stale (e.g. a TX_DONE arriving
// after its timeout). Callers must confirm the event by reading the latched
// radio IRQ flags.
template <typename RadioType>
void IRAM_ATTR NarrowbandRadio<RadioType>::radio_isr(void) {
    // a DIO1 edge between callback registration and task creation is possible
    // (the radio is in standby then, so it is not a real RX/TX event); ignore
    // it instead of asserting from ISR context
    if (nb_radio.rxtxTaskHandle == nullptr) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(nb_radio.rxtxTaskHandle,
                                  nb_radio.rxtxTaskNotifyIndex,
                                  &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Packs whole messages from the queue into the buffer as [length][payload]
// frames. Messages are never split across packets: a message that does not
// fit into the remaining space is left in the queue for the next packet.
// Returns the number of bytes packed. Only the rxtx task consumes the queue
// (the producer only sends), so xQueuePeek + xQueueReceive is safe.
template <typename RadioType>
size_t NarrowbandRadio<RadioType>::pack_messages(std::span<uint8_t> buffer,
                                                 QueueHandle_t queue) {
    size_t offset = 0;

    while (offset < buffer.size()) {
        message_t msg{};
        // peek without removing, so messages that do not fit are not lost
        if (xQueuePeek(queue, &msg, (TickType_t)0) != pdTRUE) {
            break;  // queue empty
        }

        if (msg.length == 0) {
            // drop zero-length messages
            xQueueReceive(queue, &msg, (TickType_t)0);
            free(msg.data);
            continue;
        }

        size_t length = std::min(msg.length, max_message_length);
        if (length != msg.length) {
            ESP_LOGW(TAG, "Message too long (%u bytes), truncating to %u bytes",
                     (unsigned)msg.length, (unsigned)length);
        }

        // 1 byte length prefix + payload must fit into the remaining space
        if (offset + 1 + length > buffer.size()) {
            break;  // leave the message in the queue for the next packet
        }

        xQueueReceive(queue, &msg, (TickType_t)0);
        buffer[offset++] = (uint8_t)length;
        memcpy(buffer.data() + offset, msg.data, length);
        offset += length;
        free(msg.data);
    }

    return offset;
}

// Parses [length][payload] frames from a received packet and enqueues each
// message. Malformed frames (length exceeding the packet) drop the rest of
// the packet, so a damaged stream cannot desynchronize subsequent packets.
template <typename RadioType>
void NarrowbandRadio<RadioType>::unpack_messages(
    std::span<const uint8_t> buffer, QueueHandle_t queue) {
    size_t offset = 0;

    while (offset < buffer.size()) {
        uint8_t length = buffer[offset++];
        if (length == 0) {
            continue;  // zero-length frame; reserved for future use (e.g.
                       // acknowledgements)
        }
        if (offset + length > buffer.size()) {
            ESP_LOGW(TAG,
                     "Truncated message (%u bytes claimed, %u left), dropping "
                     "rest of packet",
                     (unsigned)length, (unsigned)(buffer.size() - offset));
            break;
        }

        uint8_t* data = (uint8_t*)malloc(length);
        if (data == nullptr) {
            ESP_LOGE(TAG, "Out of memory for %u-byte message",
                     (unsigned)length);
            break;
        }

        memcpy(data, buffer.data() + offset, length);
        offset += length;

        message_t msg = {.data = data, .length = length};
        if (xQueueSend(queue, &msg, (TickType_t)0) != pdTRUE) {
            ESP_LOGW(TAG, "Receive queue full, dropping message");
            free(data);
        }
    }
}

template <typename RadioType>
void NarrowbandRadio<RadioType>::transmit_data(std::span<uint8_t> buffer) {
    if (buffer.empty()) {
        return;  // nothing to send
    }

    // discard stale notifications (e.g. a late RX event from a previous listen
    // window); the radio is in standby here, so no real event can be lost
    while (ulTaskNotifyTakeIndexed(rxtxTaskNotifyIndex, pdTRUE, 0) != 0) {
    }

    int state = radio.startTransmit(buffer.data(), buffer.size());
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "startTransmit failed, code %d", state);
        return;
    }

    // base timeout plus on-air time of the payload (~10 bytes of
    // preamble/sync/CRC overhead)
    uint32_t airtime_ms = ((buffer.size() + 10) * 8 * 1000) / bitrate_bps;
    uint32_t timeout_ms = tx_timeout_ms + airtime_ms;

    ulTaskNotifyTakeIndexed(rxtxTaskNotifyIndex, pdTRUE,
                            pdMS_TO_TICKS(timeout_ms));

    // the latched TX_DONE flag, not the notification count, proves completion:
    // the software timeout may race with TX_DONE arriving in the last tick
    if (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_TX_DONE) {
        ESP_LOGI(TAG, "Transmitted %u bytes", (unsigned)buffer.size());
    } else {
        ESP_LOGW(TAG, "Transmission timed out after %u ms",
                 (unsigned)timeout_ms);
    }

    // always return to standby and clear IRQ flags, also on timeout
    state = radio.finishTransmit();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "finishTransmit failed, code %d", state);
    }
}

// Listens for packets for up to timeout_ms. The receiver is armed for a
// single packet per iteration with a hardware timeout (the SX126x then falls
// back to standby after a packet or timeout), so no partial or stale FIFO
// data can ever be read. If return_on_receive is set, stops after the first
// packet, otherwise keeps listening for the whole window.
template <typename RadioType>
bool NarrowbandRadio<RadioType>::listen(uint16_t timeout_ms,
                                        bool return_on_receive,
                                        QueueHandle_t rxQueue) {
    bool packet_received = false;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        if (packet_received && return_on_receive) {
            break;
        }

        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout_ticks) {
            break;  // listen window over
        }

        // single-shot receive, hardware timeout in units of 15.625 us
        uint32_t rx_timeout_raw = pdTICKS_TO_MS(timeout_ticks - elapsed) * 64;
        int state =
            radio.startReceive(rx_timeout_raw, RADIOLIB_IRQ_RX_DEFAULT_FLAGS,
                               RADIOLIB_IRQ_RX_DEFAULT_MASK);
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGW(TAG, "Failed to start receiver, code %d", state);
            break;
        }

        // wait for the packet to complete, the hardware timeout, or a stale
        // notification
        uint32_t count = ulTaskNotifyTakeIndexed(rxtxTaskNotifyIndex, pdTRUE,
                                                 timeout_ticks - elapsed);

        // the latched IRQ flags are the source of truth: a notification alone
        // may be stale (e.g. TX_DONE from an earlier transmission), and the
        // software timeout can race with a packet completing in the last tick
        if (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE) {
            handle_receive(rxQueue);
            packet_received = true;
        }

        if (count == 0) {
            break;  // listen window over
        }
    }

    // return to standby so the next operation starts from a known state
    int state = radio.finishReceive();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to finish receive, code %d", state);
    }
    return packet_received;
}

template <typename RadioType>
void NarrowbandRadio<RadioType>::handle_receive(QueueHandle_t rxQueue) {
    size_t len = radio.getPacketLength();
    if (len == 0) {
        ESP_LOGD(TAG, "RX_DONE without payload, ignoring");
        return;
    }

    uint8_t* buf = (uint8_t*)malloc(len);
    if (buf == nullptr) {
        ESP_LOGE(TAG, "Out of memory for %u-byte receive buffer",
                 (unsigned)len);
        return;
    }

    int state = radio.readData(buf, len);
    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        ESP_LOGW(TAG, "Dropping %u-byte packet with CRC mismatch",
                 (unsigned)len);
    } else if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to read received packet, code %d", state);
    } else {
        ESP_LOGI(TAG, "Received %u-byte packet", (unsigned)len);
        unpack_messages(std::span<const uint8_t>(buf, len), rxQueue);
    }

    free(buf);
}

template <typename RadioType>
void NarrowbandRadio<RadioType>::rxtx_task_trampoline(void* param) {
    static_cast<RadioType*>(param)->rxtx_task();
}
}  // namespace

// C COMPATIBILITY WRAPPERS

extern "C" {
void init_narrowband(QueueHandle_t commandQueue,
                     QueueHandle_t sensorDataQueue) {
    nb_radio.init(commandQueue, sensorDataQueue);
}
}
