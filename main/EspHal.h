#ifndef ESP32S3_HAL_H
#define ESP32S3_HAL_H

// include RadioLib
#include <RadioLib.h>

// ESP-IDF standard headers
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Arduino-style macros (must match ESP-IDF enum values for the HAL to work)
#define LOW                         (0x0)
#define HIGH                        (0x1)
#define INPUT                       (0x01)   // GPIO_MODE_INPUT
#define OUTPUT                      (0x03)   // GPIO_MODE_INPUT_OUTPUT
#define RISING                      (0x01)   // GPIO_INTR_POSEDGE
#define FALLING                     (0x02)   // GPIO_INTR_NEGEDGE

class EspHal : public RadioLibHal {
  public:
    // default constructor - initializes the base HAL and any needed private members
    EspHal(int8_t sck, int8_t miso, int8_t mosi, spi_host_device_t host = SPI2_HOST)
      : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
        spiSCK(sck), spiMISO(miso), spiMOSI(mosi), spiHost(host), spiHandle(nullptr) {
    }

    void init() override {
      spiBegin();
    }

    void term() override {
      spiEnd();
    }

    // GPIO-related methods (pinMode, digitalWrite etc.) should check
    // RADIOLIB_NC as an alias for non-connected pins
    void pinMode(uint32_t pin, uint32_t mode) override {
      if(pin == RADIOLIB_NC) {
        return;
      }
      gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
      if(pin == RADIOLIB_NC) {
        return;
      }
      gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
      if(pin == RADIOLIB_NC) {
        return(0);
      }
      return(gpio_get_level((gpio_num_t)pin));
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
      if(interruptNum == RADIOLIB_NC) {
        return;
      }

      gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode & 0x7));

      // NOTE: gpio_install_isr_service() must be called once in your app_main()
      // before any attachInterrupt() calls, e.g.:
      //   gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
      gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
    }

    void detachInterrupt(uint32_t interruptNum) override {
      if(interruptNum == RADIOLIB_NC) {
        return;
      }

      gpio_isr_handler_remove((gpio_num_t)interruptNum);
      gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
    }

    void delay(unsigned long ms) override {
      vTaskDelay(ms / portTICK_PERIOD_MS);
    }

    void delayMicroseconds(unsigned long us) override {
      uint64_t start = (uint64_t)esp_timer_get_time();
      uint64_t end = start + us;
      if(us) {
        while((uint64_t)esp_timer_get_time() < end) {
          __asm__ volatile ("nop");
        }
      }
    }

    unsigned long millis() override {
      return((unsigned long)(esp_timer_get_time() / 1000ULL));
    }

    unsigned long micros() override {
      return((unsigned long)(esp_timer_get_time()));
    }

    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
      if(pin == RADIOLIB_NC) {
        return(0);
      }

      this->pinMode(pin, INPUT);
      uint32_t start = this->micros();

      // wait for any previous pulse to end
      while(this->digitalRead(pin) == state) {
        if((this->micros() - start) > timeout) {
          return(0);
        }
      }

      // wait for the pulse to start
      while(this->digitalRead(pin) != state) {
        if((this->micros() - start) > timeout) {
          return(0);
        }
      }

      uint32_t pulseStart = this->micros();

      // wait for the pulse to end
      while(this->digitalRead(pin) == state) {
        if((this->micros() - pulseStart) > timeout) {
          return(0);
        }
      }

      return(this->micros() - pulseStart);
    }

    void spiBegin() {
      if(spiSCK == RADIOLIB_NC || spiMOSI == RADIOLIB_NC || spiMISO == RADIOLIB_NC) {
        return;
      }

      spi_bus_config_t buscfg = {
        .mosi_io_num = spiMOSI,
        .miso_io_num = spiMISO,
        .sclk_io_num = spiSCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = 4096,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0
      };

      esp_err_t ret = spi_bus_initialize(spiHost, &buscfg, SPI_DMA_DISABLED);
      if(ret != ESP_OK) {
        // handle error as needed (bus may already be initialized)
        return;
      }

      spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 128,      // 50% duty cycle
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 2000000, // 2 MHz default
        .input_delay_ns = 0,
        .sample_point = SPI_SAMPLING_POINT_PHASE_0,
        .spics_io_num = -1,        // CS is handled by RadioLib via digitalWrite()
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
      };

      spi_bus_add_device(spiHost, &devcfg, &spiHandle);
    }

    void spiBeginTransaction() {
      // not needed - configuration is done in spiBegin
    }

    uint8_t spiTransferByte(uint8_t b) {
      if(!spiHandle) {
        return(0);
      }
      uint8_t rx = 0;
      spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8,
        .rxlength = 8,
        .override_freq_hz = 0,
        .user = nullptr,
        .tx_buffer = &b,
        .rx_buffer = &rx
      };
      spi_device_transmit(spiHandle, &t);
      return(rx);
    }

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
      if(!spiHandle || len == 0) {
        return;
      }
      spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = len * 8,
        .rxlength = len * 8,
        .override_freq_hz = 0,
        .user = nullptr,
        .tx_buffer = out,
        .rx_buffer = in
      };
      spi_device_transmit(spiHandle, &t);
    }

    void spiEndTransaction() {
      // nothing needs to be done here
    }

    void spiEnd() {
      if(spiHandle) {
        spi_bus_remove_device(spiHandle);
        spiHandle = nullptr;
      }
      spi_bus_free(spiHost);
    }

  private:
    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;
    spi_host_device_t spiHost;
    spi_device_handle_t spiHandle;
};

#endif