#include "ieee802154_sniffer.h"

#include <driver/usb_serial_jtag.h>
#include <esp_attr.h>
#include <esp_ieee802154.h>
#include <freertos/task.h>

#include <atomic>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::ieee802154_sniffer {

static const char *const TAG = "ieee802154_sniffer";

static constexpr uint8_t SLIP_END = 0xC0;
static constexpr uint8_t SLIP_ESC = 0xDB;
static constexpr uint8_t SLIP_ESC_END = 0xDC;
static constexpr uint8_t SLIP_ESC_ESC = 0xDD;

static constexpr uint8_t RECORD_FRAME = 0x01;
static constexpr uint8_t RECORD_STATUS = 0x02;
static constexpr uint8_t COMMAND_SET_CHANNEL = 0x10;

static constexpr size_t FCS_LENGTH = 2;
static constexpr size_t MAX_PSDU_LENGTH = 127;
static constexpr size_t QUEUE_LENGTH = 64;
static constexpr uint32_t STATUS_INTERVAL_MS = 1000;

struct RxFrame {
  uint64_t timestamp;
  uint8_t channel;
  int8_t rssi;
  uint8_t lqi;
  uint8_t length;
  uint8_t data[MAX_PSDU_LENGTH];
};

// The driver callback has no context argument.
static QueueHandle_t rx_queue = nullptr;
static std::atomic<uint32_t> rx_received{0};
static std::atomic<uint32_t> rx_dropped{0};

static void IRAM_ATTR on_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *info) {
  rx_received.fetch_add(1, std::memory_order_relaxed);
  // The first byte is the PHR length. The driver replaces the FCS with RSSI and LQI.
  size_t length = frame[0];
  RxFrame rx;
  rx.timestamp = info->timestamp;
  rx.channel = info->channel;
  rx.rssi = info->rssi;
  rx.lqi = info->lqi;
  rx.length = length >= FCS_LENGTH ? length - FCS_LENGTH : 0;
  memcpy(rx.data, frame + 1, rx.length);
  esp_ieee802154_receive_handle_done(frame);

  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(rx_queue, &rx, &woken) != pdTRUE)
    rx_dropped.fetch_add(1, std::memory_order_relaxed);
  portYIELD_FROM_ISR(woken);
}

static void put_u32(uint8_t *out, uint32_t value) {
  for (size_t i = 0; i < 4; i++)
    out[i] = value >> (8 * i);
}

void Ieee802154Sniffer::setup() {
  usb_serial_jtag_driver_config_t usb_config = {
      .tx_buffer_size = 4096,
      .rx_buffer_size = 256,
  };
  esp_err_t err = usb_serial_jtag_driver_install(&usb_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB Serial/JTAG driver install failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  this->queue_ = xQueueCreate(QUEUE_LENGTH, sizeof(RxFrame));
  rx_queue = this->queue_;

  esp_ieee802154_event_cb_list_t callbacks = {};
  callbacks.rx_done_cb = on_receive_done;
  esp_ieee802154_event_callback_list_register(callbacks);

  err = esp_ieee802154_enable();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Radio enable failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  esp_ieee802154_set_promiscuous(true);
  esp_ieee802154_set_rx_when_idle(true);
  this->apply_channel_(this->channel_);

  xTaskCreate(task_entry_, "ieee802154_sniffer", 4096, this, 10, nullptr);
}

void Ieee802154Sniffer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "IEEE 802.15.4 Sniffer:\n"
                "  Channel: %u",
                this->channel_);
}

void Ieee802154Sniffer::task_entry_(void *arg) { static_cast<Ieee802154Sniffer *>(arg)->run_(); }

void Ieee802154Sniffer::run_() {
  RxFrame rx;
  uint8_t record[1 + 8 + 3 + MAX_PSDU_LENGTH];
  uint32_t last_status = millis();
  while (true) {
    if (xQueueReceive(this->queue_, &rx, pdMS_TO_TICKS(10)) == pdTRUE) {
      record[0] = RECORD_FRAME;
      for (size_t i = 0; i < 8; i++)
        record[1 + i] = rx.timestamp >> (8 * i);
      record[9] = rx.channel;
      record[10] = static_cast<uint8_t>(rx.rssi);
      record[11] = rx.lqi;
      memcpy(record + 12, rx.data, rx.length);
      this->write_record_(record, 12 + rx.length);

      if (this->frame_callback_ready_.load(std::memory_order_acquire) && this->frame_callback_)
        this->frame_callback_(rx.timestamp, rx.channel, rx.rssi, rx.lqi, rx.data, rx.length);
    }

    this->poll_commands_();

    uint32_t now = millis();
    if (now - last_status >= STATUS_INTERVAL_MS) {
      last_status = now;
      this->send_status_();
    }
  }
}

void Ieee802154Sniffer::poll_commands_() {
  uint8_t buf[64];
  int read = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
  for (int i = 0; i < read; i++) {
    uint8_t byte = buf[i];
    if (byte == SLIP_END) {
      if (!this->command_overflow_ && this->command_len_ > 0)
        this->handle_command_(this->command_buf_, this->command_len_);
      this->command_len_ = 0;
      this->command_escape_ = false;
      this->command_overflow_ = false;
      continue;
    }
    if (byte == SLIP_ESC) {
      this->command_escape_ = true;
      continue;
    }
    if (this->command_escape_) {
      byte = byte == SLIP_ESC_END ? SLIP_END : SLIP_ESC;
      this->command_escape_ = false;
    }
    if (this->command_len_ >= sizeof(this->command_buf_)) {
      this->command_overflow_ = true;
      continue;
    }
    this->command_buf_[this->command_len_++] = byte;
  }
}

void Ieee802154Sniffer::handle_command_(const uint8_t *data, size_t length) {
  if (data[0] == COMMAND_SET_CHANNEL && length == 2 && data[1] >= 11 && data[1] <= 26) {
    this->apply_channel_(data[1]);
    this->send_status_();
  }
}

void Ieee802154Sniffer::apply_channel_(uint8_t channel) {
  this->channel_ = channel;
  esp_ieee802154_set_channel(channel);
  esp_ieee802154_receive();
}

void Ieee802154Sniffer::send_status_() {
  uint8_t record[10];
  record[0] = RECORD_STATUS;
  record[1] = this->channel_;
  put_u32(record + 2, rx_received.load(std::memory_order_relaxed));
  put_u32(record + 6, rx_dropped.load(std::memory_order_relaxed));
  this->write_record_(record, sizeof(record));
}

void Ieee802154Sniffer::write_record_(const uint8_t *data, size_t length) {
  uint8_t out[2 + 2 * (1 + 8 + 3 + MAX_PSDU_LENGTH)];
  size_t pos = 0;
  out[pos++] = SLIP_END;
  for (size_t i = 0; i < length; i++) {
    if (data[i] == SLIP_END) {
      out[pos++] = SLIP_ESC;
      out[pos++] = SLIP_ESC_END;
    } else if (data[i] == SLIP_ESC) {
      out[pos++] = SLIP_ESC;
      out[pos++] = SLIP_ESC_ESC;
    } else {
      out[pos++] = data[i];
    }
  }
  out[pos++] = SLIP_END;
  usb_serial_jtag_write_bytes(out, pos, pdMS_TO_TICKS(20));
}

}  // namespace esphome::ieee802154_sniffer
