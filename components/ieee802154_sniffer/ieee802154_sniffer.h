#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cstddef>
#include <cstdint>

#include "esphome/core/component.h"

namespace esphome::ieee802154_sniffer {

/// Receives every 802.15.4 frame on one channel and streams it over USB Serial/JTAG as SLIP records.
///
/// Device to host records:
/// - 0x01 frame: u64 timestamp_us, u8 channel, i8 rssi, u8 lqi, PSDU without FCS.
/// - 0x02 status: u8 channel, u32 received, u32 dropped.
///
/// Host to device records:
/// - 0x10 set channel: u8 channel.
///
/// All integers are little endian.
class Ieee802154Sniffer : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_channel(uint8_t channel) { this->channel_ = channel; }

 protected:
  static void task_entry_(void *arg);
  void run_();
  void poll_commands_();
  void handle_command_(const uint8_t *data, size_t length);
  void apply_channel_(uint8_t channel);
  void send_status_();
  void write_record_(const uint8_t *data, size_t length);

  uint8_t channel_{11};
  QueueHandle_t queue_{nullptr};

  uint8_t command_buf_[16];
  size_t command_len_{0};
  bool command_escape_{false};
  bool command_overflow_{false};
};

}  // namespace esphome::ieee802154_sniffer
