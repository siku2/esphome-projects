#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "esphome/core/component.h"

namespace esphome::ieee802154_sniffer {

/// Called on the sniffer task for every received frame. The PSDU is valid only
/// for the duration of the call. Runs off the main loop, so implementations must
/// not touch ESPHome entities directly.
using FrameCallback = std::function<void(uint64_t timestamp, uint8_t channel, int8_t rssi, uint8_t lqi,
                                         const uint8_t *psdu, size_t length)>;

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

  /// Register a frame sink. Call during setup. Only one sink is supported.
  void set_frame_callback(FrameCallback callback) {
    this->frame_callback_ = std::move(callback);
    this->frame_callback_ready_.store(true, std::memory_order_release);
  }

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

  FrameCallback frame_callback_{};
  std::atomic<bool> frame_callback_ready_{false};

  uint8_t command_buf_[16];
  size_t command_len_{0};
  bool command_escape_{false};
  bool command_overflow_{false};
};

}  // namespace esphome::ieee802154_sniffer
