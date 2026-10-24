#pragma once

#include <atomic>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/ieee802154_sniffer/ieee802154_sniffer.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome::sureflap {

/// Watches the SureFlap radio link through the 802.15.4 sniffer. It does not own
/// the radio. Frames arrive on the sniffer task, so the callback only updates
/// counters and loop() publishes the entities.
class SureFlap : public Component {
 public:
  void set_sniffer(ieee802154_sniffer::Ieee802154Sniffer *sniffer) { this->sniffer_ = sniffer; }
  void set_heartbeat_timeout(uint32_t timeout_ms) { this->heartbeat_timeout_ms_ = timeout_ms; }
  void set_frames_sensor(sensor::Sensor *sensor) { this->frames_sensor_ = sensor; }
  void set_movements_sensor(sensor::Sensor *sensor) { this->movements_sensor_ = sensor; }
  void set_rssi_sensor(sensor::Sensor *sensor) { this->rssi_sensor_ = sensor; }
  void set_flap_online(binary_sensor::BinarySensor *binary_sensor) { this->flap_online_ = binary_sensor; }
  void set_hub_online(binary_sensor::BinarySensor *binary_sensor) { this->hub_online_ = binary_sensor; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void on_frame_(int8_t rssi, const uint8_t *psdu, size_t length);
  void publish_online_(binary_sensor::BinarySensor *entity, std::atomic<uint32_t> &last_ms, std::atomic<bool> &seen,
                       bool &published, bool &valid, uint32_t now);

  ieee802154_sniffer::Ieee802154Sniffer *sniffer_{nullptr};
  sensor::Sensor *frames_sensor_{nullptr};
  sensor::Sensor *movements_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};
  binary_sensor::BinarySensor *flap_online_{nullptr};
  binary_sensor::BinarySensor *hub_online_{nullptr};

  uint32_t heartbeat_timeout_ms_{30000};

  // Written on the sniffer task, read on the main loop.
  std::atomic<uint32_t> frame_count_{0};
  std::atomic<uint32_t> movement_count_{0};
  std::atomic<int32_t> last_flap_rssi_{0};
  std::atomic<bool> rssi_seen_{false};
  std::atomic<uint32_t> last_flap_ms_{0};
  std::atomic<bool> flap_seen_{false};
  std::atomic<uint32_t> last_hub_ms_{0};
  std::atomic<bool> hub_seen_{false};

  uint32_t published_frame_count_{0};
  uint32_t published_movement_count_{0};
  uint32_t last_publish_ms_{0};
  bool published_flap_online_{false};
  bool published_flap_valid_{false};
  bool published_hub_online_{false};
  bool published_hub_valid_{false};
};

}  // namespace esphome::sureflap
