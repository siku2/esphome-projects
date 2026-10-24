#include "sureflap.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "protocol.h"

namespace esphome::sureflap {

static const char *const TAG = "sureflap";

static constexpr uint32_t PUBLISH_INTERVAL_MS = 1000;

void SureFlap::setup() {
  if (this->sniffer_ == nullptr) {
    ESP_LOGE(TAG, "No sniffer set");
    this->mark_failed();
    return;
  }
  this->sniffer_->set_frame_callback([this](uint64_t, uint8_t, int8_t rssi, uint8_t, const uint8_t *psdu,
                                            size_t length) { this->on_frame_(rssi, psdu, length); });
}

void SureFlap::on_frame_(int8_t rssi, const uint8_t *psdu, size_t length) {
  sureflap_protocol::Frame frame = sureflap_protocol::parse_frame(psdu, length);
  if (!frame.valid)
    return;
  this->frame_count_.fetch_add(1, std::memory_order_relaxed);
  uint32_t now = millis();

  if (frame.src_is_hub) {
    this->last_hub_ms_.store(now, std::memory_order_relaxed);
    this->hub_seen_.store(true, std::memory_order_relaxed);
    return;
  }

  // From here on the frame is from the flap.
  this->last_flap_rssi_.store(rssi, std::memory_order_relaxed);
  this->rssi_seen_.store(true, std::memory_order_relaxed);
  if (frame.type == sureflap_protocol::MSG_HEARTBEAT) {
    this->last_flap_ms_.store(now, std::memory_order_relaxed);
    this->flap_seen_.store(true, std::memory_order_relaxed);
  }
  if (sureflap_protocol::is_movement(frame))
    this->movement_count_.fetch_add(1, std::memory_order_relaxed);
}

void SureFlap::publish_online_(binary_sensor::BinarySensor *entity, std::atomic<uint32_t> &last_ms,
                               std::atomic<bool> &seen, bool &published, bool &valid, uint32_t now) {
  if (entity == nullptr)
    return;
  bool online = seen.load(std::memory_order_relaxed) &&
                (now - last_ms.load(std::memory_order_relaxed)) < this->heartbeat_timeout_ms_;
  if (!valid || online != published) {
    valid = true;
    published = online;
    entity->publish_state(online);
  }
}

void SureFlap::loop() {
  uint32_t now = millis();
  if (now - this->last_publish_ms_ < PUBLISH_INTERVAL_MS)
    return;
  this->last_publish_ms_ = now;

  if (this->frames_sensor_ != nullptr) {
    uint32_t count = this->frame_count_.load(std::memory_order_relaxed);
    if (count != this->published_frame_count_) {
      this->published_frame_count_ = count;
      this->frames_sensor_->publish_state(count);
    }
  }

  if (this->movements_sensor_ != nullptr) {
    uint32_t count = this->movement_count_.load(std::memory_order_relaxed);
    if (count != this->published_movement_count_) {
      this->published_movement_count_ = count;
      this->movements_sensor_->publish_state(count);
    }
  }

  if (this->rssi_sensor_ != nullptr && this->rssi_seen_.load(std::memory_order_relaxed))
    this->rssi_sensor_->publish_state(this->last_flap_rssi_.load(std::memory_order_relaxed));

  this->publish_online_(this->flap_online_, this->last_flap_ms_, this->flap_seen_, this->published_flap_online_,
                        this->published_flap_valid_, now);
  this->publish_online_(this->hub_online_, this->last_hub_ms_, this->hub_seen_, this->published_hub_online_,
                        this->published_hub_valid_, now);
}

void SureFlap::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SureFlap:\n"
                "  Heartbeat timeout: %u ms",
                this->heartbeat_timeout_ms_);
}

}  // namespace esphome::sureflap
