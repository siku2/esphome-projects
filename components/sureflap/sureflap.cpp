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
  this->sniffer_->set_frame_callback([this](uint64_t, uint8_t, int8_t, uint8_t, const uint8_t *psdu, size_t length) {
    this->on_frame_(psdu, length);
  });
}

void SureFlap::on_frame_(const uint8_t *psdu, size_t length) {
  sureflap_protocol::Frame frame = sureflap_protocol::parse_frame(psdu, length);
  if (!frame.valid)
    return;
  this->frame_count_.fetch_add(1, std::memory_order_relaxed);
  if (!frame.src_is_hub && frame.type == sureflap_protocol::MSG_HEARTBEAT) {
    this->last_heartbeat_ms_.store(millis(), std::memory_order_relaxed);
    this->heartbeat_seen_.store(true, std::memory_order_relaxed);
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

  if (this->flap_online_ != nullptr) {
    bool online = this->heartbeat_seen_.load(std::memory_order_relaxed) &&
                  (now - this->last_heartbeat_ms_.load(std::memory_order_relaxed)) < this->heartbeat_timeout_ms_;
    if (!this->published_online_valid_ || online != this->published_online_) {
      this->published_online_valid_ = true;
      this->published_online_ = online;
      this->flap_online_->publish_state(online);
    }
  }
}

void SureFlap::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SureFlap:\n"
                "  Heartbeat timeout: %u ms",
                this->heartbeat_timeout_ms_);
}

}  // namespace esphome::sureflap
