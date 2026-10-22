#include "zeromouse.h"

#ifdef USE_ESP32

#include <algorithm>

#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::zeromouse {

static const char *const TAG = "zeromouse";

static constexpr uint8_t EVENT_FRAMES = 8;

void ZeroMouse::set_state_(State state) {
  if (this->state_ == state) {
    return;
  }
  this->state_ = state;
  if (this->state_sensor_ != nullptr) {
    this->state_sensor_->publish_state(state_name_(state));
  }
}

void ZeroMouse::set_blocker_(bool on) {
  if (this->blocker_ == nullptr || this->blocker_->state == on) {
    return;
  }
  if (on) {
    this->blocker_->turn_on();
  } else {
    this->blocker_->turn_off();
  }
}

uint32_t ZeroMouse::now_sec_() const {
  const ESPTime now = this->time_->utcnow();
  return now.is_valid() ? static_cast<uint32_t>(now.timestamp) : millis() / 1000 + 1;
}

void ZeroMouse::load_ref_event_() {
  this->ref_event_pref_ = global_preferences->make_preference<RefEvent>(fnv1_hash("zeromouse.ref_event"));
  if (!this->ref_event_pref_.load(&this->ref_event_)) {
    this->ref_event_ = {};
  }
}

void ZeroMouse::start_capture_(int64_t time_zero_us) {
  if (this->state_ == State::SCANNING) {
    this->set_state_(State::CAPTURING);
  }
  const bool started = this->camera_->capture_event(time_zero_us, EVENT_FRAMES, [this](auto frames) {
    this->frames_ = std::move(frames);
    this->frames_done_ = true;
    this->check_scan_complete_();
  });
  if (!started) {
    ESP_LOGW(TAG, "Starting the capture failed");
    this->frames_done_ = true;
  }
}

void ZeroMouse::check_scan_complete_() {
  if ((this->state_ != State::SCANNING && this->state_ != State::CAPTURING) || !this->scan_done_) {
    return;
  }
  switch (this->scan_message_.outcome) {
    case Outcome::TIME_ZERO:
      if (!this->frames_done_) {
        return;
      }
      if (this->frames_.empty()) {
        this->set_blocker_(false);
        this->end_event_("capture_failed");
        return;
      }
      this->publish_event_(false);
      break;
    case Outcome::EMPTY:
      this->camera_->disarm_event();
      this->publish_event_(true);
      break;
    case Outcome::DISCARDED:
      this->set_blocker_(false);
      this->end_event_("discarded");
      if (this->scan_message_.valid && this->presence_active_) {
        this->try_start_event_();
      }
      break;
  }
}

void ZeroMouse::publish_event_(bool empty) {
  this->set_state_(State::PUBLISHING);
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
  if (this->cloud_ != nullptr) {
    zeromouse_cloud::ZeroMouseCloud::EventData data;
    data.frames = std::move(this->frames_);
    data.time_zero_us = this->time_zero_us_;
    data.pir_to_t0_ms = static_cast<int32_t>((this->pir_us_ - this->time_zero_us_) / 1000);
    data.air.reserve(this->scan_samples().size());
    for (const auto &sample : this->scan_samples()) {
      data.air.push_back({sample.mean, this->sample_time_ms(sample), sample.slope, sample.environment});
    }
    this->frames_.clear();
    this->cloud_->publish_event_async(std::move(data), [this, empty](bool published, uint32_t event_id) {
      this->on_published_(empty, published, event_id);
    });
    return;
  }
#endif
  this->on_published_(empty, false, 0);
}

void ZeroMouse::on_published_(bool empty, bool published, uint32_t event_id) {
  if (this->state_ != State::PUBLISHING) {
    return;
  }
  if (published) {
    this->published_trigger_.trigger(event_id);
  }
  if (empty) {
    this->set_blocker_(false);
    this->end_event_("empty");
    if (this->presence_active_) {
      this->try_start_event_();
    }
    return;
  }
  if (!published) {
    ESP_LOGI(TAG, "Event not published, continuing without a classification");
    this->dispatch_();
    return;
  }
  this->set_state_(State::AWAITING_CLASSIFICATION);
  const uint32_t wait_s = std::max<int32_t>(1, this->read_number_(this->finale_interval_delta_, 5));
  this->set_timeout("classification", wait_s * 1000, [this]() {
    ESP_LOGI(TAG, "Classification timeout, continuing without it");
    this->dispatch_();
  });
}

void ZeroMouse::on_classification_(int class_index, bool block) {
  BlockParams params;
  if (this->undecidable_mode_ != nullptr) {
    params.undecidable_mode = static_cast<int>(this->undecidable_mode_->active_index().value_or(0));
  }
  params.ref_max_time_sec = std::max<int32_t>(0, this->read_number_(this->last_ref_event_max_time_sec_, 1800));
  params.ref_max_count = std::max<int32_t>(0, this->read_number_(this->last_ref_event_max_count_, 10));

  const RefEvent before = this->ref_event_;
  const BlockResult result = decide_block(this->ref_event_, class_index, block, this->now_sec_(), params);
  if (!(this->ref_event_ == before)) {
    this->ref_event_pref_.save(&this->ref_event_);
  }
  if (this->state_ == State::IDLE || this->state_ == State::CALIBRATING) {
    ESP_LOGI(TAG, "Classification %d arrived without an active event, only the reference event is updated",
             class_index);
    return;
  }
  this->set_blocker_(result.block && this->block_enabled_value_());

  if (this->state_ == State::AWAITING_CLASSIFICATION) {
    this->cancel_timeout("classification");
    this->class_index_ = result.class_index;
    this->dispatch_();
  }
}

void ZeroMouse::dispatch_() {
  const bool prey = this->ref_event_.cls == CLASS_PREY || this->class_index_ == CLASS_PREY;
  if (!prey) {
    this->start_finale_();
    return;
  }
  this->set_state_(State::PREY_LOCK);
  const uint32_t lock_s = std::max<int32_t>(1, this->read_number_(this->ir_prey_event_time_, 30));
  this->set_timeout("prey_lock", lock_s * 1000, [this]() { this->start_finale_(); });
}

void ZeroMouse::start_finale_() {
  const int32_t delta_s = std::max<int32_t>(1, this->read_number_(this->finale_interval_delta_, 5));
  const int32_t count = std::max<int32_t>(1, this->read_number_(this->finale_interval_max_count_, 12));
  this->finale_params_.free_value = this->calibration_.free_value;
  this->finale_params_.check_drift = (this->ir_sensor_status_ & STATUS_RECALIBRATION_COUNT) == 0;
  this->finale_params_.ambient = this->read_number_(this->ir_ambient_, 1000);
  this->finale_params_.min_duration_ms =
      this->class_index_ == CLASS_CLEAN ? std::max<int32_t>(0, this->read_number_(this->clean_event_time_, 30)) * 1000 : 0;
  this->finale_params_.max_duration_ms = static_cast<uint32_t>(count) * delta_s * 1000;
  this->set_state_(State::FINALE);
  this->command_ = Command::FINALE;
  xTaskNotifyGive(this->task_);
}

void ZeroMouse::finish_finale_(const Message &message) {
  if (this->state_ != State::FINALE) {
    return;
  }
  const char *reason = message.value == static_cast<uint16_t>(FinaleResult::STABLE) ? "finale_stable" : "finale_timeout";
  const bool prey = (this->class_index_ == CLASS_PREY || this->ref_event_.cls == CLASS_PREY) &&
                    this->class_index_ != CLASS_CLEAN;
  if (prey) {
    this->forced_scan_ = true;
    this->end_event_(reason);
    if (const char *blocked = this->start_blocked_reason_()) {
      ESP_LOGW(TAG, "Not restarting after prey because %s, releasing the blocker", blocked);
      this->forced_scan_ = false;
      this->set_blocker_(false);
    } else {
      this->try_start_event_();
    }
    return;
  }
  this->set_blocker_(false);
  this->end_event_(reason);
}

void ZeroMouse::end_event_(const char *reason) {
  this->camera_->disarm_event();
  this->cancel_timeout("classification");
  this->cancel_timeout("prey_lock");
  this->frames_.clear();
  this->set_state_(State::IDLE);
  ESP_LOGI(TAG, "Event ended: %s", reason);
  this->event_end_trigger_.trigger(reason);
}

}  // namespace esphome::zeromouse

#endif
