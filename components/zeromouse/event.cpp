#include "zeromouse.h"

#ifdef USE_ESP32

#include <algorithm>

#include "esp_timer.h"

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
  if (this->presence_sampler_ != nullptr) {
    this->presence_sampler_->set_busy(state != State::IDLE);
  }
  if (this->state_sensor_ != nullptr) {
    this->state_sensor_->publish_state(state_name_(state));
  }
}

void ZeroMouse::set_blocker_(bool on) {
  if (this->blocker_ == nullptr || this->blocker_->state == on) {
    return;
  }
  if (on) {
    if (this->blocker_latched_ || this->suspended_) {
      ESP_LOGD(TAG, "Blocker stays off because it was released by the watchdog or the controller is suspended");
      return;
    }
    this->blocker_->turn_on();
  } else {
    this->blocker_->turn_off();
  }
}

uint32_t ZeroMouse::now_sec_() const {
  const ESPTime now = this->time_->utcnow();
  return now.is_valid() ? static_cast<uint32_t>(now.timestamp) : millis() / 1000 + 1;
}

bool ZeroMouse::ref_active_(uint32_t max_sec) const {
  const ESPTime now = this->time_->utcnow();
  if (!now.is_valid() || this->ref_event_.time_sec == 0) {
    return false;
  }
  const uint32_t now_sec = static_cast<uint32_t>(now.timestamp);
  return now_sec >= this->ref_event_.time_sec && now_sec - this->ref_event_.time_sec <= max_sec;
}

bool ZeroMouse::prey_reference_() {
  if (this->ref_event_.cls != CLASS_PREY) {
    return false;
  }
  if (this->ref_active_(this->ref_window_s_())) {
    return true;
  }
  this->ref_event_ = {};
  return false;
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
      this->notify_captured_();
      this->publish_event_(false);
      break;
    case Outcome::EMPTY:
      this->camera_->disarm_event();
      this->frames_.clear();
      this->publish_event_(true);
      break;
    case Outcome::DISCARDED:
      this->end_event_("discarded");
      if (!this->scan_message_.valid || !this->presence_active_ || !this->try_start_event_()) {
        this->set_blocker_(false);
      }
      break;
  }
}

void ZeroMouse::notify_captured_() {
  if (this->captured_callbacks_.empty()) {
    return;
  }
  const int64_t age_us = esp_timer_get_time() - this->time_zero_us_;
  const CapturedEvent event{this->frames_,
                            this->scan_samples(),
                            this->scan_params_,
                            this->scan_result_,
                            outcome_name_(this->scan_message_.outcome),
                            this->time_zero_us_,
                            static_cast<int32_t>((this->pir_us_ - this->time_zero_us_) / 1000),
                            this->read_number_(this->ir_ambient_, 1000),
                            this->calibration_.raw_value,
                            static_cast<time_t>(this->time_->utcnow().timestamp - age_us / 1000000)};
  for (const auto &callback : this->captured_callbacks_) {
    callback(event);
  }
}

void ZeroMouse::publish_event_(bool empty) {
  this->set_state_(State::PUBLISHING);
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
  if (this->cloud_ != nullptr) {
    if (this->cloud_->is_failed()) {
      ESP_LOGW(TAG, "The cloud component failed, not publishing");
      this->defer([this, empty]() { this->on_published_(empty, false, 0); });
      return;
    }
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
  if (!empty) {
    for (const auto &callback : this->published_callbacks_) {
      callback(published, event_id);
    }
  }
  if (this->state_ != State::PUBLISHING) {
    return;
  }
  if (!published) {
    ESP_LOGI(TAG, "Event not published, ending it without a finale");
    this->set_blocker_(false);
    this->end_event_("not_published");
    return;
  }
  this->published_trigger_.trigger(event_id);
  if (empty) {
    this->end_event_("empty");
    if (!this->presence_active_ || !this->try_start_event_()) {
      this->set_blocker_(false);
    }
    return;
  }
  this->set_state_(State::AWAITING_CLASSIFICATION);
  this->set_timeout("classification", this->classification_wait_s_() * 1000, [this]() {
    ESP_LOGI(TAG, "Classification timeout, continuing without it");
    this->dispatch_();
  });
}

void ZeroMouse::on_classification_(int class_index, bool block) {
  BlockParams params;
  if (this->undecidable_mode_ != nullptr) {
    params.undecidable_mode = static_cast<int>(this->undecidable_mode_->active_index().value_or(0));
  }
  params.ref_max_time_sec = this->ref_window_s_();
  params.ref_max_count = std::max<int32_t>(0, this->read_number_(this->last_ref_event_max_count_, 10));

  const RefEvent before = this->ref_event_;
  const BlockResult result = decide_block(this->ref_event_, class_index, block, this->now_sec_(), params);
  if (this->ref_event_.cls == CLASS_PREY && this->ref_event_.time_sec != before.time_sec) {
    this->blocker_latched_ = false;
  }
  if (this->state_ != State::AWAITING_CLASSIFICATION && this->state_ != State::PREY_LOCK &&
      this->state_ != State::FINALE) {
    ESP_LOGI(TAG, "Classification %d arrived outside of the decision states, only the reference event is updated",
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
  const bool prey = this->prey_reference_() || this->class_index_ == CLASS_PREY;
  if (!prey) {
    this->start_finale_();
    return;
  }
  this->set_state_(State::PREY_LOCK);
  this->set_timeout("prey_lock", this->prey_lock_s_() * 1000, [this]() { this->start_finale_(); });
}

void ZeroMouse::start_finale_() {
  this->finale_params_.free_value = this->calibration_.free_value;
  this->finale_params_.check_drift = (this->ir_sensor_status_ & STATUS_RECALIBRATION_COUNT) == 0;
  this->finale_params_.ambient = this->read_number_(this->ir_ambient_, 1000);
  this->finale_params_.min_duration_ms =
      this->class_index_ == CLASS_CLEAN ? std::max<int32_t>(0, this->read_number_(this->clean_event_time_, 30)) * 1000 : 0;
  this->finale_params_.max_duration_ms = this->finale_total_s_() * 1000;
  this->set_state_(State::FINALE);
  this->command_ = Command::FINALE;
  xTaskNotifyGive(this->task_);
}

void ZeroMouse::finish_finale_(const Message &message) {
  if (this->state_ != State::FINALE) {
    return;
  }
  const char *reason = message.value == static_cast<uint16_t>(FinaleResult::STABLE) ? "finale_stable" : "finale_timeout";
  const bool prey = (this->class_index_ == CLASS_PREY || this->prey_reference_()) && this->class_index_ != CLASS_CLEAN;
  if (prey) {
    this->forced_scan_ = true;
    this->end_event_(reason);
    if (!this->try_start_event_()) {
      ESP_LOGW(TAG, "Not restarting after prey, releasing the blocker");
      this->forced_scan_ = false;
      this->set_blocker_(false);
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
  this->run_idle_work_();
}

}  // namespace esphome::zeromouse

#endif
