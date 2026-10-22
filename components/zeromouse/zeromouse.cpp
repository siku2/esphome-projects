#include "zeromouse.h"

#ifdef USE_ESP32

#include <cinttypes>
#include <cmath>

#include "esp_timer.h"

#include "esphome/components/network/util.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "calibration.h"

namespace esphome::zeromouse {

static const char *const TAG = "zeromouse";

static constexpr uint32_t TASK_STACK_BYTES = 4096;
static constexpr BaseType_t TASK_CORE = 1;
static constexpr UBaseType_t TASK_PRIORITY_OFFSET = 2;
static constexpr UBaseType_t QUEUE_LENGTH = 8;

const char *ZeroMouse::outcome_name_(Outcome outcome) {
  switch (outcome) {
    case Outcome::TIME_ZERO:
      return "time_zero";
    case Outcome::EMPTY:
      return "empty";
    case Outcome::DISCARDED:
      return "discarded";
  }
  return "discarded";
}

void ZeroMouse::setup() {
  this->queue_ = xQueueCreate(QUEUE_LENGTH, sizeof(Message));
  const UBaseType_t priority = uxTaskPriorityGet(nullptr) + TASK_PRIORITY_OFFSET;
  if (this->queue_ == nullptr ||
      xTaskCreatePinnedToCore(task_entry_, "zeromouse", TASK_STACK_BYTES, this, priority, &this->task_, TASK_CORE) !=
          pdPASS) {
    ESP_LOGE(TAG, "Creating the scan task failed");
    this->mark_failed();
    return;
  }

  this->calibration_pref_ = global_preferences->make_preference<CalibrationData>(fnv1_hash("zeromouse.calibration"));
  if (!this->calibration_pref_.load(&this->calibration_)) {
    this->calibration_ = {50, 50};
  }

  if (this->presence_->has_state()) {
    this->presence_active_ = this->presence_->state;
  }
  this->presence_->add_on_state_callback([this](bool state) { this->on_presence_(state); });
  this->time_->add_on_time_sync_callback([this]() { this->on_time_synced_(); });
  if (this->time_->now().is_valid()) {
    this->on_time_synced_();
  }

  this->set_interval("calibration", CALIBRATION_INTERVAL_MS, [this]() { this->request_calibration_(); });
  this->request_calibration_();
  this->disable_loop();
}

void ZeroMouse::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ZeroMOUSE:\n"
                "  Startup delay: %" PRIu32 " ms\n"
                "  IR free value: %u (raw %u)",
                this->startup_delay_ms_, this->calibration_.free_value, this->calibration_.raw_value);
  LOG_SENSOR("  ", "IR Free Value", this->ir_free_value_sensor_);
  LOG_SENSOR("  ", "IR Free Value Raw", this->ir_free_value_raw_sensor_);
}

void ZeroMouse::loop() {
  Message message;
  while (xQueueReceive(this->queue_, &message, 0) == pdTRUE) {
    this->handle_(message);
  }
  this->disable_loop();
}

void ZeroMouse::task_entry_(void *arg) { static_cast<ZeroMouse *>(arg)->task_main_(); }

void ZeroMouse::task_main_() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    switch (this->command_.exchange(Command::NONE)) {
      case Command::SCAN:
        this->run_scan_();
        break;
      case Command::CALIBRATE:
        this->run_calibration_();
        break;
      case Command::NONE:
        break;
    }
  }
}

void ZeroMouse::run_scan_() {
  this->scan_result_ = this->scan_.run(*this->air_, this->scan_params_, [this](int64_t time_zero_us) {
    this->post_({MessageType::TIME_ZERO, Outcome::TIME_ZERO, true, 0, time_zero_us});
  });

  Outcome outcome = Outcome::DISCARDED;
  if (this->scan_result_.threshold_reached && !this->scan_result_.failed) {
    outcome = Outcome::TIME_ZERO;
  } else if (this->upload_empty_value_ && !this->scan_result_.failed) {
    outcome = Outcome::EMPTY;
  }
  this->post_({MessageType::SCAN_FINISHED, outcome, !this->scan_result_.failed, 0, this->scan_result_.time_zero_us});
}

void ZeroMouse::run_calibration_() {
  const CalibrationResult result = calibrate(*this->air_, this->calibration_ambient_);
  this->post_({MessageType::CALIBRATION_FINISHED, Outcome::DISCARDED, result.valid, result.value, 0});
}

void ZeroMouse::post_(const Message &message) {
  if (xQueueSend(this->queue_, &message, 0) != pdTRUE) {
    ESP_LOGE(TAG, "Message queue is full");
  }
  this->enable_loop_soon_any_context();
}

void ZeroMouse::handle_(const Message &message) {
  switch (message.type) {
    case MessageType::TIME_ZERO:
      this->time_zero_us_ = message.time_zero_us;
      this->time_zero_fired_ = true;
      this->time_zero_trigger_.trigger(static_cast<int>((this->pir_us_ - message.time_zero_us) / 1000));
      break;
    case MessageType::SCAN_FINISHED:
      this->finish_scan_(message);
      break;
    case MessageType::CALIBRATION_FINISHED:
      this->finish_calibration_(message);
      break;
  }
}

void ZeroMouse::on_time_synced_() {
  if (this->time_synced_) {
    return;
  }
  this->time_synced_ = true;
  this->set_timeout("startup_delay", this->startup_delay_ms_, [this]() {
    this->ready_ = true;
    ESP_LOGD(TAG, "Startup delay passed");
    this->ready_trigger_.trigger();
  });
}

void ZeroMouse::on_presence_(bool state) {
  this->presence_active_ = state;
  if (state) {
    this->try_start_event_();
  }
}

void ZeroMouse::try_start_event_() {
  const char *reason = nullptr;
  if (this->is_failed()) {
    reason = "the controller failed";
  } else if (!this->time_->now().is_valid()) {
    reason = "the time is not valid";
  } else if (!network::is_connected()) {
    reason = "the network is not connected";
  } else if (!this->ready_) {
    reason = "the startup delay is active";
  } else if (this->state_ != State::IDLE) {
    reason = this->state_ == State::SCANNING ? "an event is running" : "the sensor is calibrating";
  }
  if (reason != nullptr) {
    ESP_LOGD(TAG, "Event rejected because %s", reason);
    return;
  }

  this->scan_params_.slope_thresh = this->read_number_(this->ir_slope_thresh_value_, 20);
  this->scan_params_.mean_window = this->read_number_(this->ir_median_count_, 5);
  this->scan_params_.range_ignore_event = this->read_number_(this->ir_range_ignore_event_, 100);
  this->scan_params_.shots_before_t0 = this->read_number_(this->cam_shots_before_t0_, 0);
  this->scan_params_.ir_free_value = this->calibration_.free_value;
  this->scan_params_.forced = this->forced_scan_.exchange(false);
  this->upload_empty_value_ = this->upload_empty_ != nullptr && this->upload_empty_->state;

  this->pir_us_ = esp_timer_get_time();
  this->time_zero_fired_ = false;
  this->state_ = State::SCANNING;
  this->command_ = Command::SCAN;
  xTaskNotifyGive(this->task_);
  ESP_LOGD(TAG, "Event started");
  this->event_start_trigger_.trigger();
}

void ZeroMouse::request_calibration_() {
  if (this->is_failed() || this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Calibration skipped because the controller is busy");
    return;
  }
  this->calibration_ambient_ = this->read_number_(this->ir_ambient_, 1000);
  this->state_ = State::CALIBRATING;
  this->command_ = Command::CALIBRATE;
  xTaskNotifyGive(this->task_);
}

void ZeroMouse::finish_scan_(const Message &message) {
  this->state_ = State::IDLE;
  const char *name = outcome_name_(message.outcome);
  if (!message.valid) {
    ESP_LOGE(TAG, "Reading the air sensor failed");
  }
  if (message.outcome == Outcome::TIME_ZERO) {
    this->time_zero_us_ = message.time_zero_us;
    if (!this->time_zero_fired_) {
      this->time_zero_trigger_.trigger(static_cast<int>((this->pir_us_ - message.time_zero_us) / 1000));
    }
  }
  ESP_LOGI(TAG, "Scan ended: %u samples, mean %d to %d, PIR %d ms from T0, result %s",
           static_cast<unsigned>(this->scan_result_.sample_count), static_cast<int>(this->scan_result_.min_mean),
           static_cast<int>(this->scan_result_.max_mean), static_cast<int>((this->pir_us_ - message.time_zero_us) / 1000),
           name);
  this->scan_end_trigger_.trigger(name);
  ESP_LOGI(TAG, "Event ended after the scan");

  if (message.outcome != Outcome::TIME_ZERO && this->presence_active_) {
    this->try_start_event_();
  }
}

void ZeroMouse::finish_calibration_(const Message &message) {
  this->state_ = State::IDLE;
  this->calibration_.raw_value = message.value;
  if (message.valid) {
    this->calibration_.free_value = message.value;
    this->ir_sensor_status_ &= ~STATUS_RECALIBRATION_COUNT;
  } else if ((this->ir_sensor_status_ & STATUS_RECALIBRATION_COUNT) < STATUS_RECALIBRATION_COUNT) {
    this->ir_sensor_status_++;
  }
  this->calibration_pref_.save(&this->calibration_);
  ESP_LOGI(TAG, "Calibration %s: free value %u, raw %u", message.valid ? "succeeded" : "failed",
           this->calibration_.free_value, this->calibration_.raw_value);
  if (this->ir_free_value_sensor_ != nullptr) {
    this->ir_free_value_sensor_->publish_state(this->calibration_.free_value);
  }
  if (this->ir_free_value_raw_sensor_ != nullptr) {
    this->ir_free_value_raw_sensor_->publish_state(this->calibration_.raw_value);
  }
}

int32_t ZeroMouse::read_number_(number::Number *number, int32_t fallback) const {
  if (number == nullptr || std::isnan(number->state)) {
    return fallback;
  }
  return static_cast<int32_t>(std::lround(number->state));
}

}  // namespace esphome::zeromouse

#endif
