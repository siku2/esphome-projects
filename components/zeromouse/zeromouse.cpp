#include "zeromouse.h"

#ifdef USE_ESP32

#include <algorithm>
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

static constexpr int32_t MAX_PREY_LOCK_S = 300;
static constexpr int32_t MAX_FINALE_S = 180;
static constexpr int32_t MAX_REF_WINDOW_S = 3600;
static constexpr int32_t MAX_CLASSIFICATION_WAIT_S = 60;
static constexpr uint32_t HOLD_CAP_S = 1800;
static constexpr uint32_t SCAN_BUDGET_S = 125;
static constexpr uint32_t CAPTURE_BUDGET_S = 10;
static constexpr uint32_t PUBLISH_BUDGET_S = 60;
static constexpr uint32_t MARGIN_S = 10;

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

const char *ZeroMouse::state_name_(State state) {
  switch (state) {
    case State::IDLE:
      return "idle";
    case State::SCANNING:
      return "scanning";
    case State::CAPTURING:
      return "capturing";
    case State::PUBLISHING:
      return "publishing";
    case State::AWAITING_CLASSIFICATION:
      return "awaiting_classification";
    case State::PREY_LOCK:
      return "prey_lock";
    case State::FINALE:
      return "finale";
    case State::CALIBRATING:
      return "calibrating";
  }
  return "idle";
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

  this->camera_tuning_.setup(this->camera_);
  this->report_sensor_state_();
  if (this->block_enabled_ != nullptr) {
    this->block_enabled_->add_on_state_callback([this](bool state) {
      if (!state) {
        this->set_blocker_(false);
      }
    });
  }
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
  if (this->cloud_ != nullptr) {
    this->cloud_->add_on_classification_callback([this](const std::string &class_name, int class_index, bool block) {
      this->on_classification_(class_index, block);
      for (const auto &callback : this->classification_callbacks_) {
        callback(class_name, class_index, block);
      }
    });
    this->cloud_->set_idle_check([this]() { return this->is_idle(); });
  }
#endif

  if (this->presence_->has_state()) {
    this->presence_active_ = this->presence_->state;
  }
  if (this->presence_sampler_ != nullptr) {
    this->motion_hook_ = this->presence_sampler_->add_on_motion_callback([this]() { this->on_motion_edge_(); });
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
                "  IR free value: %u (raw %u)\n"
                "  Cloud: %s",
                this->startup_delay_ms_, this->calibration_.free_value, this->calibration_.raw_value,
                YESNO(this->has_cloud_()));
  LOG_TEXT_SENSOR("  ", "Event State", this->state_sensor_);
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
    this->task_abort_ = false;
    switch (this->command_.exchange(Command::NONE)) {
      case Command::SCAN:
        this->run_scan_();
        break;
      case Command::CALIBRATE:
        this->run_calibration_();
        break;
      case Command::FINALE:
        this->run_finale_();
        break;
      case Command::NONE:
        break;
    }
  }
}

void ZeroMouse::run_scan_() {
  this->scan_result_ = this->scan_.run(*this->air_, this->presence_sampler_, this->scan_params_, [this](int64_t time_zero_us) {
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
  const CalibrationResult result = calibrate(*this->air_, this->calibration_ambient_, this->task_abort_);
  this->post_({result.aborted ? MessageType::CALIBRATION_ABORTED : MessageType::CALIBRATION_FINISHED, Outcome::DISCARDED,
               result.valid, result.value, 0});
}

void ZeroMouse::run_finale_() {
  const FinaleResult result = run_finale(*this->air_, this->finale_params_, this->presence_active_, this->task_abort_);
  this->post_({MessageType::FINALE_FINISHED, Outcome::DISCARDED, true, static_cast<uint16_t>(result), 0});
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
      if (this->state_ != State::SCANNING && this->state_ != State::CAPTURING) {
        break;
      }
      this->time_zero_us_ = message.time_zero_us;
      this->time_zero_fired_ = true;
      this->start_capture_(message.time_zero_us);
      this->time_zero_trigger_.trigger(static_cast<int>((this->pir_us_ - message.time_zero_us) / 1000));
      break;
    case MessageType::SCAN_FINISHED:
      if (this->state_ == State::SCANNING || this->state_ == State::CAPTURING) {
        this->finish_scan_(message);
      }
      break;
    case MessageType::CALIBRATION_FINISHED:
    case MessageType::CALIBRATION_ABORTED:
      if (this->state_ == State::CALIBRATING) {
        this->finish_calibration_(message);
      }
      break;
    case MessageType::FINALE_FINISHED:
      this->finish_finale_(message);
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
    this->run_idle_work_();
  });
}

void ZeroMouse::on_presence_(bool state) {
  this->presence_active_ = state;
  if (state) {
    if (!this->motion_hook_) {
      this->on_motion_edge_();
    }
    return;
  }
  this->pending_start_ = false;
  this->blocker_latched_ = false;
  this->run_idle_work_();
}

void ZeroMouse::on_motion_edge_() {
  switch (this->state_) {
    case State::IDLE:
      if (!this->try_start_event_() && !this->ready_) {
        this->pending_start_ = true;
      }
      break;
    case State::CALIBRATING:
      this->pending_start_ = true;
      this->task_abort_ = this->ready_;
      break;
    default:
      if (!this->edge_logged_) {
        this->edge_logged_ = true;
        ESP_LOGD(TAG, "Motion ignored because an event is running");
      }
      break;
  }
}

void ZeroMouse::run_idle_work_() {
  this->defer([this]() {
    if (this->state_ != State::IDLE) {
      return;
    }
    if (this->pending_start_) {
      this->pending_start_ = false;
      if (this->presence_active_) {
        this->try_start_event_();
        return;
      }
    }
    if (this->calibration_pending_ && !this->presence_active_) {
      this->request_calibration_();
    }
  });
}

const char *ZeroMouse::start_blocked_reason_() const {
  if (this->is_failed()) {
    return "the controller failed";
  } else if (!this->time_->now().is_valid()) {
    return "the time is not valid";
  } else if (!network::is_connected()) {
    return "the network is not connected";
  } else if (this->suspended_) {
    return "the controller is suspended";
  } else if (!this->ready_) {
    return "the startup delay is active";
  } else if (this->state_ != State::IDLE) {
    return this->state_ == State::CALIBRATING ? "the sensor is calibrating" : "an event is running";
  }
  return nullptr;
}

bool ZeroMouse::try_start_event_() {
  if (const char *reason = this->start_blocked_reason_()) {
    ESP_LOGD(TAG, "Event rejected because %s", reason);
    return false;
  }

  this->scan_params_.slope_thresh = this->read_number_(this->ir_slope_thresh_value_, 20);
  this->scan_params_.mean_window = this->read_number_(this->ir_median_count_, 5);
  this->scan_params_.range_ignore_event = this->read_number_(this->ir_range_ignore_event_, 100);
  this->scan_params_.shots_before_t0 = this->read_number_(this->cam_shots_before_t0_, 0);
  this->scan_params_.ambient = this->read_number_(this->ir_ambient_, 1000);
  this->scan_params_.ir_free_value = this->calibration_.free_value;
  this->scan_params_.forced = this->forced_scan_.exchange(false);
  this->upload_empty_value_ = this->upload_empty_ != nullptr && this->upload_empty_->state;

  this->pir_us_ = esp_timer_get_time();
  this->pending_start_ = false;
  this->edge_logged_ = false;
  this->time_zero_fired_ = false;
  this->scan_done_ = false;
  this->frames_done_ = false;
  this->frames_.clear();
  this->reserved_event_id_ = 0;
  this->class_index_ = CLASS_NONE;
  this->set_state_(State::SCANNING);
  this->command_ = Command::SCAN;
  xTaskNotifyGive(this->task_);
  if (this->block_enabled_value_()) {
    this->set_blocker_(true);
  }
  if (!this->camera_->arm_event(static_cast<uint8_t>(std::clamp<int32_t>(this->scan_params_.shots_before_t0, 0, 8)),
                                 this->event_frame_spacing_us_)) {
    ESP_LOGW(TAG, "Arming the camera failed");
  }
  ESP_LOGD(TAG, "Event started");
  this->event_start_trigger_.trigger();
  return true;
}

void ZeroMouse::request_calibration_() {
  if (this->is_failed()) {
    return;
  }
  if (this->state_ != State::IDLE || this->suspended_) {
    ESP_LOGD(TAG, "Calibration postponed because the controller is busy");
    this->calibration_pending_ = true;
    return;
  }
  this->calibration_pending_ = false;
  this->calibration_ambient_ = this->read_number_(this->ir_ambient_, 1000);
  this->set_state_(State::CALIBRATING);
  this->command_ = Command::CALIBRATE;
  xTaskNotifyGive(this->task_);
}

void ZeroMouse::finish_scan_(const Message &message) {
  const char *name = outcome_name_(message.outcome);
  if (!message.valid) {
    ESP_LOGE(TAG, "Reading the air sensor failed");
  }
  this->scan_done_ = true;
  this->scan_message_ = message;
  if (message.outcome != Outcome::DISCARDED) {
    this->time_zero_us_ = message.time_zero_us;
  }
  const int pir_to_t0_ms = static_cast<int>((this->pir_us_ - message.time_zero_us) / 1000);
  if (message.outcome == Outcome::TIME_ZERO && !this->time_zero_fired_) {
    this->time_zero_fired_ = true;
    this->start_capture_(message.time_zero_us);
    this->time_zero_trigger_.trigger(pir_to_t0_ms);
  }
  ESP_LOGI(TAG, "Scan ended: %u samples, mean %d to %d, PIR %d ms from T0, result %s",
           static_cast<unsigned>(this->scan_result_.sample_count), static_cast<int>(this->scan_result_.min_mean),
           static_cast<int>(this->scan_result_.max_mean), pir_to_t0_ms, name);
  this->scan_end_trigger_.trigger(name);
  this->check_scan_complete_();
}

void ZeroMouse::finish_calibration_(const Message &message) {
  this->set_state_(State::IDLE);
  if (message.type == MessageType::CALIBRATION_ABORTED) {
    ESP_LOGD(TAG, "Calibration aborted");
    this->calibration_pending_ = true;
    this->run_idle_work_();
    return;
  }
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
  this->report_sensor_state_();
  this->run_idle_work_();
}

void ZeroMouse::abort_event(const char *reason) {
  ESP_LOGW(TAG, "Aborting the event: %s", reason);
  this->suspended_ = true;
  this->pending_start_ = false;
  this->task_abort_ = true;
  this->set_blocker_(false);
  if (this->is_event_running()) {
    this->end_event_(reason);
  } else {
    this->camera_->disarm_event();
  }
}

void ZeroMouse::resume() {
  this->suspended_ = false;
  this->run_idle_work_();
}

void ZeroMouse::on_network_lost() {
  if (!this->is_event_running()) {
    return;
  }
  ESP_LOGW(TAG, "Network lost, ending the event");
  this->pending_start_ = false;
  this->task_abort_ = true;
  this->set_blocker_(false);
  this->end_event_("network");
}

void ZeroMouse::on_watchdog_release(bool event_stuck) {
  if (this->presence_active_) {
    this->blocker_latched_ = true;
  }
  if (event_stuck && this->is_event_running()) {
    this->pending_start_ = false;
    this->task_abort_ = true;
    this->end_event_("watchdog");
  }
}

uint32_t ZeroMouse::event_deadline_ms() const {
  return (SCAN_BUDGET_S + CAPTURE_BUDGET_S + PUBLISH_BUDGET_S + this->classification_wait_s_() + this->prey_lock_s_() +
          this->finale_total_s_() + MARGIN_S) *
         1000;
}

bool ZeroMouse::prey_hold_allowed() const {
  return this->ref_event_.cls == CLASS_PREY && this->ref_active_(std::min(this->ref_window_s_(), HOLD_CAP_S));
}

uint32_t ZeroMouse::ref_window_s_() const {
  return std::clamp<int32_t>(this->read_number_(this->last_ref_event_max_time_sec_, 1800), 0, MAX_REF_WINDOW_S);
}

uint32_t ZeroMouse::classification_wait_s_() const {
  return std::clamp<int32_t>(this->read_number_(this->finale_interval_delta_, 5), 1, MAX_CLASSIFICATION_WAIT_S);
}

uint32_t ZeroMouse::prey_lock_s_() const {
  return std::clamp<int32_t>(this->read_number_(this->ir_prey_event_time_, 30), 1, MAX_PREY_LOCK_S);
}

uint32_t ZeroMouse::finale_total_s_() const {
  const int64_t delta = std::max<int32_t>(1, this->read_number_(this->finale_interval_delta_, 5));
  const int64_t count = std::max<int32_t>(1, this->read_number_(this->finale_interval_max_count_, 12));
  return static_cast<uint32_t>(std::min<int64_t>(delta * count, MAX_FINALE_S));
}

int32_t ZeroMouse::read_number_(number::Number *number, int32_t fallback) const {
  if (number == nullptr || std::isnan(number->state)) {
    return fallback;
  }
  return static_cast<int32_t>(std::lround(number->state));
}

}  // namespace esphome::zeromouse

#endif
