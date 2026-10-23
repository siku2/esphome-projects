#pragma once

#ifdef USE_ESP32

#include <atomic>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/burst_camera/burst_camera.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/task_util/task_util.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/zeromouse_air/air_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "block_decision.h"
#include "camera_tuning.h"
#include "finale.h"
#include "scan.h"
#include "watchdog.h"

#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
#include "esphome/components/zeromouse_cloud/zeromouse_cloud.h"
#endif

namespace esphome::zeromouse {

/// An event with frames, as seen right after the scan ended. Only valid during the callback.
struct CapturedEvent {
  const std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> &frames;
  const std::vector<zeromouse_air::AirSample> &samples;
  const ScanParams &settings;
  const ScanResult &scan;
  const char *result;
  int64_t time_zero_us;
  int32_t pir_to_t0_ms;
  int32_t ambient;
  uint16_t free_value_raw;
  /// Wall clock time of time zero in UTC.
  time_t time_zero_epoch;
  int32_t sample_time_ms(const zeromouse_air::AirSample &sample) const {
    return static_cast<int32_t>((sample.time_us - this->time_zero_us) / 1000);
  }
};

/// Detects cat flap events with the IR sensor.
class ZeroMouse : public Component {
 public:
  void set_air_sensor(zeromouse_air::AirSensor *air) { this->air_ = air; }
  void set_presence(binary_sensor::BinarySensor *presence) { this->presence_ = presence; }
  void set_presence_sampler(zeromouse_air::PresenceSampler *sampler) { this->presence_sampler_ = sampler; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void set_startup_delay(uint32_t delay_ms) { this->startup_delay_ms_ = delay_ms; }
  void set_event_frame_spacing(uint32_t spacing_us) { this->event_frame_spacing_us_ = spacing_us; }
  void set_ir_slope_thresh_value(number::Number *number) { this->ir_slope_thresh_value_ = number; }
  void set_ir_median_count(number::Number *number) { this->ir_median_count_ = number; }
  void set_ir_range_ignore_event(number::Number *number) { this->ir_range_ignore_event_ = number; }
  void set_ir_ambient(number::Number *number) { this->ir_ambient_ = number; }
  void set_cam_shots_before_t0(number::Number *number) {
    this->cam_shots_before_t0_ = number;
    this->camera_tuning_.cam_shots_before_t0 = number;
  }
  void set_upload_empty(switch_::Switch *upload_empty) { this->upload_empty_ = upload_empty; }
  void set_block_enabled(switch_::Switch *block_enabled) { this->block_enabled_ = block_enabled; }
  void set_undecidable_mode(select::Select *select) { this->undecidable_mode_ = select; }
  void set_clean_event_time(number::Number *number) { this->clean_event_time_ = number; }
  void set_finale_interval_max_count(number::Number *number) { this->finale_interval_max_count_ = number; }
  void set_finale_interval_delta(number::Number *number) { this->finale_interval_delta_ = number; }
  void set_ir_prey_event_time(number::Number *number) { this->ir_prey_event_time_ = number; }
  void set_last_ref_event_max_count(number::Number *number) { this->last_ref_event_max_count_ = number; }
  void set_last_ref_event_max_time_sec(number::Number *number) { this->last_ref_event_max_time_sec_ = number; }
  void set_brightness(number::Number *number) { this->camera_tuning_.brightness = number; }
  void set_auto_exp_level(number::Number *number) { this->camera_tuning_.auto_exp_level = number; }
  void set_auto_gain_ctrl_min(number::Number *number) { this->camera_tuning_.auto_gain_ctrl_min = number; }
  void set_auto_gain_ctrl_max(number::Number *number) { this->camera_tuning_.auto_gain_ctrl_max = number; }
  void set_auto_gain_stbl_min(number::Number *number) { this->camera_tuning_.auto_gain_stbl_min = number; }
  void set_auto_gain_stbl_max(number::Number *number) { this->camera_tuning_.auto_gain_stbl_max = number; }
  void set_gain_ceiling(number::Number *number) { this->camera_tuning_.gain_ceiling = number; }
  void set_camera(burst_camera::BurstCamera *camera) { this->camera_ = camera; }
  void set_blocker(switch_::Switch *blocker) { this->blocker_ = blocker; }
  void set_state_sensor(text_sensor::TextSensor *sensor) { this->state_sensor_ = sensor; }
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
  void set_cloud(zeromouse_cloud::ZeroMouseCloud *cloud) { this->cloud_ = cloud; }
#endif
  void set_ir_free_value_sensor(sensor::Sensor *sensor) { this->ir_free_value_sensor_ = sensor; }
  void set_ir_free_value_raw_sensor(sensor::Sensor *sensor) { this->ir_free_value_raw_sensor_ = sensor; }

  using CapturedCallback = std::function<void(const CapturedEvent &)>;
  using PublishedCallback = std::function<void(bool published, uint32_t event_id)>;
  using ClassificationCallback = std::function<void(const std::string &class_name, int class_index, bool block)>;

  /// Runs for every event with frames, before it is published.
  void add_on_event_captured(CapturedCallback callback) { this->captured_callbacks_.push_back(std::move(callback)); }
  /// Runs for every event with frames, once the cloud upload is decided. The event id is only valid if published.
  void add_on_event_published(PublishedCallback callback) { this->published_callbacks_.push_back(std::move(callback)); }
  /// Runs for every classification from the cloud.
  void add_on_classification(ClassificationCallback callback) {
    this->classification_callbacks_.push_back(std::move(callback));
  }

  Trigger<> *get_event_start_trigger() { return &this->event_start_trigger_; }
  Trigger<int> *get_time_zero_trigger() { return &this->time_zero_trigger_; }
  Trigger<std::string> *get_scan_end_trigger() { return &this->scan_end_trigger_; }
  Trigger<> *get_ready_trigger() { return &this->ready_trigger_; }
  Trigger<std::string> *get_event_end_trigger() { return &this->event_end_trigger_; }
  Trigger<uint32_t> *get_published_trigger() { return &this->published_trigger_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  /// Makes the next scan report a time zero, as after a prey event.
  void set_forced_scan(bool forced) { this->forced_scan_ = forced; }

  bool is_idle() const { return this->state_ == State::IDLE; }
  bool is_event_running() const { return this->state_ != State::IDLE && this->state_ != State::CALIBRATING; }
  bool is_suspended() const { return this->suspended_; }
  int64_t event_start_us() const { return this->pir_us_; }

  /// Releases the blocker, ends the event and blocks new events and the blocker until resume() is called.
  void abort_event(const char *reason);
  void resume();
  /// Releases the blocker and ends the event because the network connection dropped.
  void on_network_lost();

  /// The longest time one event can take with the current settings.
  uint32_t event_deadline_ms() const;
  /// True while a prey reference from a classification keeps the blocker allowed beyond one event.
  bool prey_hold_allowed() const;
  /// Called by the watchdog after it released the blocker.
  void on_watchdog_release(bool event_stuck);
  uint16_t ir_free_value() const { return this->calibration_.free_value; }
  uint16_t ir_free_value_raw() const { return this->calibration_.raw_value; }
  uint32_t ir_sensor_status() const { return this->ir_sensor_status_; }
  /// 0 unknown, 1 OV7670, 2 GC032A.
  uint8_t sensor_id() const { return static_cast<uint8_t>(this->camera_->sensor_kind()); }

  /// The samples of the last scan, oldest first. Valid until the next event starts.
  const std::vector<zeromouse_air::AirSample> &scan_samples() const { return this->scan_.samples(); }
  int64_t time_zero_us() const { return this->time_zero_us_; }
  int32_t sample_time_ms(const zeromouse_air::AirSample &sample) const {
    return static_cast<int32_t>((sample.time_us - this->time_zero_us_) / 1000);
  }

 protected:
  static constexpr uint32_t CALIBRATION_INTERVAL_MS = 3600 * 1000;
  static constexpr uint32_t STATUS_RECALIBRATION_COUNT = 0x3F;

  enum class State : uint8_t {
    IDLE,
    SCANNING,
    CAPTURING,
    PUBLISHING,
    AWAITING_CLASSIFICATION,
    PREY_LOCK,
    FINALE,
    CALIBRATING,
  };
  enum class Command : uint8_t { NONE, SCAN, CALIBRATE, FINALE };
  enum class Outcome : uint8_t { TIME_ZERO, EMPTY, DISCARDED };
  enum class MessageType : uint8_t {
    TIME_ZERO,
    SCAN_FINISHED,
    CALIBRATION_FINISHED,
    CALIBRATION_ABORTED,
    FINALE_FINISHED
  };

  struct Message {
    MessageType type;
    Outcome outcome;
    bool valid;
    uint16_t value;
    int64_t time_zero_us;
  };

  struct CalibrationData {
    uint16_t free_value;
    uint16_t raw_value;
  };

  static void task_entry_(void *arg);
  static const char *outcome_name_(Outcome outcome);
  static const char *state_name_(State state);
  void task_main_();
  void run_scan_();
  void run_calibration_();
  void run_finale_();
  void post_(const Message &message);
  void handle_(const Message &message);

  void on_time_synced_();
  void on_presence_(bool state);
  void on_motion_edge_();
  void run_idle_work_();
  const char *start_blocked_reason_() const;
  bool try_start_event_();
  void request_calibration_();
  void finish_scan_(const Message &message);
  void finish_calibration_(const Message &message);
  int32_t read_number_(number::Number *number, int32_t fallback) const;

  void set_state_(State state);
  void set_blocker_(bool on);
  void start_capture_(int64_t time_zero_us);
  void check_scan_complete_();
  void notify_captured_();
  void reserve_event_();
  void report_sensor_state_();
  void publish_event_(bool empty);
  void on_published_(bool empty, bool published, uint32_t event_id);
  void on_classification_(int class_index, bool block);
  void dispatch_();
  void start_finale_();
  void finish_finale_(const Message &message);
  void end_event_(const char *reason);
  bool block_enabled_value_() const { return this->block_enabled_ == nullptr || this->block_enabled_->state; }
  bool has_cloud_() const {
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
    return this->cloud_ != nullptr;
#else
    return false;
#endif
  }
  uint32_t now_sec_() const;
  bool wall_time_valid_() const;
  bool ref_active_(uint32_t max_sec) const;
  bool prey_reference_();
  uint32_t ref_window_s_() const;
  uint32_t classification_wait_s_() const;
  uint32_t prey_lock_s_() const;
  uint32_t finale_total_s_() const;

  zeromouse_air::AirSensor *air_{nullptr};
  binary_sensor::BinarySensor *presence_{nullptr};
  zeromouse_air::PresenceSampler *presence_sampler_{nullptr};
  time::RealTimeClock *time_{nullptr};
  number::Number *ir_slope_thresh_value_{nullptr};
  number::Number *ir_median_count_{nullptr};
  number::Number *ir_range_ignore_event_{nullptr};
  number::Number *ir_ambient_{nullptr};
  number::Number *cam_shots_before_t0_{nullptr};
  switch_::Switch *upload_empty_{nullptr};
  switch_::Switch *block_enabled_{nullptr};
  select::Select *undecidable_mode_{nullptr};
  number::Number *clean_event_time_{nullptr};
  number::Number *finale_interval_max_count_{nullptr};
  number::Number *finale_interval_delta_{nullptr};
  number::Number *ir_prey_event_time_{nullptr};
  number::Number *last_ref_event_max_count_{nullptr};
  number::Number *last_ref_event_max_time_sec_{nullptr};
  burst_camera::BurstCamera *camera_{nullptr};
  CameraTuning camera_tuning_;
  switch_::Switch *blocker_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
#ifdef USE_ZEROMOUSE_CONTROLLER_CLOUD
  zeromouse_cloud::ZeroMouseCloud *cloud_{nullptr};
#endif
  sensor::Sensor *ir_free_value_sensor_{nullptr};
  sensor::Sensor *ir_free_value_raw_sensor_{nullptr};

  std::vector<CapturedCallback> captured_callbacks_;
  std::vector<PublishedCallback> published_callbacks_;
  std::vector<ClassificationCallback> classification_callbacks_;

  Trigger<> event_start_trigger_;
  Trigger<int> time_zero_trigger_;
  Trigger<std::string> scan_end_trigger_;
  Trigger<> ready_trigger_;
  Trigger<std::string> event_end_trigger_;
  Trigger<uint32_t> published_trigger_;

  TaskHandle_t task_{nullptr};
  task_util::LoopInbox<Message> queue_{this, 8};
  Scan scan_{&this->presence_active_, &this->task_abort_};
  ScanParams scan_params_;
  ScanResult scan_result_;
  FinaleParams finale_params_;
  int32_t calibration_ambient_{0};
  bool upload_empty_value_{false};
  std::atomic<Command> command_{Command::NONE};
  std::atomic<bool> presence_active_{false};
  std::atomic<bool> forced_scan_{false};
  std::atomic<bool> task_abort_{false};

  ESPPreferenceObject calibration_pref_;
  RefEvent ref_event_;
  std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> frames_;
  Message scan_message_{};
  int class_index_{CLASS_NONE};
  CalibrationData calibration_{50, 50};
  uint32_t ir_sensor_status_{0};
  uint32_t startup_delay_ms_{13000};
  uint32_t event_frame_spacing_us_{80000};
  uint32_t reserved_event_id_{0};
  time_t reserved_event_time_{0};
  int64_t pir_us_{0};
  int64_t time_zero_us_{0};
  State state_{State::IDLE};
  bool time_synced_{false};
  bool ready_{false};
  bool time_zero_fired_{false};
  bool scan_done_{false};
  bool frames_done_{false};
  bool suspended_{false};
  bool blocker_latched_{false};
  bool pending_start_{false};
  bool calibration_pending_{false};
  bool edge_logged_{false};
  bool motion_hook_{false};
};

}  // namespace esphome::zeromouse

#endif
