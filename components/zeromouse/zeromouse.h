#pragma once

#ifdef USE_ESP32

#include <atomic>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/zeromouse_air/air_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "scan.h"

namespace esphome::zeromouse {

/// Detects cat flap events with the IR sensor.
class ZeroMouse : public Component {
 public:
  void set_air_sensor(zeromouse_air::AirSensor *air) { this->air_ = air; }
  void set_presence(binary_sensor::BinarySensor *presence) { this->presence_ = presence; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void set_startup_delay(uint32_t delay_ms) { this->startup_delay_ms_ = delay_ms; }
  void set_ir_slope_thresh_value(number::Number *number) { this->ir_slope_thresh_value_ = number; }
  void set_ir_median_count(number::Number *number) { this->ir_median_count_ = number; }
  void set_ir_range_ignore_event(number::Number *number) { this->ir_range_ignore_event_ = number; }
  void set_ir_ambient(number::Number *number) { this->ir_ambient_ = number; }
  void set_cam_shots_before_t0(number::Number *number) { this->cam_shots_before_t0_ = number; }
  void set_upload_empty(switch_::Switch *upload_empty) { this->upload_empty_ = upload_empty; }
  void set_ir_free_value_sensor(sensor::Sensor *sensor) { this->ir_free_value_sensor_ = sensor; }
  void set_ir_free_value_raw_sensor(sensor::Sensor *sensor) { this->ir_free_value_raw_sensor_ = sensor; }

  Trigger<> *get_event_start_trigger() { return &this->event_start_trigger_; }
  Trigger<int> *get_time_zero_trigger() { return &this->time_zero_trigger_; }
  Trigger<std::string> *get_scan_end_trigger() { return &this->scan_end_trigger_; }
  Trigger<> *get_ready_trigger() { return &this->ready_trigger_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  /// Makes the next scan report a time zero, as after a prey event.
  void set_forced_scan(bool forced) { this->forced_scan_ = forced; }

  bool is_idle() const { return this->state_ == State::IDLE; }
  uint16_t ir_free_value() const { return this->calibration_.free_value; }
  uint16_t ir_free_value_raw() const { return this->calibration_.raw_value; }
  uint32_t ir_sensor_status() const { return this->ir_sensor_status_; }

  /// The samples of the last scan, oldest first. Valid while the controller is idle.
  const std::vector<zeromouse_air::AirSample> &scan_samples() const { return this->scan_.samples(); }
  int64_t time_zero_us() const { return this->time_zero_us_; }
  int32_t sample_time_ms(const zeromouse_air::AirSample &sample) const {
    return static_cast<int32_t>((sample.time_us - this->time_zero_us_) / 1000);
  }

 protected:
  static constexpr uint32_t CALIBRATION_INTERVAL_MS = 3600 * 1000;
  static constexpr uint32_t STATUS_RECALIBRATION_COUNT = 0x3F;

  enum class State : uint8_t { IDLE, SCANNING, CALIBRATING };
  enum class Command : uint8_t { NONE, SCAN, CALIBRATE };
  enum class Outcome : uint8_t { TIME_ZERO, EMPTY, DISCARDED };
  enum class MessageType : uint8_t { TIME_ZERO, SCAN_FINISHED, CALIBRATION_FINISHED };

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
  void task_main_();
  void run_scan_();
  void run_calibration_();
  void post_(const Message &message);
  void handle_(const Message &message);

  void on_time_synced_();
  void on_presence_(bool state);
  void try_start_event_();
  void request_calibration_();
  void finish_scan_(const Message &message);
  void finish_calibration_(const Message &message);
  int32_t read_number_(number::Number *number, int32_t fallback) const;

  zeromouse_air::AirSensor *air_{nullptr};
  binary_sensor::BinarySensor *presence_{nullptr};
  time::RealTimeClock *time_{nullptr};
  number::Number *ir_slope_thresh_value_{nullptr};
  number::Number *ir_median_count_{nullptr};
  number::Number *ir_range_ignore_event_{nullptr};
  number::Number *ir_ambient_{nullptr};
  number::Number *cam_shots_before_t0_{nullptr};
  switch_::Switch *upload_empty_{nullptr};
  sensor::Sensor *ir_free_value_sensor_{nullptr};
  sensor::Sensor *ir_free_value_raw_sensor_{nullptr};

  Trigger<> event_start_trigger_;
  Trigger<int> time_zero_trigger_;
  Trigger<std::string> scan_end_trigger_;
  Trigger<> ready_trigger_;

  TaskHandle_t task_{nullptr};
  QueueHandle_t queue_{nullptr};
  Scan scan_{&this->presence_active_};
  ScanParams scan_params_;
  ScanResult scan_result_;
  int32_t calibration_ambient_{0};
  bool upload_empty_value_{false};
  std::atomic<Command> command_{Command::NONE};
  std::atomic<bool> presence_active_{false};
  std::atomic<bool> forced_scan_{false};

  ESPPreferenceObject calibration_pref_;
  CalibrationData calibration_{50, 50};
  uint32_t ir_sensor_status_{0};
  uint32_t startup_delay_ms_{13000};
  int64_t pir_us_{0};
  int64_t time_zero_us_{0};
  State state_{State::IDLE};
  bool time_synced_{false};
  bool ready_{false};
  bool time_zero_fired_{false};
};

}  // namespace esphome::zeromouse

#endif
