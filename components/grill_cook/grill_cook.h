#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "esphome/components/climate/climate.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include "cook_model.h"

namespace esphome::grill_cook {

using Zone = grill_cook_model::Zone;
using Phase = grill_cook_model::Phase;

// Wraps CookModel with the ESPHome entities it reads and publishes. All cook
// logic lives in CookModel; this class only moves state in and out of it.
class GrillCook : public PollingComponent {
 public:
  static constexpr size_t HISTORY_LEN = grill_cook_model::CookModel::HISTORY_LEN;
  static constexpr uint32_t HISTORY_STEP_S = grill_cook_model::CookModel::HISTORY_STEP_S;

  void update() override;
  void dump_config() override;

  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void set_zone_climate(Zone zone, climate::Climate *climate) {
    this->zones_[static_cast<size_t>(zone)].climate = climate;
  }
  void set_zone_probe(Zone zone, sensor::Sensor *probe) { this->zones_[static_cast<size_t>(zone)].probe = probe; }
  void set_meat_probe(sensor::Sensor *probe) { this->meat_probe_ = probe; }
  void set_meat_target_number(number::Number *number) { this->meat_target_number_ = number; }

  void set_phase_sensor(text_sensor::TextSensor *sensor) { this->phase_sensor_ = sensor; }
  void set_meat_rate_sensor(sensor::Sensor *sensor) { this->meat_rate_sensor_ = sensor; }
  void set_eta_sensor(text_sensor::TextSensor *sensor) { this->eta_sensor_ = sensor; }
  void set_remaining_minutes_sensor(sensor::Sensor *sensor) { this->remaining_minutes_sensor_ = sensor; }

  Trigger<std::string> *get_phase_trigger() { return &this->phase_trigger_; }
  Trigger<> *get_cook_started_trigger() { return &this->cook_started_trigger_; }
  Trigger<> *get_cook_ended_trigger() { return &this->cook_ended_trigger_; }
  Trigger<> *get_zone_presence_trigger() { return &this->zone_presence_trigger_; }

  void start_cook() { this->model_.start(); }
  void end_cook() { this->model_.end(); }
  bool is_cooking() const { return this->model_.is_cooking(); }
  Phase phase() const { return this->model_.phase(); }
  const char *phase_str() const { return this->model_.phase_str(); }

  bool zone_on(Zone zone) const;
  bool zone_present(Zone zone) const { return this->model_.zone_present(zone); }
  bool zone_at_setpoint(Zone zone) const { return this->model_.zone_at_setpoint(zone); }
  float zone_target(Zone zone) const;
  float zone_temperature(Zone zone) const;

  bool meat_probe_present() const { return this->model_.meat_probe_present(); }
  float meat_temperature() const;
  float meat_rate() const { return this->model_.meat_rate(); }
  int remaining_minutes() const { return this->model_.remaining_minutes(); }
  std::string eta_clock() const;

  const std::array<float, HISTORY_LEN> &zone_history(Zone zone) const { return this->model_.zone_history(zone); }
  const std::array<float, HISTORY_LEN> &meat_history() const { return this->model_.meat_history(); }

 protected:
  struct ZoneIo {
    climate::Climate *climate{nullptr};
    sensor::Sensor *probe{nullptr};
  };

  static float sensor_state_or_nan_(sensor::Sensor *sensor);
  static void publish_if_changed_(text_sensor::TextSensor *sensor, const std::string &value, std::string *memo);

  time::RealTimeClock *time_{nullptr};
  ZoneIo zones_[2]{};
  sensor::Sensor *meat_probe_{nullptr};
  number::Number *meat_target_number_{nullptr};

  text_sensor::TextSensor *phase_sensor_{nullptr};
  sensor::Sensor *meat_rate_sensor_{nullptr};
  text_sensor::TextSensor *eta_sensor_{nullptr};
  sensor::Sensor *remaining_minutes_sensor_{nullptr};

  Trigger<std::string> phase_trigger_;
  Trigger<> cook_started_trigger_;
  Trigger<> cook_ended_trigger_;
  Trigger<> zone_presence_trigger_;

  grill_cook_model::CookModel model_;

  bool last_cooking_{false};
  bool last_zone_present_[2]{false, false};
  Phase last_phase_{Phase::IDLE};
  std::string last_phase_text_;
  std::string last_eta_text_;
  float last_meat_rate_{NAN};
  int last_remaining_minutes_{-1};
};

}  // namespace esphome::grill_cook
