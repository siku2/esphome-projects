#pragma once

#include <cstdint>

#include "esphome/components/camera_snapshot/snapshotter.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "reader_core.h"

namespace esphome::meter_reader {

class MeterReader;

class MeterRebaseNumber : public number::Number {
 public:
  void set_parent(MeterReader *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;

  MeterReader *parent_{nullptr};
};

class MeterReader : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void on_observation(uint8_t level, float result, float fit, bool accepted);
  void apply_rebase(float liters);

  void set_max_flow(float max_flow) { this->core_.params.max_flow = max_flow; }
  void set_corroborations(uint32_t corroborations) { this->core_.params.corroborations = corroborations; }
  void set_pending_window_ms(uint32_t ms) { this->core_.params.pending_window_ms = ms; }
  void set_stale_after_ms(uint32_t ms) { this->core_.params.stale_after_ms = ms; }
  void set_reanchor_tolerance(float liters) { this->core_.params.reanchor_tolerance = liters; }
  void set_back_tolerance(float liters) { this->core_.params.back_tolerance = liters; }
  void set_snapshotter(camera::snapshot::Snapshotter *snapshotter) { this->snapshotter_ = snapshotter; }
  void add_wheel(uint8_t level, float resolution, float tolerance, bool linear) {
    this->core_.add_wheel(level, resolution, tolerance, linear);
  }

  void set_reading_sensor(sensor::Sensor *sensor) { this->reading_sensor_ = sensor; }
  void set_consumption_sensor(sensor::Sensor *sensor) { this->consumption_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_consistency_sensor(sensor::Sensor *sensor) { this->consistency_sensor_ = sensor; }
  void set_rejected_sensor(sensor::Sensor *sensor) { this->rejected_sensor_ = sensor; }
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
  void set_rebase_number(MeterRebaseNumber *number) { this->rebase_number_ = number; }

 protected:
  void on_cycle_end_(uint32_t now);
  void apply_(const ::meter_reader_core::CycleResult &result);
  void log_events_(const ::meter_reader_core::CycleResult &result);
  void publish_reading_();
  void publish_rejected_();
  void publish_status_();

  ::meter_reader_core::ReaderCore core_{};
  camera::snapshot::Snapshotter *snapshotter_{nullptr};

  sensor::Sensor *reading_sensor_{nullptr};
  sensor::Sensor *consumption_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  sensor::Sensor *consistency_sensor_{nullptr};
  sensor::Sensor *rejected_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  MeterRebaseNumber *rebase_number_{nullptr};

  ESPPreferenceObject rtc_{};
};

}  // namespace esphome::meter_reader
