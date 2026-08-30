#pragma once

#include <cstdint>
#include <vector>

#include "esphome/components/camera_snapshot/snapshotter.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome::meter_reader {

class MeterReader;

class MeterRebaseNumber : public number::Number {
 public:
  void set_parent(MeterReader *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;

  MeterReader *parent_{nullptr};
};

enum ReaderStatus : uint8_t {
  STATUS_ANCHORING,
  STATUS_OK,
  STATUS_STALE,
  STATUS_DIVERGENT,
};

struct Wheel {
  uint8_t level;
  float resolution;
  float tolerance;
  bool linear;
  float result{0.0f};
  float fit{0.0f};
  bool present{false};
  uint32_t last_accepted_ms{0};
};

class MeterReader : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void on_observation(uint8_t level, float result, float fit, bool accepted);
  void apply_rebase(float liters);

  void set_max_flow(float max_flow) { this->max_flow_ = max_flow; }
  void set_corroborations(uint32_t corroborations) { this->corroborations_ = corroborations; }
  void set_pending_window_ms(uint32_t ms) { this->pending_window_ms_ = ms; }
  void set_stale_after_ms(uint32_t ms) { this->stale_after_ms_ = ms; }
  void set_reanchor_tolerance(float liters) { this->reanchor_tolerance_ = liters; }
  void set_back_tolerance(float liters) { this->back_tolerance_ = liters; }
  void set_snapshotter(camera::snapshot::Snapshotter *snapshotter) { this->snapshotter_ = snapshotter; }
  void add_wheel(uint8_t level, float resolution, float tolerance, bool linear);

  void set_reading_sensor(sensor::Sensor *sensor) { this->reading_sensor_ = sensor; }
  void set_consumption_sensor(sensor::Sensor *sensor) { this->consumption_sensor_ = sensor; }
  void set_confidence_sensor(sensor::Sensor *sensor) { this->confidence_sensor_ = sensor; }
  void set_consistency_sensor(sensor::Sensor *sensor) { this->consistency_sensor_ = sensor; }
  void set_rejected_sensor(sensor::Sensor *sensor) { this->rejected_sensor_ = sensor; }
  void set_status_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
  void set_rebase_number(MeterRebaseNumber *number) { this->rebase_number_ = number; }

 protected:
  float quantum_() const;
  float reading_liters_() const;
  uint64_t reanchor_span_u_() const;
  uint64_t back_span_u_() const;
  float consistency_error_(uint64_t u) const;
  float dial_error_(uint64_t u) const;
  bool consistent_(uint64_t u) const;
  bool coarse_consistent_(uint64_t u) const;
  bool find_consistent_(uint64_t lo, uint64_t span, uint64_t *out) const;

  void process_cycle_(uint32_t now);
  void commit_(uint64_t u, uint32_t now);
  void anchor_(bool jump, uint32_t now);
  void publish_reading_();
  void publish_rejected_();
  void update_status_();
  void save_pref_(uint32_t now);

  std::vector<Wheel> wheels_{};
  camera::snapshot::Snapshotter *snapshotter_{nullptr};
  float max_flow_{1.5f};
  uint32_t corroborations_{1};
  uint32_t pending_window_ms_{6000};
  uint32_t stale_after_ms_{60000};
  float reanchor_tolerance_{100.0f};
  float back_tolerance_{20.0f};

  sensor::Sensor *reading_sensor_{nullptr};
  sensor::Sensor *consumption_sensor_{nullptr};
  sensor::Sensor *confidence_sensor_{nullptr};
  sensor::Sensor *consistency_sensor_{nullptr};
  sensor::Sensor *rejected_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  MeterRebaseNumber *rebase_number_{nullptr};

  uint64_t u_{0};
  bool has_saved_{false};
  bool anchored_{false};
  bool stale_{false};
  bool divergent_{false};
  bool has_pending_{false};
  uint64_t pending_u_{0};
  uint32_t pending_confirmed_{0};
  uint32_t pending_ms_{0};
  uint32_t last_commit_ms_{0};
  uint32_t last_save_ms_{0};
  uint32_t inconsistent_since_ms_{0};
  uint32_t rejected_{0};
  float last_consistency_error_{0.0f};
  ReaderStatus status_{STATUS_ANCHORING};
  ESPPreferenceObject rtc_{};
};

}  // namespace esphome::meter_reader
