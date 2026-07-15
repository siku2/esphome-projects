#pragma once

#include <deque>
#include <string>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/component.h"

namespace esphome {
namespace grill_thermal_model {

enum class CookPhase : uint8_t {
  IDLE = 0,
  LEARNING,
  ACTIVE,
  STALL,
  STRETCH,
  PULL,
  RESTING,
  COMPLETE,
  PAUSE,
};

class GrillThermalModel : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_rtc(time::RealTimeClock *rtc) { this->rtc_ = rtc; }
  void set_grill_probe(sensor::Sensor *sensor) { this->grill_probe_ = sensor; }
  void set_meat_probe(sensor::Sensor *sensor) { this->meat_probe_ = sensor; }
  void set_target_number(number::Number *number) { this->target_number_ = number; }
  void set_humidity_sensor(sensor::Sensor *sensor) { this->humidity_sensor_ = sensor; }
  void set_lid_reference_sensor(sensor::Sensor *sensor) { this->lid_reference_sensor_ = sensor; }

  void set_finish_time_sensor(text_sensor::TextSensor *sensor) { this->finish_time_sensor_ = sensor; }
  void set_pull_time_sensor(text_sensor::TextSensor *sensor) { this->pull_time_sensor_ = sensor; }
  void set_cook_phase_sensor(text_sensor::TextSensor *sensor) { this->cook_phase_sensor_ = sensor; }
  void set_thermal_mass_index_sensor(sensor::Sensor *sensor) { this->thermal_mass_index_sensor_ = sensor; }
  void set_rest_end_time_sensor(text_sensor::TextSensor *sensor) { this->rest_end_time_sensor_ = sensor; }
  void set_rest_remaining_min_sensor(sensor::Sensor *sensor) { this->rest_remaining_min_sensor_ = sensor; }
  void set_rest_duration_minutes(uint32_t minutes) { this->rest_duration_s_ = minutes * 60U; }

  void start_new_cook();
  void trigger_pull();
  void on_climate_active_changed(bool active);

 protected:
  struct RegressionPoint {
    float t_s;
    float y;
  };

  struct AlphaBetaFilter {
    float value{0.0f};
    float velocity{0.0f};
    bool initialized{false};

    void update(float sample, float dt_s, float alpha, float beta);
  };

  time::RealTimeClock *rtc_{nullptr};
  sensor::Sensor *grill_probe_{nullptr};
  sensor::Sensor *meat_probe_{nullptr};
  number::Number *target_number_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *lid_reference_sensor_{nullptr};

  text_sensor::TextSensor *finish_time_sensor_{nullptr};
  text_sensor::TextSensor *pull_time_sensor_{nullptr};
  text_sensor::TextSensor *cook_phase_sensor_{nullptr};
  text_sensor::TextSensor *rest_end_time_sensor_{nullptr};
  sensor::Sensor *thermal_mass_index_sensor_{nullptr};
  sensor::Sensor *rest_remaining_min_sensor_{nullptr};

  AlphaBetaFilter ambient_filter_;
  AlphaBetaFilter internal_filter_;

  std::deque<RegressionPoint> regression_points_;

  CookPhase phase_{CookPhase::IDLE};
  CookPhase last_non_pause_phase_{CookPhase::IDLE};

  uint32_t session_start_ms_{0};
  uint32_t last_update_ms_{0};
  uint32_t learning_start_ms_{0};
  bool learning_started_{false};
  float learning_baseline_internal_c_{0.0f};
  bool lag_measured_{false};
  float lag_seconds_{0.0f};
  float thermal_mass_index_{0.0f};

  uint32_t last_regression_ms_{0};
  float regression_slope_{0.0f};
  float regression_intercept_{0.0f};
  float regression_r2_{0.0f};
  bool regression_valid_{false};

  bool stall_penalty_seeded_{false};
  float stall_penalty_s_{0.0f};
  uint32_t rest_duration_s_{1800};
  bool rest_started_{false};
  uint32_t rest_start_ms_{0};
  float pull_peak_internal_c_{0.0f};

  bool climate_active_{false};
  bool probe_fault_{false};

  std::string last_finish_time_text_;
  std::string last_pull_time_text_;
  std::string last_phase_text_;
  std::string last_rest_end_time_text_;
  float last_rest_remaining_min_{NAN};

  static constexpr float AB_ALPHA = 0.32f;
  static constexpr float AB_BETA = 0.06f;
  static constexpr float LEARNING_DELTA_C = 2.0f;
  static constexpr float LID_DROP_RATIO = 0.85f;
  static constexpr float STALL_SLOPE_C_PER_MIN = 0.05f;
  static constexpr float ACTIVE_LIMIT_C = 65.0f;
  static constexpr float STRETCH_LIMIT_C = 75.0f;
  static constexpr float MIN_VALID_DELTA_C = 0.2f;
  static constexpr float MIN_R2 = 0.90f;
  static constexpr float STALL_PENALTY_MULTIPLIER = 4.0f;
  static constexpr float STALL_PENALTY_DECAY_TAU_S = 1800.0f;

  bool inputs_ready_() const;
  float get_target_temperature_() const;
  float get_humidity_() const;
  float get_lid_reference_() const;
  float get_session_elapsed_s_() const;
  bool is_lid_open_(float filtered_ambient, float lid_reference) const;

  void update_phase_(float filtered_ambient, float filtered_internal, float dt_s, bool lid_open);
  void push_regression_point_(float t_s, float filtered_ambient, float filtered_internal);
  void compute_regression_();
  float compute_wet_bulb_temperature_(float ambient_c, float humidity_percent) const;

  bool compute_eta_seconds_(float target_temp_c, float now_s, float *out_eta_s) const;
  float compute_pull_temperature_(float ambient_c, float target_temp_c) const;

  std::string format_clock_from_eta_(float eta_s) const;
  void publish_if_changed_(text_sensor::TextSensor *sensor, const std::string &value, std::string *memo);
  void publish_phase_();
  void clear_time_entities_();
  void reset_session_();
  const char *phase_to_str_(CookPhase phase) const;
};

}  // namespace grill_thermal_model
}  // namespace esphome
