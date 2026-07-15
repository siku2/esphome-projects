#include "grill_thermal_model.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace grill_thermal_model {

static const char *const TAG = "grill_thermal_model";

void GrillThermalModel::AlphaBetaFilter::update(float sample, float dt_s, float alpha, float beta) {
  if (!this->initialized || dt_s <= 0.0f) {
    this->value = sample;
    this->velocity = 0.0f;
    this->initialized = true;
    return;
  }

  const float predicted = this->value + this->velocity * dt_s;
  const float residual = sample - predicted;
  this->value = predicted + alpha * residual;
  this->velocity = this->velocity + (beta * residual) / dt_s;
}

void GrillThermalModel::setup() {
  this->phase_ = CookPhase::IDLE;
  this->last_non_pause_phase_ = CookPhase::IDLE;
  this->session_start_ms_ = millis();
}

void GrillThermalModel::dump_config() {
  ESP_LOGCONFIG(TAG, "Grill Thermal Model:");
  ESP_LOGCONFIG(TAG, "  Grill probe: %p", static_cast<void *>(this->grill_probe_));
  ESP_LOGCONFIG(TAG, "  Meat probe: %p", static_cast<void *>(this->meat_probe_));
  ESP_LOGCONFIG(TAG, "  Target number: %p", static_cast<void *>(this->target_number_));
  ESP_LOGCONFIG(TAG, "  Humidity sensor: %p", static_cast<void *>(this->humidity_sensor_));
  ESP_LOGCONFIG(TAG, "  Lid reference sensor: %p", static_cast<void *>(this->lid_reference_sensor_));
  ESP_LOGCONFIG(TAG, "  Finish time sensor: %p", static_cast<void *>(this->finish_time_sensor_));
  ESP_LOGCONFIG(TAG, "  Pull time sensor: %p", static_cast<void *>(this->pull_time_sensor_));
  ESP_LOGCONFIG(TAG, "  Cook phase sensor: %p", static_cast<void *>(this->cook_phase_sensor_));
  ESP_LOGCONFIG(TAG, "  Thermal mass index sensor: %p", static_cast<void *>(this->thermal_mass_index_sensor_));
  ESP_LOGCONFIG(TAG, "  Rest end time sensor: %p", static_cast<void *>(this->rest_end_time_sensor_));
  ESP_LOGCONFIG(TAG, "  Rest remaining minutes sensor: %p", static_cast<void *>(this->rest_remaining_min_sensor_));
  ESP_LOGCONFIG(TAG, "  Rest duration: %u min", static_cast<unsigned>(this->rest_duration_s_ / 60U));
  ESP_LOGCONFIG(TAG, "  Model constants: alpha=%.2f beta=%.2f min_r2=%.2f", AB_ALPHA, AB_BETA, MIN_R2);
  LOG_UPDATE_INTERVAL(this);
}

bool GrillThermalModel::inputs_ready_() const {
  if (this->grill_probe_ == nullptr || this->meat_probe_ == nullptr)
    return false;
  if (!this->grill_probe_->has_state() || !this->meat_probe_->has_state())
    return false;
  return std::isfinite(this->grill_probe_->state) && std::isfinite(this->meat_probe_->state);
}

float GrillThermalModel::get_target_temperature_() const { return this->target_number_->state; }

float GrillThermalModel::get_humidity_() const {
  if (this->humidity_sensor_ != nullptr && this->humidity_sensor_->has_state()) {
    return clamp(this->humidity_sensor_->get_state(), 0.0f, 100.0f);
  }
  return NAN;
}

float GrillThermalModel::get_lid_reference_() const {
  if (this->lid_reference_sensor_ != nullptr && this->lid_reference_sensor_->has_state()) {
    float v = this->lid_reference_sensor_->state;
    if (std::isfinite(v))
      return v;
  }
  return NAN;
}

float GrillThermalModel::get_session_elapsed_s_() const { return (millis() - this->session_start_ms_) / 1000.0f; }

bool GrillThermalModel::is_lid_open_(float filtered_ambient, float lid_reference) const {
  if (!std::isfinite(lid_reference))
    return false;
  return filtered_ambient < (lid_reference * LID_DROP_RATIO);
}

void GrillThermalModel::push_regression_point_(float t_s, float filtered_ambient, float filtered_internal) {
  const float delta = filtered_ambient - filtered_internal;
  if (delta <= MIN_VALID_DELTA_C)
    return;

  this->regression_points_.push_back({t_s, logf(delta)});

  const float cutoff = t_s - 15.0f * 60.0f;
  while (!this->regression_points_.empty() && this->regression_points_.front().t_s < cutoff) {
    this->regression_points_.pop_front();
  }
}

void GrillThermalModel::compute_regression_() {
  if (this->regression_points_.size() < 8) {
    this->regression_valid_ = false;
    return;
  }

  const float n = static_cast<float>(this->regression_points_.size());
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  for (const auto &p : this->regression_points_) {
    sum_x += p.t_s;
    sum_y += p.y;
    sum_xx += p.t_s * p.t_s;
    sum_xy += p.t_s * p.y;
  }

  const float denom = n * sum_xx - sum_x * sum_x;
  if (fabsf(denom) < 1e-6f) {
    this->regression_valid_ = false;
    return;
  }

  const float slope = (n * sum_xy - sum_x * sum_y) / denom;
  const float intercept = (sum_y - slope * sum_x) / n;

  float ss_res = 0.0f;
  float ss_tot = 0.0f;
  const float mean_y = sum_y / n;
  for (const auto &p : this->regression_points_) {
    const float fit = slope * p.t_s + intercept;
    const float residual = p.y - fit;
    const float centered = p.y - mean_y;
    ss_res += residual * residual;
    ss_tot += centered * centered;
  }

  float r2 = 0.0f;
  if (ss_tot > 1e-6f) {
    r2 = 1.0f - (ss_res / ss_tot);
  }

  this->regression_slope_ = slope;
  this->regression_intercept_ = intercept;
  this->regression_r2_ = r2;
  this->regression_valid_ = slope < -1e-6f;

  ESP_LOGD(TAG, "Regression slope=%.6f intercept=%.3f r2=%.3f points=%u", slope, intercept, r2,
           static_cast<unsigned>(this->regression_points_.size()));
}

float GrillThermalModel::compute_pull_temperature_(float ambient_c, float target_temp_c) const {
  const float mass_adjustment = 10.0f - clamp(this->thermal_mass_index_, 0.0f, 4.0f);
  return target_temp_c - ((ambient_c - target_temp_c) / mass_adjustment);
}

float GrillThermalModel::compute_wet_bulb_temperature_(float ambient_c, float humidity_percent) const {
  if (!std::isfinite(humidity_percent))
    return NAN;
  const float rh = clamp(humidity_percent, 0.0f, 100.0f);
  return ambient_c * atanf(0.151977f * sqrtf(rh + 8.313659f)) + atanf(ambient_c + rh) - atanf(rh - 1.676331f) +
         0.00391838f * powf(rh, 1.5f) * atanf(0.023101f * rh) - 4.686035f;
}

bool GrillThermalModel::compute_eta_seconds_(float target_temp_c, float now_s, float *out_eta_s) const {
  if (!this->regression_valid_)
    return false;

  const float delta = this->ambient_filter_.value - target_temp_c;
  if (delta <= MIN_VALID_DELTA_C)
    return false;

  const float y_target = logf(delta);
  const float t_target = (y_target - this->regression_intercept_) / this->regression_slope_;

  float remaining = t_target - now_s;
  if (!std::isfinite(remaining))
    return false;

  remaining = std::max(0.0f, remaining);
  remaining += this->stall_penalty_s_;

  *out_eta_s = remaining;
  return true;
}

std::string GrillThermalModel::format_clock_from_eta_(float eta_s) const {
  if (this->rtc_ == nullptr)
    return "Estimating...";

  auto now = this->rtc_->now();
  if (!now.is_valid())
    return "Estimating...";

  const auto epoch = now.timestamp + static_cast<int32_t>(roundf(eta_s));
  auto done = ESPTime::from_epoch_local(epoch);
  char buffer[6];
  done.strftime(buffer, sizeof(buffer), "%H:%M");
  return std::string(buffer);
}

void GrillThermalModel::publish_if_changed_(text_sensor::TextSensor *sensor, const std::string &value,
                                            std::string *memo) {
  if (sensor == nullptr)
    return;
  if (*memo == value)
    return;
  sensor->publish_state(value);
  *memo = value;
}

const char *GrillThermalModel::phase_to_str_(CookPhase phase) const {
  switch (phase) {
    case CookPhase::IDLE:
      return "IDLE";
    case CookPhase::LEARNING:
      return "LEARNING";
    case CookPhase::ACTIVE:
      return "ACTIVE";
    case CookPhase::STALL:
      return "STALL";
    case CookPhase::STRETCH:
      return "STRETCH";
    case CookPhase::PULL:
      return "PULL";
    case CookPhase::RESTING:
      return "RESTING";
    case CookPhase::COMPLETE:
      return "COMPLETE";
    case CookPhase::PAUSE:
      return "PAUSE";
    default:
      return "IDLE";
  }
}

void GrillThermalModel::publish_phase_() {
  this->publish_if_changed_(this->cook_phase_sensor_, this->phase_to_str_(this->phase_), &this->last_phase_text_);
}

void GrillThermalModel::clear_time_entities_() {
  this->publish_if_changed_(this->finish_time_sensor_, "", &this->last_finish_time_text_);
  this->publish_if_changed_(this->pull_time_sensor_, "", &this->last_pull_time_text_);
  this->publish_if_changed_(this->rest_end_time_sensor_, "", &this->last_rest_end_time_text_);
  if (this->rest_remaining_min_sensor_ != nullptr && !std::isnan(this->last_rest_remaining_min_)) {
    this->rest_remaining_min_sensor_->publish_state(NAN);
    this->last_rest_remaining_min_ = NAN;
  }
}

void GrillThermalModel::reset_session_() {
  ESP_LOGI(TAG, "Resetting cook session");

  this->phase_ = CookPhase::IDLE;
  this->last_non_pause_phase_ = CookPhase::IDLE;

  this->ambient_filter_.initialized = false;
  this->internal_filter_.initialized = false;

  this->regression_points_.clear();
  this->regression_valid_ = false;
  this->regression_slope_ = 0.0f;
  this->regression_intercept_ = 0.0f;
  this->regression_r2_ = 0.0f;
  this->last_regression_ms_ = 0;

  this->learning_started_ = false;
  this->lag_measured_ = false;
  this->lag_seconds_ = 0.0f;
  this->thermal_mass_index_ = 0.0f;

  this->stall_penalty_seeded_ = false;
  this->stall_penalty_s_ = 0.0f;

  this->rest_started_ = false;
  this->rest_start_ms_ = 0;
  this->pull_peak_internal_c_ = 0.0f;

  this->session_start_ms_ = millis();
  this->last_update_ms_ = 0;
  this->probe_fault_ = false;

  this->clear_time_entities_();
  if (this->thermal_mass_index_sensor_ != nullptr) {
    this->thermal_mass_index_sensor_->publish_state(NAN);
  }
  this->publish_phase_();
}

void GrillThermalModel::start_new_cook() {
  ESP_LOGI(TAG, "Starting new cook session");
  this->reset_session_();
}

void GrillThermalModel::trigger_pull() {
  if (this->phase_ == CookPhase::IDLE || this->phase_ == CookPhase::COMPLETE || this->phase_ == CookPhase::PAUSE) {
    return;
  }
  ESP_LOGI(TAG, "Triggering pull phase");
  this->phase_ = CookPhase::PULL;
  if (this->internal_filter_.initialized) {
    this->pull_peak_internal_c_ = std::max(this->pull_peak_internal_c_, this->internal_filter_.value);
  }
}

void GrillThermalModel::on_climate_active_changed(bool active) {
  if (active && !this->climate_active_) {
    this->start_new_cook();
  } else if (!active && this->climate_active_) {
    this->trigger_pull();
  }
  this->climate_active_ = active;
}

void GrillThermalModel::update_phase_(float filtered_ambient, float filtered_internal, float dt_s, bool lid_open) {
  const float target_temp = this->get_target_temperature_();
  const float humidity = this->get_humidity_();
  const float wet_bulb_c = this->compute_wet_bulb_temperature_(filtered_ambient, humidity);

  if (filtered_internal < 30.0f) {
    this->phase_ = CookPhase::IDLE;
    this->learning_started_ = false;
    this->lag_measured_ = false;
    this->stall_penalty_s_ = 0.0f;
    this->stall_penalty_seeded_ = false;
    this->rest_started_ = false;
    this->pull_peak_internal_c_ = 0.0f;
    return;
  }

  if (lid_open) {
    if (this->phase_ != CookPhase::PAUSE) {
      this->last_non_pause_phase_ = this->phase_;
    }
    this->phase_ = CookPhase::PAUSE;
    return;
  }

  if (this->phase_ == CookPhase::PAUSE) {
    this->phase_ = this->last_non_pause_phase_;
  }

  if (this->phase_ == CookPhase::COMPLETE) {
    return;
  }

  if (!this->learning_started_ && filtered_ambient >= target_temp * 0.97f) {
    this->learning_started_ = true;
    this->learning_start_ms_ = millis();
    this->learning_baseline_internal_c_ = filtered_internal;
    this->phase_ = CookPhase::LEARNING;
    return;
  }

  if (this->learning_started_ && !this->lag_measured_) {
    const float rise = filtered_internal - this->learning_baseline_internal_c_;
    if (rise >= LEARNING_DELTA_C) {
      this->lag_seconds_ = (millis() - this->learning_start_ms_) / 1000.0f;
      this->thermal_mass_index_ = clamp(this->lag_seconds_ / 600.0f, 0.1f, 5.0f);
      this->lag_measured_ = true;
      if (this->thermal_mass_index_sensor_ != nullptr) {
        this->thermal_mass_index_sensor_->publish_state(this->thermal_mass_index_);
      }
    } else {
      this->phase_ = CookPhase::LEARNING;
      return;
    }
  }

  const float slope_c_per_min = this->internal_filter_.velocity * 60.0f;

  if (this->phase_ == CookPhase::RESTING) {
    const float rest_elapsed_s = (millis() - this->rest_start_ms_) / 1000.0f;
    if (rest_elapsed_s >= static_cast<float>(this->rest_duration_s_)) {
      this->phase_ = CookPhase::COMPLETE;
      this->rest_started_ = false;
    }
    return;
  }

  if (this->phase_ == CookPhase::PULL) {
    this->pull_peak_internal_c_ = std::max(this->pull_peak_internal_c_, filtered_internal);
    if (!this->rest_started_ && filtered_internal <= (this->pull_peak_internal_c_ - 0.3f)) {
      this->phase_ = CookPhase::RESTING;
      this->rest_started_ = true;
      this->rest_start_ms_ = millis();
      return;
    }
    if (filtered_internal >= STRETCH_LIMIT_C) {
      return;
    }
  }

  if (filtered_internal >= this->compute_pull_temperature_(filtered_ambient, target_temp)) {
    this->phase_ = CookPhase::PULL;
    this->pull_peak_internal_c_ = std::max(this->pull_peak_internal_c_, filtered_internal);
    return;
  }

  if (filtered_internal >= STRETCH_LIMIT_C) {
    this->phase_ = CookPhase::STRETCH;
    return;
  }

  const bool near_wet_bulb =
      std::isfinite(wet_bulb_c) && filtered_internal >= (wet_bulb_c - 0.7f) && filtered_internal >= ACTIVE_LIMIT_C;
  const bool stalled =
      (filtered_internal >= ACTIVE_LIMIT_C && fabsf(slope_c_per_min) <= STALL_SLOPE_C_PER_MIN) || near_wet_bulb;

  if (stalled) {
    this->phase_ = CookPhase::STALL;
    if (!this->stall_penalty_seeded_ && this->lag_seconds_ > 0.0f) {
      this->stall_penalty_s_ += this->lag_seconds_ * STALL_PENALTY_MULTIPLIER;
      this->stall_penalty_seeded_ = true;
    }
    this->stall_penalty_s_ += dt_s * (0.20f + 0.02f * this->thermal_mass_index_);
    return;
  }

  this->phase_ = CookPhase::ACTIVE;
}

void GrillThermalModel::update() {
  if (!this->inputs_ready_()) {
    if (!this->probe_fault_) {
      this->probe_fault_ = true;
      if (this->phase_ != CookPhase::PAUSE && this->phase_ != CookPhase::IDLE && this->phase_ != CookPhase::COMPLETE) {
        this->last_non_pause_phase_ = this->phase_;
      }
      this->phase_ = CookPhase::PAUSE;
      this->clear_time_entities_();
      this->publish_phase_();
      ESP_LOGW(TAG, "Probe fault detected");
    }
    return;
  }

  if (this->probe_fault_) {
    this->probe_fault_ = false;
    this->ambient_filter_.initialized = false;
    this->internal_filter_.initialized = false;
    this->regression_points_.clear();
    this->regression_valid_ = false;
    this->last_update_ms_ = 0;
    if (this->phase_ == CookPhase::PAUSE) {
      this->phase_ = this->last_non_pause_phase_;
    }
    ESP_LOGI(TAG, "Probe recovered, reinitializing filters");
  }

  const uint32_t now_ms = millis();
  float dt_s = 0.0f;
  if (this->last_update_ms_ == 0) {
    dt_s = this->get_update_interval() / 1000.0f;
  } else {
    dt_s = (now_ms - this->last_update_ms_) / 1000.0f;
  }
  this->last_update_ms_ = now_ms;

  const float ambient_sample = this->grill_probe_->state;
  const float internal_sample = this->meat_probe_->state;

  this->ambient_filter_.update(ambient_sample, dt_s, AB_ALPHA, AB_BETA);
  this->internal_filter_.update(internal_sample, dt_s, AB_ALPHA, AB_BETA);

  const float target_temp = this->get_target_temperature_();
  const float lid_ref = this->get_lid_reference_();
  const bool lid_open = this->is_lid_open_(this->ambient_filter_.value, lid_ref);

  this->update_phase_(this->ambient_filter_.value, this->internal_filter_.value, dt_s, lid_open);
  this->publish_phase_();

  if (this->phase_ == CookPhase::STRETCH && this->stall_penalty_s_ > 0.0f) {
    const float decay = expf(-dt_s / STALL_PENALTY_DECAY_TAU_S);
    this->stall_penalty_s_ *= decay;
  }

  if (this->phase_ == CookPhase::IDLE || this->phase_ == CookPhase::LEARNING || this->phase_ == CookPhase::PAUSE) {
    this->clear_time_entities_();
    return;
  }

  if (this->phase_ == CookPhase::COMPLETE) {
    this->publish_if_changed_(this->finish_time_sensor_, "", &this->last_finish_time_text_);
    this->publish_if_changed_(this->pull_time_sensor_, "", &this->last_pull_time_text_);
    this->publish_if_changed_(this->rest_end_time_sensor_, "", &this->last_rest_end_time_text_);
    if (this->rest_remaining_min_sensor_ != nullptr && this->last_rest_remaining_min_ != 0.0f) {
      this->rest_remaining_min_sensor_->publish_state(0.0f);
      this->last_rest_remaining_min_ = 0.0f;
    }
    return;
  }

  if (this->phase_ == CookPhase::RESTING) {
    const float elapsed_s = (millis() - this->rest_start_ms_) / 1000.0f;
    const float remaining_s = std::max(0.0f, static_cast<float>(this->rest_duration_s_) - elapsed_s);
    if (this->rest_end_time_sensor_ != nullptr) {
      this->publish_if_changed_(this->rest_end_time_sensor_, this->format_clock_from_eta_(remaining_s),
                                &this->last_rest_end_time_text_);
    }
    if (this->rest_remaining_min_sensor_ != nullptr) {
      float remaining_min = roundf(remaining_s / 60.0f);
      if (remaining_min != this->last_rest_remaining_min_) {
        this->rest_remaining_min_sensor_->publish_state(remaining_min);
        this->last_rest_remaining_min_ = remaining_min;
      }
    }
    this->publish_if_changed_(this->finish_time_sensor_, "", &this->last_finish_time_text_);
    this->publish_if_changed_(this->pull_time_sensor_, "", &this->last_pull_time_text_);
    return;
  }

  const float now_s = this->get_session_elapsed_s_();
  if (this->phase_ != CookPhase::STALL) {
    this->push_regression_point_(now_s, this->ambient_filter_.value, this->internal_filter_.value);

    if (this->last_regression_ms_ == 0 || (now_ms - this->last_regression_ms_) >= 60000UL) {
      this->compute_regression_();
      this->last_regression_ms_ = now_ms;
    }
  }

  if (!this->regression_valid_ || this->regression_r2_ < MIN_R2) {
    this->publish_if_changed_(this->finish_time_sensor_, "", &this->last_finish_time_text_);
    this->publish_if_changed_(this->pull_time_sensor_, "", &this->last_pull_time_text_);
    return;
  }

  const float pull_temp = this->compute_pull_temperature_(this->ambient_filter_.value, target_temp);

  float finish_eta_s = 0.0f;
  float pull_eta_s = 0.0f;

  bool finish_ok = this->compute_eta_seconds_(target_temp, now_s, &finish_eta_s);
  bool pull_ok = this->compute_eta_seconds_(pull_temp, now_s, &pull_eta_s);

  if (!finish_ok || !pull_ok) {
    this->publish_if_changed_(this->finish_time_sensor_, "", &this->last_finish_time_text_);
    this->publish_if_changed_(this->pull_time_sensor_, "", &this->last_pull_time_text_);
    return;
  }

  this->publish_if_changed_(this->finish_time_sensor_, this->format_clock_from_eta_(finish_eta_s),
                            &this->last_finish_time_text_);
  this->publish_if_changed_(this->pull_time_sensor_, this->format_clock_from_eta_(pull_eta_s),
                            &this->last_pull_time_text_);
}

}  // namespace grill_thermal_model
}  // namespace esphome
