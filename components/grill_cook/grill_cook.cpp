#include "grill_cook.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::grill_cook {

static const char *const TAG = "grill_cook";

float GrillCook::sensor_state_or_nan_(sensor::Sensor *sensor) {
  return (sensor != nullptr && sensor->has_state()) ? sensor->state : NAN;
}

void GrillCook::publish_if_changed_(text_sensor::TextSensor *sensor, const std::string &value, std::string *memo) {
  if (sensor == nullptr)
    return;
  if (*memo == value)
    return;
  sensor->publish_state(value);
  *memo = value;
}

bool GrillCook::zone_on(Zone zone) const {
  const auto *climate = this->zones_[static_cast<size_t>(zone)].climate;
  return climate != nullptr && climate->mode != climate::CLIMATE_MODE_OFF;
}

float GrillCook::zone_target(Zone zone) const {
  const auto *climate = this->zones_[static_cast<size_t>(zone)].climate;
  return climate != nullptr ? climate->target_temperature : NAN;
}

float GrillCook::zone_temperature(Zone zone) const {
  return sensor_state_or_nan_(this->zones_[static_cast<size_t>(zone)].probe);
}

float GrillCook::meat_temperature() const { return sensor_state_or_nan_(this->meat_probe_); }

std::string GrillCook::eta_clock() const {
  const int remaining = this->model_.remaining_minutes();
  if (remaining < 0 || this->time_ == nullptr)
    return "";

  const auto now = this->time_->now();
  if (!now.is_valid())
    return "";

  auto done = ESPTime::from_epoch_local(now.timestamp + remaining * 60);
  char buffer[6];
  done.strftime(buffer, sizeof(buffer), "%H:%M");
  return std::string(buffer);
}

void GrillCook::dump_config() {
  ESP_LOGCONFIG(TAG, "Grill Cook:");
  ESP_LOGCONFIG(TAG, "  West climate: %p, probe: %p", static_cast<void *>(this->zones_[0].climate),
                static_cast<void *>(this->zones_[0].probe));
  ESP_LOGCONFIG(TAG, "  East climate: %p, probe: %p", static_cast<void *>(this->zones_[1].climate),
                static_cast<void *>(this->zones_[1].probe));
  ESP_LOGCONFIG(TAG, "  Meat probe: %p", static_cast<void *>(this->meat_probe_));
  LOG_UPDATE_INTERVAL(this);
}

void GrillCook::update() {
  grill_cook_model::CookInputs in{};
  in.t_s = millis() / 1000;

  for (size_t z = 0; z < 2; z++) {
    const auto *climate = this->zones_[z].climate;
    in.zone_on[z] = climate != nullptr && climate->mode != climate::CLIMATE_MODE_OFF;
    in.zone_target[z] = climate != nullptr ? climate->target_temperature : NAN;
    in.zone_temp[z] = sensor_state_or_nan_(this->zones_[z].probe);
  }

  in.meat_temp = sensor_state_or_nan_(this->meat_probe_);
  in.meat_target = (this->meat_target_number_ != nullptr && this->meat_target_number_->has_state())
                       ? this->meat_target_number_->state
                       : NAN;

  this->model_.update(in);

  const bool now_cooking = this->model_.is_cooking();
  const Phase now_phase = this->model_.phase();

  if (now_cooking && !this->last_cooking_)
    this->cook_started_trigger_.trigger();
  if (!now_cooking && this->last_cooking_)
    this->cook_ended_trigger_.trigger();
  if (now_phase != this->last_phase_)
    this->phase_trigger_.trigger(std::string(this->model_.phase_str()));
  this->last_cooking_ = now_cooking;
  this->last_phase_ = now_phase;

  this->publish_if_changed_(this->phase_sensor_, std::string(this->model_.phase_str()), &this->last_phase_text_);
  this->publish_if_changed_(this->eta_sensor_, this->eta_clock(), &this->last_eta_text_);

  const float rate = this->model_.meat_rate();
  const bool rate_changed =
      std::isnan(rate) != std::isnan(this->last_meat_rate_) || (!std::isnan(rate) && rate != this->last_meat_rate_);
  if (this->meat_rate_sensor_ != nullptr && rate_changed) {
    this->meat_rate_sensor_->publish_state(rate);
    this->last_meat_rate_ = rate;
  }

  const int remaining = this->model_.remaining_minutes();
  if (this->remaining_minutes_sensor_ != nullptr && remaining != this->last_remaining_minutes_) {
    this->remaining_minutes_sensor_->publish_state(remaining < 0 ? NAN : static_cast<float>(remaining));
    this->last_remaining_minutes_ = remaining;
  }
}

}  // namespace esphome::grill_cook
