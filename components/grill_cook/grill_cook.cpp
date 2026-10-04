#include "grill_cook.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::grill_cook {

static const char *const TAG = "grill_cook";

float GrillCook::sensor_state_or_nan_(sensor::Sensor *sensor) {
  return (sensor != nullptr && sensor->has_state()) ? sensor->state : NAN;
}

bool GrillCook::changed_(float value, float *memo) {
  const bool changed = std::isnan(value) != std::isnan(*memo) || (!std::isnan(value) && value != *memo);
  *memo = value;
  return changed;
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
  if (this->model_.phase() == Phase::DONE)
    return "";
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
}

void GrillCook::setup() {
  for (auto &zone : this->zones_) {
    if (zone.climate != nullptr)
      zone.climate->add_on_state_callback([this](climate::Climate &) { this->recompute_(false); });
    if (zone.probe != nullptr)
      zone.probe->add_on_state_callback([this](float) { this->recompute_(false); });
  }
  if (this->meat_probe_ != nullptr)
    this->meat_probe_->add_on_state_callback([this](float) { this->recompute_(true); });
  if (this->meat_target_number_ != nullptr)
    this->meat_target_number_->add_on_state_callback([this](float) { this->recompute_(false); });
}

void GrillCook::recompute_(bool meat_fresh) {
  grill_cook_model::CookInputs in{};
  in.t_s = static_cast<uint32_t>(millis_64() / 1000);
  in.meat_fresh = meat_fresh;

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

  bool presence_changed = false;
  bool control_changed = false;
  bool measurement_changed = false;
  for (size_t z = 0; z < 2; z++) {
    const bool present = this->model_.zone_present(static_cast<Zone>(z));
    presence_changed |= present != this->last_zone_present_[z];
    this->last_zone_present_[z] = present;

    control_changed |= in.zone_on[z] != this->last_zone_on_[z];
    this->last_zone_on_[z] = in.zone_on[z];
    control_changed |= changed_(in.zone_target[z], &this->last_zone_target_[z]);
    measurement_changed |= changed_(in.zone_temp[z], &this->last_zone_temp_[z]);
  }
  control_changed |= changed_(in.meat_target, &this->last_meat_target_);
  measurement_changed |= changed_(in.meat_temp, &this->last_meat_temp_);

  if (presence_changed)
    this->zone_presence_trigger_.trigger();

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

  const std::string phase_text(this->model_.phase_str());
  if (phase_text != this->last_phase_text_) {
    this->last_phase_text_ = phase_text;
    if (this->phase_sensor_ != nullptr)
      this->phase_sensor_->publish_state(phase_text);
  }

  const std::string eta = this->eta_clock();
  if (eta != this->last_eta_text_) {
    this->last_eta_text_ = eta;
    measurement_changed = true;
    if (this->eta_sensor_ != nullptr)
      this->eta_sensor_->publish_state(eta);
  }

  const float rate = std::round(this->model_.meat_rate() * 100.0f) / 100.0f;
  if (changed_(rate, &this->last_meat_rate_)) {
    measurement_changed = true;
    if (this->meat_rate_sensor_ != nullptr)
      this->meat_rate_sensor_->publish_state(rate);
  }

  const int remaining = this->model_.remaining_minutes();
  if (remaining != this->last_remaining_minutes_) {
    this->last_remaining_minutes_ = remaining;
    measurement_changed = true;
    if (this->remaining_minutes_sensor_ != nullptr)
      this->remaining_minutes_sensor_->publish_state(remaining < 0 ? NAN : static_cast<float>(remaining));
  }

  if (control_changed)
    this->control_trigger_.trigger();
  if (measurement_changed)
    this->measurement_trigger_.trigger();

  // The climate callback calls back into this method, so this comes last.
  for (size_t z = 0; z < 2; z++) {
    auto *climate = this->zones_[z].climate;
    if (in.zone_on[z] && !this->last_zone_present_[z] && climate != nullptr) {
      climate->make_call().set_mode(climate::CLIMATE_MODE_OFF).perform();
    }
  }
}

}  // namespace esphome::grill_cook
