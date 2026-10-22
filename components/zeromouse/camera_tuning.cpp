#include "camera_tuning.h"

#ifdef USE_ESP32

#include <algorithm>
#include <array>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome::zeromouse {

static const char *const TAG = "zeromouse.camera";

static constexpr uint32_t GC032A_XCLK_HZ = 24000000;
static constexpr int32_t GC032A_SHOTS_BEFORE_T0 = 2;
static constexpr int32_t BRIGHTNESS_MIN = -3;
static constexpr int32_t BRIGHTNESS_MAX = 3;

static constexpr std::array<int32_t, 7> OV7670_ROWS = {44, 48, 52, 56, 60, 64, 64};
static constexpr std::array<int32_t, 7> OV7670_GAIN_CEILING = {2, 2, 2, 2, 2, 3, 4};
static constexpr std::array<int32_t, 7> GC032A_LUMA_TARGET = {12, 20, 28, 36, 44, 52, 60};
static constexpr std::array<int32_t, 7> GC032A_PRE_GAIN = {0x30, 0x30, 0x30, 0x30, 0x40, 0x50, 0x60};

int32_t CameraTuning::read_(const number::Number *number, int32_t fallback) {
  if (number == nullptr || std::isnan(number->state))
    return fallback;
  return static_cast<int32_t>(std::lround(number->state));
}

CameraTuning::Sensor CameraTuning::sensor_() const { return this->camera_->sensor_kind(); }

void CameraTuning::setup(burst_camera::BurstCamera *camera) {
  this->camera_ = camera;
  if (this->brightness != nullptr)
    this->brightness->add_on_state_callback([this](float value) { this->on_brightness_(value); });
  for (auto *number : {this->auto_exp_level, this->auto_gain_ctrl_min, this->auto_gain_ctrl_max,
                       this->auto_gain_stbl_min, this->auto_gain_stbl_max, this->gain_ceiling}) {
    if (number != nullptr)
      number->add_on_state_callback([this](float) { this->apply_(); });
  }
  this->camera_->add_on_sensor_ready([this]() {
    if (this->sensor_() == Sensor::GC032A) {
      if (this->camera_->external_clock_frequency() != GC032A_XCLK_HZ)
        ESP_LOGW(TAG, "The GC032A needs a 24 MHz external clock");
      if (this->cam_shots_before_t0 != nullptr && read_(this->cam_shots_before_t0, 0) != GC032A_SHOTS_BEFORE_T0)
        this->cam_shots_before_t0->make_call().set_value(GC032A_SHOTS_BEFORE_T0).perform();
    }
    this->apply_();
  });
}

void CameraTuning::on_brightness_(float value) {
  if (std::isnan(value) || this->applying_brightness_)
    return;
  const Sensor sensor = this->sensor_();
  if (sensor == Sensor::UNKNOWN)
    return;
  const int32_t level = std::clamp<int32_t>(std::lround(value), BRIGHTNESS_MIN, BRIGHTNESS_MAX) - BRIGHTNESS_MIN;
  const bool ov7670 = sensor == Sensor::OV7670;
  this->applying_brightness_ = true;
  if (this->auto_exp_level != nullptr) {
    this->auto_exp_level->make_call()
        .set_value(ov7670 ? OV7670_ROWS[level] : GC032A_LUMA_TARGET[level])
        .perform();
  }
  if (this->gain_ceiling != nullptr) {
    this->gain_ceiling->make_call()
        .set_value(ov7670 ? OV7670_GAIN_CEILING[level] : GC032A_PRE_GAIN[level])
        .perform();
  }
  this->applying_brightness_ = false;
  this->apply_();
}

bool CameraTuning::update_reg_(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current;
  if (!this->camera_->read_sensor_reg(reg, current))
    return false;
  return this->camera_->write_sensor_reg(reg, (current & ~mask) | (value & mask));
}

bool CameraTuning::apply_ov7670_(int32_t exp_level, int32_t gain_ceiling) {
  const uint32_t rows = exp_level;
  const uint8_t gain_max = read_(this->auto_gain_ctrl_max, 0);
  const uint8_t gain_min = read_(this->auto_gain_ctrl_min, 0);
  const uint8_t stable_max = read_(this->auto_gain_stbl_max, 0);
  const uint8_t stable_min = read_(this->auto_gain_stbl_min, 0);
  auto &cam = *this->camera_;
  bool ok = cam.write_sensor_reg(0x0D, 0x60);
  ok &= cam.write_sensor_reg(0x42, 0x80);
  ok &= this->update_reg_(0x13, 0x01, 0x00);
  ok &= this->update_reg_(0x04, 0x03, rows & 0x03);
  ok &= cam.write_sensor_reg(0x10, (rows >> 2) & 0xFF);
  ok &= this->update_reg_(0x07, 0x3F, (rows >> 10) & 0x3F);
  ok &= cam.write_sensor_reg(0x24, gain_max);
  ok &= cam.write_sensor_reg(0x25, gain_min);
  ok &= cam.write_sensor_reg(0x26, ((stable_min & 0xF0) >> 4) | (stable_max & 0xF0));
  ok &= this->update_reg_(0x14, 0x70, (gain_ceiling & 0x07) << 4);
  return ok;
}

bool CameraTuning::apply_gc032a_(int32_t exp_level, int32_t gain_ceiling) {
  auto &cam = *this->camera_;
  bool ok = cam.write_sensor_reg(0xFE, 0x00);
  ok &= this->update_reg_(0x4F, 0x01, 0x01);
  ok &= cam.write_sensor_reg(0xFE, 0x00);
  ok &= cam.write_sensor_reg(0x03, 0x00);
  ok &= cam.write_sensor_reg(0x04, 0x30);
  ok &= cam.write_sensor_reg(0xFE, 0x01);
  ok &= cam.write_sensor_reg(0x25, 0x00);
  ok &= cam.write_sensor_reg(0x26, 0x41);
  for (uint8_t k = 0; k < 7; k++) {
    ok &= cam.write_sensor_reg(0x27 + 2 * k, 0x00);
    ok &= cam.write_sensor_reg(0x28 + 2 * k, 0x41);
  }
  ok &= cam.write_sensor_reg(0x1F, gain_ceiling & 0xFF);
  ok &= cam.write_sensor_reg(0x13, exp_level & 0xFF);
  ok &= cam.write_sensor_reg(0xFE, 0x00);
  return ok;
}

void CameraTuning::apply_() {
  if (this->camera_ == nullptr || !this->camera_->is_sensor_ready() || this->applying_brightness_)
    return;
  const Sensor sensor = this->sensor_();
  if (sensor == Sensor::UNKNOWN) {
    ESP_LOGW(TAG, "Unsupported camera sensor 0x%04X, not tuning", this->camera_->sensor_pid());
    return;
  }
  const int32_t exp_level = read_(this->auto_exp_level, 0);
  const int32_t gain_ceiling = read_(this->gain_ceiling, 0);
  burst_camera::SensorLock lock(*this->camera_);
  const bool ov7670 = sensor == Sensor::OV7670;
  const bool ok = ov7670 ? this->apply_ov7670_(exp_level, gain_ceiling) : this->apply_gc032a_(exp_level, gain_ceiling);
  ESP_LOGI(TAG,
           "%s: exposure level %d, gain ceiling %d, gain control %d to %d, gain stable %d to %d, brightness %d%s",
           ov7670 ? "OV7670" : "GC032A", static_cast<int>(exp_level), static_cast<int>(gain_ceiling),
           static_cast<int>(read_(this->auto_gain_ctrl_min, 0)), static_cast<int>(read_(this->auto_gain_ctrl_max, 0)),
           static_cast<int>(read_(this->auto_gain_stbl_min, 0)), static_cast<int>(read_(this->auto_gain_stbl_max, 0)),
           static_cast<int>(read_(this->brightness, 0)), ok ? "" : " (some register writes failed)");
}

}  // namespace esphome::zeromouse

#endif
