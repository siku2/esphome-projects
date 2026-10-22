#pragma once

#ifdef USE_ESP32

#include <cstdint>

#include "esphome/components/burst_camera/burst_camera.h"
#include "esphome/components/number/number.h"

namespace esphome::zeromouse {

/// Writes the exposure and gain settings to the camera sensor.
class CameraTuning {
 public:
  number::Number *brightness{nullptr};
  number::Number *auto_exp_level{nullptr};
  number::Number *auto_gain_ctrl_min{nullptr};
  number::Number *auto_gain_ctrl_max{nullptr};
  number::Number *auto_gain_stbl_min{nullptr};
  number::Number *auto_gain_stbl_max{nullptr};
  number::Number *gain_ceiling{nullptr};
  number::Number *cam_shots_before_t0{nullptr};

  /// Applies the settings once the sensor is ready and again on every change.
  void setup(burst_camera::BurstCamera *camera);

 protected:
  using Sensor = burst_camera::SensorKind;

  static int32_t read_(const number::Number *number, int32_t fallback);
  Sensor sensor_() const;
  void on_brightness_(float value);
  void apply_();
  bool apply_ov7670_(int32_t exp_level, int32_t gain_ceiling);
  bool apply_gc032a_(int32_t exp_level, int32_t gain_ceiling);
  bool update_reg_(uint8_t reg, uint8_t mask, uint8_t value);

  burst_camera::BurstCamera *camera_{nullptr};
  bool applying_brightness_{false};
};

}  // namespace esphome::zeromouse

#endif
