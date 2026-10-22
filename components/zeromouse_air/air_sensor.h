#pragma once

#include <cstdint>
#include <functional>

#include "esphome/core/component.h"

namespace esphome::zeromouse_air {

struct AirSample {
  int32_t value{0};
  int32_t environment{0};
  int64_t time_us{0};
  int32_t mean{0};
  int32_t slope{0};
  bool has_presence{false};
  int16_t motion{0};
  int16_t presence{0};
};

/// A presence sensor that delivers one sample per call.
class PresenceSampler {
 public:
  /// Called from a FreeRTOS task, not from the main loop. Returns false if there is no new sample.
  virtual bool read_sample(int16_t &motion, int16_t &presence) = 0;
  /// Registers a callback for every motion edge. Returns false if the sampler has no such edge.
  virtual bool add_on_motion_callback(std::function<void()>) { return false; }
  /// Tells the sampler that an event is running. It should not touch the sensor configuration then.
  virtual void set_busy(bool) {}
};

/// A proximity sensor that is sampled by a dedicated task.
class AirSensor : public Component {
 public:
  void set_calibration_tolerance(uint16_t tolerance) { this->calibration_tolerance_ = tolerance; }
  void set_calibration_max(uint16_t max) { this->calibration_max_ = max; }
  uint16_t calibration_tolerance() const { return this->calibration_tolerance_; }
  uint16_t calibration_max() const { return this->calibration_max_; }

  virtual bool power_on() = 0;
  virtual void power_off() = 0;
  virtual void set_one_shot(bool one_shot) = 0;
  /// Called from a FreeRTOS task, not from the main loop.
  virtual bool read(AirSample &sample) = 0;

 protected:
  uint16_t calibration_tolerance_{20};
  uint16_t calibration_max_{2000};
};

}  // namespace esphome::zeromouse_air
