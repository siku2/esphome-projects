#pragma once

#include <cstdint>

#include "esphome/components/zeromouse_air/air_sensor.h"

namespace esphome::zeromouse {

struct CalibrationResult {
  uint16_t value{0};
  bool valid{false};
};

/// Measures the sensor reading without an object in front of it. Blocks for up to 10 seconds.
CalibrationResult calibrate(zeromouse_air::AirSensor &air, int32_t ambient);

}  // namespace esphome::zeromouse
