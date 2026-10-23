#pragma once

#include <atomic>
#include <cstdint>

#include "esphome/components/zeromouse_air/air_sensor.h"

namespace esphome::zeromouse {

struct FinaleParams {
  uint16_t free_value{50};
  /// False if the last calibration was invalid.
  bool check_drift{true};
  int32_t ambient{1000};
  /// The finale cannot end before this time has passed.
  uint32_t min_duration_ms{0};
  uint32_t max_duration_ms{60000};
};

enum class FinaleResult : uint8_t { STABLE, TIMEOUT };

/// Samples the sensor until the flap area is free and quiet or the time is up. Blocks. Runs in the scan task. Ends
/// early with a timeout if aborted.
FinaleResult run_finale(zeromouse_air::AirSensor &air, const FinaleParams &params, const std::atomic<bool> &presence,
                        const std::atomic<bool> &abort);

}  // namespace esphome::zeromouse
