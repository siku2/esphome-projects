#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>

#include "esphome/components/zeromouse_air/air_sensor.h"

namespace esphome::zeromouse {

struct ScanParams {
  int32_t slope_thresh{20};
  int32_t mean_window{5};
  int32_t range_ignore_event{100};
  int32_t shots_before_t0{0};
  uint16_t ir_free_value{50};
  bool forced{false};
};

struct ScanResult {
  bool threshold_reached{false};
  bool failed{false};
  int64_t time_zero_us{0};
  size_t sample_count{0};
  int32_t min_mean{0};
  int32_t max_mean{0};
};

/// One IR scan of an event. Runs in the scan task.
class Scan {
 public:
  static constexpr size_t SAMPLES = 114;

  using TimeZeroCallback = std::function<void(int64_t time_zero_us)>;

  explicit Scan(const std::atomic<bool> *presence) : presence_(presence) {}

  ScanResult run(zeromouse_air::AirSensor &air, const ScanParams &params, const TimeZeroCallback &on_time_zero);

  /// The ring contents of the last scan, oldest first.
  const std::vector<zeromouse_air::AirSample> &samples() const { return this->chronological_; }

 protected:
  bool is_sunlight_(const ScanResult &result, const ScanParams &params) const;

  const std::atomic<bool> *presence_;
  std::array<zeromouse_air::AirSample, SAMPLES> ring_{};
  std::vector<zeromouse_air::AirSample> chronological_;
};

}  // namespace esphome::zeromouse
