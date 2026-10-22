#include "scan.h"

#include <algorithm>
#include <cstdlib>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome::zeromouse {

using zeromouse_air::AirSample;

static constexpr int32_t SAMPLE_PERIOD_MS = 10;
static constexpr int32_t SCAN_CYCLE_S = 10;
static constexpr int32_t MAX_SCAN_CYCLES = 6;
static constexpr int32_t FORCED_SCAN_CYCLE_S = 60;
static constexpr int32_t FORCED_SCAN_CYCLES = 1;
static constexpr int32_t SCAN_DEADLINE_MS = 720;
static constexpr int32_t MAX_SHOTS = 8;
static constexpr int32_t SHOT_MS = 80;
static constexpr int32_t EXTRA_POST_SAMPLES = 10;
static constexpr int32_t INVALID_VALUE = 0xFFFF;

ScanResult Scan::run(zeromouse_air::AirSensor &air, zeromouse_air::PresenceSampler *sampler, const ScanParams &params,
                     const TimeZeroCallback &on_time_zero) {
  ScanResult result;
  result.min_mean = INVALID_VALUE;
  result.max_mean = 0;
  this->ring_.fill(AirSample{});
  this->chronological_.clear();

  const int32_t window = std::clamp<int32_t>(params.mean_window, 1, SAMPLES);
  const int32_t shots = std::clamp<int32_t>(params.shots_before_t0, 0, MAX_SHOTS);
  const int64_t cycle_us = static_cast<int64_t>(params.forced ? FORCED_SCAN_CYCLE_S : SCAN_CYCLE_S) * 1000000;
  int32_t cycles_left = params.forced ? FORCED_SCAN_CYCLES : MAX_SCAN_CYCLES;

  int64_t scan_start = esp_timer_get_time();
  int32_t post_samples = 0;
  size_t index = 0;
  if (!air.power_on()) {
    result.failed = true;
    return result;
  }

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    if (this->abort_->load()) {
      result.failed = true;
      break;
    }
    AirSample &sample = this->ring_[index];
    if (sampler != nullptr) {
      sample.has_presence = sampler->read_sample(sample.motion, sample.presence);
      if (!sample.has_presence) {
        sample.motion = 0;
        sample.presence = 0;
      }
    }
    if (!air.read(sample)) {
      result.failed = true;
      break;
    }
    result.sample_count++;
    if (sample.value == INVALID_VALUE) {
      sample.value = params.ir_free_value;
    }

    const int32_t used = std::min<int32_t>(window, result.sample_count);
    int32_t sum = 0;
    for (int32_t i = 0; i < used; i++) {
      sum += this->ring_[(index + SAMPLES - i) % SAMPLES].value;
    }
    sample.mean = sum / used;
    result.min_mean = std::min(result.min_mean, sample.mean);
    result.max_mean = std::max(result.max_mean, sample.mean);

    bool triggered = false;
    if (result.sample_count >= static_cast<size_t>(window)) {
      // The stock firmware reads a zeroed slot here when the window is 1 at the first sample.
      sample.slope = result.sample_count > 1 ? sample.mean - this->ring_[(index + SAMPLES - 1) % SAMPLES].mean : 0;
      triggered = std::abs(sample.slope) > params.slope_thresh ||
                  (result.max_mean - result.min_mean) > 3 * params.slope_thresh;
    }

    const int64_t now = esp_timer_get_time();
    if (!result.threshold_reached && triggered) {
      result.threshold_reached = true;
      result.time_zero_us = now;
      if (on_time_zero) {
        on_time_zero(now);
      }
      const int32_t min_post_ms = (MAX_SHOTS - shots) * SHOT_MS;
      const int32_t post_ms = std::max<int32_t>(SCAN_DEADLINE_MS - (now - scan_start) / 1000, min_post_ms);
      post_samples = post_ms / SAMPLE_PERIOD_MS + EXTRA_POST_SAMPLES;
    }

    if (post_samples > 0 && --post_samples == 0) {
      break;
    }

    if (!result.threshold_reached && now - scan_start > cycle_us) {
      if (this->presence_->load() && cycles_left > 0) {
        cycles_left--;
        scan_start = now;
      } else {
        result.time_zero_us = now;
        break;
      }
    }

    index = (index + 1) % SAMPLES;
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }

  air.set_one_shot(true);
  air.power_off();

  const size_t stored = std::min<size_t>(result.sample_count, SAMPLES);
  const size_t oldest = result.sample_count > SAMPLES ? (index + 1) % SAMPLES : 0;
  this->chronological_.reserve(stored);
  for (size_t i = 0; i < stored; i++) {
    this->chronological_.push_back(this->ring_[(oldest + i) % SAMPLES]);
  }

  if (params.forced) {
    result.threshold_reached = true;
  }
  if (result.threshold_reached && this->is_sunlight_(result, params, sampler != nullptr)) {
    result.threshold_reached = false;
  }
  return result;
}

bool Scan::is_sunlight_(const ScanResult &result, const ScanParams &params, bool has_sampler) const {
  if ((result.max_mean - result.min_mean) < params.range_ignore_event) {
    return true;
  }
  if (!has_sampler) {
    return false;
  }
  bool flat = true;
  bool bright = true;
  for (const AirSample &sample : this->chronological_) {
    flat &= !sample.has_presence || sample.presence == sample.motion;
    bright &= sample.environment >= params.ambient;
  }
  return flat && bright;
}

}  // namespace esphome::zeromouse
