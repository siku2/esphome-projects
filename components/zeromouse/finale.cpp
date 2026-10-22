#include "finale.h"

#include <array>
#include <cstdlib>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome::zeromouse {

using zeromouse_air::AirSample;

static constexpr size_t RING_SIZE = 6;
static constexpr uint32_t SAMPLE_PERIOD_MS = 190;
static constexpr uint32_t STABLE_HOLD_MS = 1000;
static constexpr int32_t VALUE_TOLERANCE = 20;
static constexpr int32_t DRIFT_MIN_FREE_VALUE = 25;
static constexpr int32_t DRIFT_PERCENT = 15;
static constexpr int32_t INVALID_VALUE = 0xFFFF;

FinaleResult run_finale(zeromouse_air::AirSensor &air, const FinaleParams &params, const std::atomic<bool> &presence) {
  std::array<AirSample, RING_SIZE> ring{};
  size_t index = 0;
  size_t filled = 0;
  int64_t stable_since_ms = 0;
  const int64_t start_ms = esp_timer_get_time() / 1000;
  const int32_t free_value = params.free_value;
  const int32_t drift_limit = (free_value > DRIFT_MIN_FREE_VALUE ? free_value : DRIFT_MIN_FREE_VALUE) * DRIFT_PERCENT / 100;

  air.power_on();
  air.set_one_shot(true);
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    AirSample &sample = ring[index];
    if (air.read(sample)) {
      if (sample.value == INVALID_VALUE) {
        sample.value = free_value;
      }
      index = (index + 1) % RING_SIZE;
      filled = filled < RING_SIZE ? filled + 1 : filled;
    }

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const int64_t elapsed_ms = now_ms - start_ms;
    if (filled == RING_SIZE) {
      int32_t sum_value = 0;
      int32_t sum_environment = 0;
      for (const AirSample &s : ring) {
        sum_value += s.value;
        sum_environment += s.environment;
      }
      const int32_t mean_value = sum_value / static_cast<int32_t>(RING_SIZE);
      const int32_t mean_environment = sum_environment / static_cast<int32_t>(RING_SIZE);

      const bool drifted = params.check_drift && std::abs(mean_value - free_value) > drift_limit;
      const bool bright = params.ambient < mean_environment;
      bool unstable = false;
      for (const AirSample &s : ring) {
        unstable |= std::abs(s.value - mean_value) > VALUE_TOLERANCE;
      }
      unstable &= !bright;

      const bool ready = elapsed_ms >= params.min_duration_ms;
      if (ready && !unstable && !drifted && !presence.load()) {
        if (stable_since_ms == 0) {
          stable_since_ms = now_ms;
        } else if (now_ms - stable_since_ms >= STABLE_HOLD_MS) {
          air.power_off();
          return FinaleResult::STABLE;
        }
      } else {
        stable_since_ms = 0;
      }
    }

    if (elapsed_ms >= params.max_duration_ms) {
      air.power_off();
      return FinaleResult::TIMEOUT;
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

}  // namespace esphome::zeromouse
