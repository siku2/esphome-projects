#include "calibration.h"

#include <array>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome::zeromouse {

using zeromouse_air::AirSample;

static constexpr size_t RING_SIZE = 10;
static constexpr int MIN_ROUNDS = 5;
static constexpr int MAX_ROUNDS = 50;
static constexpr uint32_t ROUND_DELAY_MS = 200;

CalibrationResult calibrate(zeromouse_air::AirSensor &air, int32_t ambient) {
  std::array<AirSample, RING_SIZE> ring{};
  size_t index = 0;
  size_t filled = 0;
  int32_t mean_value = 0;
  int32_t last_environment = 0;
  int rounds = 0;

  air.power_on();
  for (;; rounds++) {
    AirSample &sample = ring[index];
    if (air.read(sample)) {
      last_environment = sample.environment;
      index = (index + 1) % RING_SIZE;
      filled = filled < RING_SIZE ? filled + 1 : filled;
    }

    int32_t sum = 0;
    for (size_t i = 0; i < filled; i++) {
      sum += ring[i].value;
    }
    mean_value = filled > 0 ? sum / static_cast<int32_t>(filled) : 0;

    if (rounds == MAX_ROUNDS) {
      break;
    }
    if (rounds >= MIN_ROUNDS && mean_value != 0) {
      bool unstable = false;
      for (size_t i = 0; i < filled; i++) {
        const bool deviates = std::abs(ring[i].value - mean_value) > air.calibration_tolerance();
        unstable |= deviates && !(ambient < ring[i].environment);
      }
      if (!unstable) {
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(ROUND_DELAY_MS));
  }
  air.power_off();

  CalibrationResult result;
  result.value = static_cast<uint16_t>(mean_value);
  const bool dark_zero = mean_value == 0 && !(ambient < last_environment);
  result.valid = !dark_zero && mean_value <= air.calibration_max() && rounds != MAX_ROUNDS;
  return result;
}

}  // namespace esphome::zeromouse
