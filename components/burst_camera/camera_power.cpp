#ifdef USE_ESP32

#include "camera_power.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::burst_camera {

static const char *const TAG = "burst_camera";
static constexpr uint32_t STEP_DELAY_MS = 4;
static constexpr uint32_t WAKE_DELAY_MS = 10;

static void drive(InternalGPIOPin *pin, bool on) {
  if (pin != nullptr)
    pin->digital_write(on);
}

void CameraPower::power_up() {
  for (auto *pin : {this->power_down_pin_, this->supply_pin_, this->enable_pin_}) {
    if (pin != nullptr) {
      pin->setup();
      pin->pin_mode(gpio::FLAG_OUTPUT);
    }
  }

  drive(this->power_down_pin_, true);
  delay(STEP_DELAY_MS);
  drive(this->enable_pin_, true);
  delay(STEP_DELAY_MS);
  drive(this->supply_pin_, true);
  delay(STEP_DELAY_MS);
  drive(this->power_down_pin_, false);
  delay(WAKE_DELAY_MS);
}

bool CameraPower::set_light_on(bool on) {
  if (this->light_ == nullptr || this->light_on_ == on)
    return false;
  this->light_on_ = on;
  if (on) {
    this->light_->turn_on();
  } else {
    this->light_->turn_off();
  }
  return true;
}

void CameraPower::dump_config() {
  LOG_PIN("  Power Down Pin: ", this->power_down_pin_);
  LOG_PIN("  Supply Pin: ", this->supply_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  ESP_LOGCONFIG(TAG, "  Light: %s", YESNO(this->light_ != nullptr));
}

}  // namespace esphome::burst_camera

#endif
