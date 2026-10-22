#pragma once

#ifdef USE_ESP32

#include "esphome/components/output/binary_output.h"
#include "esphome/core/gpio.h"

namespace esphome::burst_camera {

/// Power rails, power-down line and illumination of the camera module.
class CameraPower {
 public:
  void set_power_down_pin(InternalGPIOPin *pin) { this->power_down_pin_ = pin; }
  void set_supply_pin(InternalGPIOPin *pin) { this->supply_pin_ = pin; }
  void set_enable_pin(InternalGPIOPin *pin) { this->enable_pin_ = pin; }
  void set_light(output::BinaryOutput *light) { this->light_ = light; }

  /// Cold power-up. Blocks for about 22 ms.
  void power_up();
  /// Returns true if the light changed.
  bool set_light_on(bool on);

  void dump_config();

 protected:
  InternalGPIOPin *power_down_pin_{nullptr};
  InternalGPIOPin *supply_pin_{nullptr};
  InternalGPIOPin *enable_pin_{nullptr};
  output::BinaryOutput *light_{nullptr};
  bool light_on_{false};
};

}  // namespace esphome::burst_camera

#endif
