#pragma once

#ifdef USE_ESP32

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/number/number.h"
#include "esphome/components/zeromouse_air/air_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

namespace esphome::sths34pf80 {

/// STHS34PF80 presence sensor. The interrupt pin raises the state and a register check ends it.
class Sths34pf80 final : public binary_sensor::BinarySensor,
                         public Component,
                         public i2c::I2CDevice,
                         public zeromouse_air::PresenceSampler {
 public:
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }
  void set_motion_threshold(uint16_t threshold) { this->motion_threshold_ = threshold; }
  void set_presence_threshold(uint16_t threshold) { this->presence_threshold_ = threshold; }
  void set_motion_threshold_number(number::Number *number) { this->motion_threshold_number_ = number; }
  void set_presence_threshold_number(number::Number *number) { this->presence_threshold_number_ = number; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  /// Called from a FreeRTOS task, not from the main loop.
  bool read_sample(int16_t &motion, int16_t &presence) override;

 protected:
  static void isr_(Sths34pf80 *self);

  uint16_t motion_threshold_value_() const;
  uint16_t presence_threshold_value_() const;
  void on_threshold_change_();
  void on_hold_expired_();
  void arm_hold_();

  bool read8_(uint8_t reg, uint8_t &value);
  bool write8_(uint8_t reg, uint8_t value);
  bool update8_(uint8_t reg, uint8_t mask, uint8_t value);
  bool read16_(uint8_t reg, int16_t &value);

  bool apply_thresholds_();
  bool reset_();
  bool set_odr_(uint8_t odr);
  bool write_embedded_(uint8_t reg, uint8_t value);
  bool configure_algorithm_(uint16_t motion, uint16_t presence);
  bool configure_interrupt_();

  InternalGPIOPin *interrupt_pin_{nullptr};
  number::Number *motion_threshold_number_{nullptr};
  number::Number *presence_threshold_number_{nullptr};
  SemaphoreHandle_t lock_{nullptr};
  uint16_t motion_threshold_{150};
  uint16_t presence_threshold_{300};
  uint16_t applied_motion_threshold_{0};
  uint16_t applied_presence_threshold_{0};
  bool powered_down_{false};
  volatile bool edge_{false};
};

}  // namespace esphome::sths34pf80

#endif
