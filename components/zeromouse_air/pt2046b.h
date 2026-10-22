#pragma once

#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"

#include "air_sensor.h"

namespace esphome::zeromouse_air {

/// PT2046B photodiode with a switched IR LED.
class Pt2046b final : public AirSensor {
 public:
  void set_adc_pin(uint8_t pin) { this->adc_pin_ = pin; }
  void set_led_pin(uint8_t pin) { this->led_pin_ = pin; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  bool power_on() override;
  void power_off() override;
  void set_one_shot(bool one_shot) override { this->one_shot_ = one_shot; }
  bool read(AirSample &sample) override;

 protected:
  adc_oneshot_unit_handle_t adc_{nullptr};
  adc_channel_t channel_{ADC_CHANNEL_0};
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  uint8_t adc_pin_{0};
  uint8_t led_pin_{0};
  bool one_shot_{true};
};

}  // namespace esphome::zeromouse_air
