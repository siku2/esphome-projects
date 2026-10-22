#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ZEROMOUSE_AIR_VCNL4040

#include "esphome/components/i2c/i2c.h"

#include "air_sensor.h"

namespace esphome::zeromouse_air {

/// VCNL4040 proximity and ambient light sensor. Every I2C transaction is atomic in the ESP-IDF driver.
class Vcnl4040 final : public AirSensor, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  bool power_on() override;
  void power_off() override;
  void set_one_shot(bool one_shot) override;
  bool read(AirSample &sample) override;

 protected:
  enum class Byte : bool { LOW_BYTE, HIGH_BYTE };

  bool read_word_(uint8_t reg, uint16_t &value);
  bool update_(uint8_t reg, Byte byte, uint8_t mask, uint8_t value);

  bool powered_on_{false};
  bool one_shot_{false};
};

}  // namespace esphome::zeromouse_air

#endif
