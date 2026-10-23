#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::axp2101 {

inline constexpr uint8_t REG_STATUS1 = 0x00;
inline constexpr uint8_t REG_CHIP_ID = 0x03;
inline constexpr uint8_t CHIP_ID = 0x4A;

inline constexpr uint8_t STATUS1_CURRENT_LIMIT = 1 << 0;
inline constexpr uint8_t STATUS1_VBUS_GOOD = 1 << 5;

inline constexpr uint8_t LDO_VOLTAGE_MASK = 0x1F;

enum class LdoChannel : uint8_t {
  ALDO1,
  ALDO2,
  ALDO3,
  ALDO4,
  BLDO1,
  BLDO2,
  CPUSLDO,
  DLDO1,
  DLDO2,
};

struct LdoInfo {
  const char *name;
  uint8_t enable_reg;
  uint8_t enable_bit;
  uint8_t voltage_reg;
  uint16_t min_mv;
  uint16_t step_mv;
  uint8_t max_step;
};

inline constexpr LdoInfo LDO_TABLE[] = {
    {"ALDO1", 0x90, 0, 0x92, 500, 100, 30},  {"ALDO2", 0x90, 1, 0x93, 500, 100, 30},
    {"ALDO3", 0x90, 2, 0x94, 500, 100, 30},  {"ALDO4", 0x90, 3, 0x95, 500, 100, 30},
    {"BLDO1", 0x90, 4, 0x96, 500, 100, 30},  {"BLDO2", 0x90, 5, 0x97, 500, 100, 30},
    {"CPUSLDO", 0x90, 6, 0x98, 500, 50, 18}, {"DLDO1", 0x90, 7, 0x99, 500, 100, 29},
    {"DLDO2", 0x91, 0, 0x9A, 500, 100, 29},
};

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::IO; }

  void set_current_limit_binary_sensor(binary_sensor::BinarySensor *sensor) { this->current_limit_sensor_ = sensor; }
  void set_vbus_good_binary_sensor(binary_sensor::BinarySensor *sensor) { this->vbus_good_sensor_ = sensor; }

  bool set_ldo_voltage_step(LdoChannel channel, uint8_t step);
  bool set_ldo_enabled(LdoChannel channel, bool enabled);

 protected:
  binary_sensor::BinarySensor *current_limit_sensor_{nullptr};
  binary_sensor::BinarySensor *vbus_good_sensor_{nullptr};
};

}  // namespace esphome::axp2101
