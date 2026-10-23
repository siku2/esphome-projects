#include "axp2101.h"
#include "esphome/core/log.h"

namespace esphome::axp2101 {

static const char *const TAG = "axp2101";

void AXP2101Component::setup() {
  auto chip_id = this->read_byte(REG_CHIP_ID);
  if (!chip_id.has_value() || *chip_id != CHIP_ID) {
    ESP_LOGE(TAG, "Unexpected chip id 0x%02X", chip_id.value_or(0));
    this->mark_failed();
  }
}

void AXP2101Component::update() {
  if (this->current_limit_sensor_ == nullptr && this->vbus_good_sensor_ == nullptr)
    return;

  auto status = this->read_byte(REG_STATUS1);
  if (!status.has_value()) {
    ESP_LOGW(TAG, "Failed to read PMU status 1");
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();

  if (this->current_limit_sensor_ != nullptr)
    this->current_limit_sensor_->publish_state((*status & STATUS1_CURRENT_LIMIT) != 0);
  if (this->vbus_good_sensor_ != nullptr)
    this->vbus_good_sensor_->publish_state((*status & STATUS1_VBUS_GOOD) != 0);
}

void AXP2101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP2101:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Communication failed");
}

bool AXP2101Component::set_ldo_voltage_step(LdoChannel channel, uint8_t step) {
  const LdoInfo &info = LDO_TABLE[static_cast<size_t>(channel)];
  auto value = this->read_byte(info.voltage_reg);
  if (!value.has_value())
    return false;
  uint8_t new_value = (*value & ~LDO_VOLTAGE_MASK) | (step & LDO_VOLTAGE_MASK);
  return this->write_byte(info.voltage_reg, new_value);
}

bool AXP2101Component::set_ldo_enabled(LdoChannel channel, bool enabled) {
  const LdoInfo &info = LDO_TABLE[static_cast<size_t>(channel)];
  auto value = this->read_byte(info.enable_reg);
  if (!value.has_value())
    return false;
  uint8_t bit = 1 << info.enable_bit;
  uint8_t new_value = enabled ? (*value | bit) : (*value & ~bit);
  return this->write_byte(info.enable_reg, new_value);
}

}  // namespace esphome::axp2101
