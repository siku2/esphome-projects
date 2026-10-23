#include "axp2101_output.h"
#include "esphome/core/log.h"

#include <cmath>

namespace esphome::axp2101 {

static const char *const TAG = "axp2101.output";

void AXP2101Output::write_state(float state) {
  if (state <= 0.0f) {
    if (this->enabled_.value_or(true) && this->parent_->set_ldo_enabled(this->channel_, false))
      this->enabled_ = false;
    return;
  }

  const LdoInfo &info = LDO_TABLE[static_cast<size_t>(this->channel_)];
  const uint8_t min_step = (this->min_mv_ - info.min_mv) / info.step_mv;
  const uint8_t max_step = (this->max_mv_ - info.min_mv) / info.step_mv;
  const uint8_t step = min_step + static_cast<uint8_t>(std::lround(clamp(state, 0.0f, 1.0f) * (max_step - min_step)));

  if (this->step_.value_or(0xFF) != step) {
    if (!this->parent_->set_ldo_voltage_step(this->channel_, step)) {
      ESP_LOGW(TAG, "Failed to set %s voltage", info.name);
      return;
    }
    this->step_ = step;
  }
  if (!this->enabled_.value_or(false) && this->parent_->set_ldo_enabled(this->channel_, true))
    this->enabled_ = true;
}

void AXP2101Output::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AXP2101 Output:\n"
                "  Channel: %s\n"
                "  Min voltage: %u mV\n"
                "  Max voltage: %u mV",
                LDO_TABLE[static_cast<size_t>(this->channel_)].name, this->min_mv_, this->max_mv_);
  LOG_FLOAT_OUTPUT(this);
}

}  // namespace esphome::axp2101
