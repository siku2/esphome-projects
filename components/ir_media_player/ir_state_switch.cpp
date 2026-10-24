#include "ir_state_switch.h"

#include "esphome/core/log.h"

namespace esphome::ir_media_player {

static const char *const TAG = "ir_media_player.switch";

void IrStateSwitch::dump_config() {
  LOG_SWITCH("", "IR State Switch", this);
  ESP_LOGCONFIG(TAG, "  Type: %s", this->type_ == StateType::POWER ? "power" : "mute");
}

bool IrStateSwitch::current_() const {
  return this->type_ == StateType::POWER ? this->player_->powered() : this->player_->muted();
}

void IrStateSwitch::write_state(bool state) {
  if (this->type_ == StateType::POWER) {
    this->player_->correct_power(state);
  } else {
    this->player_->correct_mute(state);
  }
}

}  // namespace esphome::ir_media_player
