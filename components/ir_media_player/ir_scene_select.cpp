#include "ir_scene_select.h"

#include "esphome/core/log.h"

namespace esphome::ir_media_player {

static const char *const TAG = "ir_media_player.select";

void IrSceneSelect::setup() {
  this->pref_ = this->make_entity_preference<uint32_t>();
  uint32_t index = 0;
  if (this->pref_.load(&index) && index < this->scenes_.size())
    this->publish_state(static_cast<size_t>(index));
}

void IrSceneSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "IR Scene Select '%s':", this->get_name().c_str());
  for (size_t i = 0; i < this->scenes_.size(); i++) {
    ESP_LOGCONFIG(TAG, "  %s: address=0x%04X command=0x%04X", this->option_at(i), this->scenes_[i].code.address,
                  this->scenes_[i].code.command);
  }
}

optional<size_t> IrSceneSelect::find(const ir_toggle::Code &code) const {
  for (size_t i = 0; i < this->scenes_.size(); i++) {
    if (this->scenes_[i].code == code)
      return i;
  }
  return {};
}

void IrSceneSelect::apply_received(size_t index) {
  this->publish_state(index);
  this->store_(index);
}

void IrSceneSelect::control(size_t index) {
  if (index >= this->scenes_.size())
    return;
  this->player_->send_scene(this->scenes_[index]);
  this->publish_state(index);
  this->store_(index);
}

void IrSceneSelect::store_(size_t index) {
  uint32_t value = static_cast<uint32_t>(index);
  this->pref_.save(&value);
}

}  // namespace esphome::ir_media_player
