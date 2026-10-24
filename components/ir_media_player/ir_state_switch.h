#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

#include "ir_media_player.h"

namespace esphome::ir_media_player {

enum class StateType : uint8_t {
  POWER,
  MUTE,
};

// Shows the stored state of a media player. Writing to it never transmits.
class IrStateSwitch : public switch_::Switch, public Component {
 public:
  void dump_config() override;

  void set_media_player(IrMediaPlayer *player) {
    this->player_ = player;
    player->add_on_state_callback([this](media_player::MediaPlayerState) { this->publish_state(this->current_()); });
  }
  void set_type(StateType type) { this->type_ = type; }

 protected:
  void write_state(bool state) override;
  bool current_() const;

  IrMediaPlayer *player_{nullptr};
  StateType type_{StateType::POWER};
};

}  // namespace esphome::ir_media_player
