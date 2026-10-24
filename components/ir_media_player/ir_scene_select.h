#pragma once

#include <vector>

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "ir_media_player.h"

namespace esphome::ir_media_player {

class IrSceneSelect : public select::Select, public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_media_player(IrMediaPlayer *player) { this->player_ = player; }
  void add_scene(uint16_t address, uint16_t command, uint16_t repeats) {
    this->scenes_.push_back(IrCode{{address, command}, repeats});
  }

  optional<size_t> find(const ir_toggle::Code &code) const;
  // Called by the media player when the original remote picked a scene.
  void apply_received(size_t index);

 protected:
  void control(size_t index) override;
  void store_(size_t index);

  IrMediaPlayer *player_{nullptr};
  std::vector<IrCode> scenes_;
  ESPPreferenceObject pref_;
};

}  // namespace esphome::ir_media_player
