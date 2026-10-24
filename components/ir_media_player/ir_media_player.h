#pragma once

#include <deque>
#include <vector>

#include "esphome/components/media_player/media_player.h"
#include "esphome/components/remote_base/nec_protocol.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "ir_toggle_state.h"

namespace esphome::ir_media_player {

class IrSceneSelect;

struct IrCode {
  ir_toggle::Code code{};
  uint16_t repeats{1};
};

class IrMediaPlayer : public media_player::MediaPlayer, public Component, public remote_base::RemoteReceiverListener {
 public:
  void setup() override;
  void dump_config() override;

  media_player::MediaPlayerTraits get_traits() override;
  bool is_muted() const override { return this->state_.muted(); }
  bool on_receive(remote_base::RemoteReceiveData data) override;

  void set_transmitter(remote_base::RemoteTransmitterBase *transmitter) { this->transmitter_ = transmitter; }
  void set_echo_window(uint32_t ms) { this->state_.set_echo_window(ms); }
  void set_min_gap(uint32_t ms) { this->min_gap_ms_ = ms; }
  void set_power_toggle(uint16_t address, uint16_t command, uint16_t repeats) {
    this->power_toggle_ = IrCode{{address, command}, repeats};
  }
  void set_mute_toggle(uint16_t address, uint16_t command, uint16_t repeats) {
    this->mute_toggle_ = IrCode{{address, command}, repeats};
    this->has_mute_ = true;
  }
  void set_volume_up(uint16_t address, uint16_t command, uint16_t repeats) {
    this->volume_up_ = IrCode{{address, command}, repeats};
    this->has_volume_ = true;
  }
  void set_volume_down(uint16_t address, uint16_t command, uint16_t repeats) {
    this->volume_down_ = IrCode{{address, command}, repeats};
  }
  void set_scene_select(IrSceneSelect *select) { this->scene_select_ = select; }

  bool powered() const { return this->state_.on(); }
  bool muted() const { return this->state_.muted(); }

  // Changes the stored state only. Nothing is transmitted.
  void correct_power(bool on);
  void correct_mute(bool muted);

  // Transmits the scene code and marks the zone as on without a power toggle.
  void send_scene(const IrCode &scene);

 protected:
  struct Saved {
    bool on;
    bool muted;
  };

  void control(const media_player::MediaPlayerCall &call) override;
  void set_power_(bool on);
  void set_mute_(bool muted);
  void enqueue_(const IrCode &code);
  void schedule_next_();
  void transmit_next_();
  void commit_();

  remote_base::RemoteTransmitterBase *transmitter_{nullptr};
  IrSceneSelect *scene_select_{nullptr};
  ir_toggle::ToggleState state_;
  ESPPreferenceObject pref_;
  IrCode power_toggle_{};
  IrCode mute_toggle_{};
  IrCode volume_up_{};
  IrCode volume_down_{};
  bool has_mute_{false};
  bool has_volume_{false};

  std::deque<IrCode> queue_;
  uint32_t min_gap_ms_{200};
  uint32_t last_tx_ms_{0};
  bool has_transmitted_{false};
  bool tx_scheduled_{false};
};

}  // namespace esphome::ir_media_player
