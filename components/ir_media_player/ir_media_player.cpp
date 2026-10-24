#include "ir_media_player.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "ir_scene_select.h"

namespace esphome::ir_media_player {

static const char *const TAG = "ir_media_player";
static const char *const TX_TIMEOUT = "tx";

using media_player::MediaPlayerEntityFeature;

static void log_code(const char *label, const IrCode &code) {
  ESP_LOGCONFIG(TAG, "  %s: address=0x%04X command=0x%04X repeats=%u", label, code.code.address, code.code.command,
                code.repeats);
}

void IrMediaPlayer::setup() {
  this->pref_ = this->make_entity_preference<Saved>();
  Saved saved{};
  if (this->pref_.load(&saved))
    this->state_.restore(saved.on, saved.muted);
  this->state = this->state_.on() ? media_player::MEDIA_PLAYER_STATE_ON : media_player::MEDIA_PLAYER_STATE_OFF;
  this->publish_state();
}

void IrMediaPlayer::dump_config() {
  ESP_LOGCONFIG(TAG, "IR Media Player '%s':", this->get_name().c_str());
  log_code("Power toggle", this->power_toggle_);
  if (this->has_mute_)
    log_code("Mute toggle", this->mute_toggle_);
  if (this->has_volume_) {
    log_code("Volume up", this->volume_up_);
    log_code("Volume down", this->volume_down_);
  }
  ESP_LOGCONFIG(TAG, "  Minimum gap: %" PRIu32 " ms", this->min_gap_ms_);
}

media_player::MediaPlayerTraits IrMediaPlayer::get_traits() {
  media_player::MediaPlayerTraits traits;
  traits.clear_feature_flags(media_player::BASE_MEDIA_PLAYER_FEATURES);
  uint32_t flags = MediaPlayerEntityFeature::TURN_ON | MediaPlayerEntityFeature::TURN_OFF;
  if (this->has_mute_)
    flags |= MediaPlayerEntityFeature::VOLUME_MUTE;
  if (this->has_volume_)
    flags |= MediaPlayerEntityFeature::VOLUME_STEP;
  traits.add_feature_flags(flags);
  return traits;
}

void IrMediaPlayer::control(const media_player::MediaPlayerCall &call) {
  if (!call.get_command().has_value())
    return;
  switch (*call.get_command()) {
    case media_player::MEDIA_PLAYER_COMMAND_TURN_ON:
      this->set_power_(true);
      break;
    case media_player::MEDIA_PLAYER_COMMAND_TURN_OFF:
      this->set_power_(false);
      break;
    case media_player::MEDIA_PLAYER_COMMAND_TOGGLE:
      this->set_power_(!this->state_.on());
      break;
    case media_player::MEDIA_PLAYER_COMMAND_MUTE:
      this->set_mute_(true);
      break;
    case media_player::MEDIA_PLAYER_COMMAND_UNMUTE:
      this->set_mute_(false);
      break;
    case media_player::MEDIA_PLAYER_COMMAND_VOLUME_UP:
      if (this->has_volume_)
        this->enqueue_(this->volume_up_);
      break;
    case media_player::MEDIA_PLAYER_COMMAND_VOLUME_DOWN:
      if (this->has_volume_)
        this->enqueue_(this->volume_down_);
      break;
    default:
      break;
  }
}

void IrMediaPlayer::set_power_(bool on) {
  if (this->state_.request_power(on))
    this->enqueue_(this->power_toggle_);
  this->commit_();
}

void IrMediaPlayer::set_mute_(bool muted) {
  if (!this->has_mute_)
    return;
  if (this->state_.request_mute(muted))
    this->enqueue_(this->mute_toggle_);
  this->commit_();
}

void IrMediaPlayer::correct_power(bool on) {
  this->state_.force_power(on);
  this->commit_();
}

void IrMediaPlayer::correct_mute(bool muted) {
  this->state_.force_mute(muted);
  this->commit_();
}

void IrMediaPlayer::send_scene(const IrCode &scene) {
  this->state_.request_scene();
  this->enqueue_(scene);
  this->commit_();
}

void IrMediaPlayer::commit_() {
  this->state = this->state_.on() ? media_player::MEDIA_PLAYER_STATE_ON : media_player::MEDIA_PLAYER_STATE_OFF;
  Saved saved{this->state_.on(), this->state_.muted()};
  this->pref_.save(&saved);
  this->publish_state();
}

bool IrMediaPlayer::on_receive(remote_base::RemoteReceiveData data) {
  auto decoded = remote_base::NECProtocol().decode(data);
  if (!decoded.has_value())
    return false;
  const ir_toggle::Code code{decoded->address, decoded->command};

  ir_toggle::Input input;
  optional<size_t> scene_index;
  if (code == this->power_toggle_.code) {
    input = ir_toggle::Input::POWER_TOGGLE;
  } else if (this->has_mute_ && code == this->mute_toggle_.code) {
    input = ir_toggle::Input::MUTE_TOGGLE;
  } else if (this->scene_select_ != nullptr && (scene_index = this->scene_select_->find(code)).has_value()) {
    input = ir_toggle::Input::SCENE;
  } else {
    return false;
  }

  auto outcome = this->state_.receive(input, code, millis());
  switch (outcome) {
    case ir_toggle::Outcome::IGNORED:
      ESP_LOGV(TAG, "'%s': ignored 0x%04X 0x%04X", this->get_name().c_str(), code.address, code.command);
      break;
    case ir_toggle::Outcome::SCENE_SELECTED:
      this->scene_select_->apply_received(*scene_index);
      this->commit_();
      break;
    default:
      this->commit_();
      break;
  }
  return false;
}

void IrMediaPlayer::enqueue_(const IrCode &code) {
  this->queue_.push_back(code);
  this->schedule_next_();
}

void IrMediaPlayer::schedule_next_() {
  if (this->tx_scheduled_ || this->queue_.empty())
    return;
  uint32_t delay = 0;
  if (this->has_transmitted_) {
    uint32_t elapsed = millis() - this->last_tx_ms_;
    if (elapsed < this->min_gap_ms_)
      delay = this->min_gap_ms_ - elapsed;
  }
  this->tx_scheduled_ = true;
  this->set_timeout(TX_TIMEOUT, delay, [this]() {
    this->tx_scheduled_ = false;
    this->transmit_next_();
  });
}

void IrMediaPlayer::transmit_next_() {
  if (this->queue_.empty())
    return;
  IrCode next = this->queue_.front();
  this->queue_.pop_front();

  this->last_tx_ms_ = millis();
  this->has_transmitted_ = true;
  this->state_.note_transmitted(next.code, this->last_tx_ms_);

  remote_base::NECData data{next.code.address, next.code.command, next.repeats};
  auto call = this->transmitter_->transmit();
  remote_base::NECProtocol().encode(call.get_data(), data);
  call.perform();

  this->schedule_next_();
}

}  // namespace esphome::ir_media_player
