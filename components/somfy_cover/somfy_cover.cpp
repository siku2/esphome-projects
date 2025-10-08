#include "somfy_cover.h"

#include "esphome/core/log.h"

// See: <https://github.com/Legion2/Somfy_Remote_Lib>

#define SYMBOL 640

namespace esphome {
namespace somfy_cover {
using namespace esphome::cover;
using namespace esphome::cc1101;

static const char *const TAG = "somfy_cover";

static const uint32_t RESTORE_STATE_VERSION = 0xB2D7C9D7UL;

void SomfyCover::dump_config() {
  LOG_COVER("", "Somfy Cover", this);
  ESP_LOGCONFIG(TAG, "  CC1101: %p", static_cast<void *>(this->cc1101_));
  ESP_LOGCONFIG(TAG, "  Cover ID: %s", this->cover_id_);
  ESP_LOGCONFIG(TAG, "  Remote Code: 0x%08X", this->remote_code_);
  ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
}

void SomfyCover::setup() {
  this->rolling_code_pref_ =
      global_preferences->make_preference<uint16_t>(this->get_preference_hash() ^ RESTORE_STATE_VERSION);

  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;
  }
}

void SomfyCover::loop() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  const uint32_t now = millis();

  // Recompute position every loop cycle
  this->recompute_position_();

  if (this->is_at_target_()) {
    if (this->target_position_ == COVER_OPEN || this->target_position_ == COVER_CLOSED) {
      // Don't trigger stop, let the cover stop by itself.
      this->current_operation = COVER_OPERATION_IDLE;
    } else {
      this->start_direction_(COVER_OPERATION_IDLE);
    }
    this->publish_state();
  }

  // Send current position every second
  if (now - this->last_publish_time_ > 1000) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

CoverTraits SomfyCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(true);
  return traits;
}

void SomfyCover::control(const CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(COVER_OPERATION_IDLE);
    this->publish_state();
  }
  if (call.get_toggle().has_value()) {
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->start_direction_(COVER_OPERATION_IDLE);
      this->publish_state();
    } else {
      if (this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_OPEN;
        this->start_direction_(COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = COVER_CLOSED;
        this->start_direction_(COVER_OPERATION_CLOSING);
      }
    }
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos == this->position) {
      // already at target
      // for covers with built in end stop, we should send the command again
      if (pos == COVER_OPEN || pos == COVER_CLOSED) {
        auto op = pos == COVER_CLOSED ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
        this->target_position_ = pos;
        this->start_direction_(op);
      }
    } else {
      auto op = pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
      this->target_position_ = pos;
      this->start_direction_(op);
    }
  }
}

bool SomfyCover::is_at_target_() const {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void SomfyCover::start_direction_(CoverOperation dir) {
  if (dir == this->current_operation && dir != COVER_OPERATION_IDLE)
    return;

  this->recompute_position_();
  switch (dir) {
    case COVER_OPERATION_IDLE:
      this->send_command_(SomfyCommand::My);
      break;
    case COVER_OPERATION_OPENING:
      this->last_operation_ = dir;
      this->send_command_(SomfyCommand::Up);
      break;
    case COVER_OPERATION_CLOSING:
      this->last_operation_ = dir;
      this->send_command_(SomfyCommand::Down);
      break;
    default:
      return;
  }

  this->current_operation = dir;

  const uint32_t now = millis();
  this->start_dir_time_ = now;
  this->last_recompute_time_ = now;
}

void SomfyCover::recompute_position_() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  float dir;
  float action_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  this->position += dir * (now - this->last_recompute_time_) / action_dur;
  this->position = clamp(this->position, 0.0f, 1.0f);

  this->last_recompute_time_ = now;
}

void SomfyCover::program() { this->send_command_(SomfyCommand::Prog); }

void SomfyCover::build_frame_(SomfyCommand command, uint16_t rolling_code, std::array<uint8_t, 7> &frame) {
  frame[0] = 0xA7;                                // Encryption key. Doesn't matter much
  frame[1] = static_cast<uint8_t>(command) << 4;  // Which button did you press? The 4 LSB will be the checksum
  frame[2] = rolling_code >> 8;                   // Rolling code (big endian)
  frame[3] = rolling_code & 0xFF;                 // Rolling code
  frame[4] = (this->remote_code_ >> 16) & 0xFF;   // Remote address
  frame[5] = (this->remote_code_ >> 8) & 0xFF;    // Remote address
  frame[6] = this->remote_code_ & 0xFF;           // Remote address

  // Checksum calculation: a XOR of all the nibbles
  uint8_t checksum = 0;
  for (size_t i = 0; i < frame.size(); i += 1) {
    checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
  }
  checksum &= 0b1111;  // We keep the last 4 bits only

  // Checksum integration
  frame[1] |= checksum;

  ESP_LOGD(TAG, "Built Frame command=%d rolling_code=%d: %02X %02X %02X %02X %02X %02X %02X", command, rolling_code,
           frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6]);

  // Obfuscation: a XOR of all the bytes
  for (size_t i = 1; i < frame.size(); i += 1) {
    frame[i] ^= frame[i - 1];
  }
}

void SomfyCover::send_frame_(const std::array<uint8_t, 7> &frame, uint8_t sync) {
  if (sync == 2) {  // Only with the first frame.
    // Wake-up pulse & Silence
    this->cc1101_->emit_pulse(true, 9415, 9565);
    delay(80);
  }

  // Hardware sync: two sync for the first frame, seven for the following ones.
  for (uint8_t i = 0; i < sync; i += 1) {
    this->cc1101_->emit_pulse(true, 4 * SYMBOL, 4 * SYMBOL);
  }

  // Software sync
  this->cc1101_->emit_pulse(true, 4550, SYMBOL);

  // Data: bits are sent one by one, starting with the MSB.
  for (size_t i = 0; i < 8 * frame.size(); i += 1) {
    uint8_t byte = frame[i / 8];
    bool bit = ((byte >> (7 - (i % 8))) & 1) == 1;
    this->cc1101_->emit_pulse(!bit, SYMBOL, SYMBOL);
  }

  // Inter-frame silence
  this->cc1101_->get_emitter_pin().digital_write(false);
  delayMicroseconds(30415);  // Originally `delayMicroseconds(415); delay(30);`
}

void SomfyCover::send_command_(SomfyCommand command, size_t repeat) {
  std::array<uint8_t, 7> frame;
  this->build_frame_(command, this->get_next_rolling_code_(), frame);

  this->cc1101_->enable_tx();
  this->send_frame_(frame, 2);
  for (size_t i = 0; i < repeat; i++) {
    this->send_frame_(frame, 7);
  }
  this->cc1101_->disable_tx();
}

void SomfyCover::reset_rolling_code() {
  uint16_t code = 1;
  this->rolling_code_pref_.save(&code);
}

uint16_t SomfyCover::get_next_rolling_code_() {
  uint16_t code = 1;
  this->rolling_code_pref_.load(&code);
  uint16_t next_code = code + 1;
  this->rolling_code_pref_.save(&next_code);
  return code;
}

}  // namespace somfy_cover
}  // namespace esphome
