#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace ir_toggle {

struct Code {
  uint16_t address{0};
  uint16_t command{0};

  bool operator==(const Code &other) const { return this->address == other.address && this->command == other.command; }
};

enum class Input : uint8_t {
  POWER_TOGGLE,
  MUTE_TOGGLE,
  SCENE,
};

enum class Outcome : uint8_t {
  IGNORED,
  POWER_CHANGED,
  MUTE_CHANGED,
  SCENE_SELECTED,
};

// Remembers recently transmitted codes so the receiver can skip its own echo.
class EchoFilter {
 public:
  static constexpr size_t SLOTS = 8;

  void record(const Code &code, uint32_t now_ms) {
    this->codes_[this->next_] = code;
    this->times_[this->next_] = now_ms;
    this->used_[this->next_] = true;
    this->next_ = (this->next_ + 1) % SLOTS;
  }

  bool is_echo(const Code &code, uint32_t now_ms, uint32_t window_ms) const {
    for (size_t i = 0; i < SLOTS; i++) {
      if (this->used_[i] && this->codes_[i] == code && now_ms - this->times_[i] < window_ms)
        return true;
    }
    return false;
  }

 private:
  std::array<Code, SLOTS> codes_{};
  std::array<uint32_t, SLOTS> times_{};
  std::array<bool, SLOTS> used_{};
  size_t next_{0};
};

// Optimistic power and mute state of a device that only understands toggle commands.
class ToggleState {
 public:
  bool on() const { return this->on_; }
  bool muted() const { return this->muted_; }

  void set_echo_window(uint32_t window_ms) { this->echo_window_ms_ = window_ms; }

  void restore(bool on, bool muted) {
    this->on_ = on;
    this->muted_ = muted;
  }

  // Overwrites the stored state without any transmission.
  void force_power(bool on) { this->on_ = on; }
  void force_mute(bool muted) { this->muted_ = muted; }

  // Each request returns true when a toggle must be sent. The state is updated right away.
  bool request_power(bool want_on) {
    if (want_on == this->on_)
      return false;
    this->on_ = want_on;
    return true;
  }
  bool request_mute(bool want_muted) {
    if (want_muted == this->muted_)
      return false;
    this->muted_ = want_muted;
    return true;
  }

  // A scene command turns the device on without a power toggle.
  void request_scene() { this->on_ = true; }

  void note_transmitted(const Code &code, uint32_t now_ms) { this->echo_.record(code, now_ms); }

  // Applies a code heard from the original remote.
  Outcome receive(Input input, const Code &code, uint32_t now_ms) {
    if (this->echo_.is_echo(code, now_ms, this->echo_window_ms_))
      return Outcome::IGNORED;
    switch (input) {
      case Input::POWER_TOGGLE:
        this->on_ = !this->on_;
        return Outcome::POWER_CHANGED;
      case Input::MUTE_TOGGLE:
        if (!this->on_)
          return Outcome::IGNORED;
        this->muted_ = !this->muted_;
        return Outcome::MUTE_CHANGED;
      case Input::SCENE:
        this->on_ = true;
        return Outcome::SCENE_SELECTED;
    }
    return Outcome::IGNORED;
  }

 private:
  bool on_{false};
  bool muted_{false};
  uint32_t echo_window_ms_{1000};
  EchoFilter echo_;
};

}  // namespace ir_toggle
