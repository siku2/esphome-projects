#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/preferences.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/cc1101/cc1101.h"

namespace esphome {
namespace somfy_cover {

enum class SomfyCommand : uint8_t {
  My = 0x1,
  Up = 0x2,
  MyUp = 0x3,
  Down = 0x4,
  MyDown = 0x5,
  UpDown = 0x6,
  Prog = 0x8,
  SunFlag = 0x9,
  Flag = 0xA
};

class SomfyCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  void loop() override;

  void set_cc1101(cc1101::Cc1101 *cc1101) { this->cc1101_ = cc1101; }
  void set_cover_id(const char *cover_id) { this->cover_id_ = cover_id; }
  void set_remote_code(uint32_t remote_code) { this->remote_code_ = remote_code; }
  void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
  void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }

  void program();

 protected:
  cc1101::Cc1101 *cc1101_;
  const char *cover_id_;
  uint32_t remote_code_;
  uint32_t open_duration_;
  uint32_t close_duration_;

  ESPPreferenceObject rolling_code_pref_;
  uint32_t last_recompute_time_{0};
  uint32_t start_dir_time_{0};
  uint32_t last_publish_time_{0};
  float target_position_{0};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};

  void control(const cover::CoverCall &call) override;

  bool is_at_target_() const;

  void start_direction_(cover::CoverOperation dir);
  void recompute_position_();

  void build_frame_(SomfyCommand command, uint16_t rolling_code, std::array<uint8_t, 7> &frame);
  void send_frame_(const std::array<uint8_t, 7> &frame, uint8_t sync);
  void send_value_(bool value, uint32_t micros);

  void send_command_(SomfyCommand command, size_t repeat = 4);
  uint16_t get_next_rolling_code_();
};

template<typename... Ts> class SomfyCoverProgramAction : public Action<Ts...> {
 public:
  SomfyCoverProgramAction(SomfyCover *parent) : parent_(parent) {}

  void play(Ts... x) { this->parent_->program(); }

 protected:
  SomfyCover *parent_;
};
}  // namespace somfy_cover
}  // namespace esphome
