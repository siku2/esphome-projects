#pragma once

#include "driver/rmt_tx.h"

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome::fdxb_emulator {

class FdxbEmulator final : public switch_::Switch, public Component {
 public:
  void set_pin(uint8_t pin) { this->pin_ = pin; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void start();
  void stop();

 protected:
  static constexpr size_t ITEM_COUNT = 129;

  void write_state(bool state) override;
  void encode_bit_(rmt_symbol_word_t *item, bool one, bool *level);
  void build_frame_(uint16_t country_code, uint64_t national_id);

  rmt_symbol_word_t items_[ITEM_COUNT];
  rmt_channel_handle_t channel_{nullptr};
  rmt_encoder_handle_t encoder_{nullptr};
  uint8_t pin_{0};
  bool running_{false};
};

}  // namespace esphome::fdxb_emulator
