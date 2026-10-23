#pragma once

#include "axp2101.h"
#include "esphome/components/output/float_output.h"

namespace esphome::axp2101 {

class AXP2101Output : public output::FloatOutput, public Component {
 public:
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(AXP2101Component *parent) { this->parent_ = parent; }
  void set_channel(LdoChannel channel) { this->channel_ = channel; }
  void set_min_voltage(uint16_t mv) { this->min_mv_ = mv; }
  void set_max_voltage(uint16_t mv) { this->max_mv_ = mv; }

 protected:
  void write_state(float state) override;

  AXP2101Component *parent_;
  LdoChannel channel_;
  uint16_t min_mv_;
  uint16_t max_mv_;
  optional<uint8_t> step_;
  optional<bool> enabled_;
};

}  // namespace esphome::axp2101
