#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <string>

#include "esphome/components/switch/switch.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome::zeromouse {

class ZeroMouse;

/// Releases the blocker if the controller keeps it on for too long. Runs apart from the controller.
class ZeroMouseWatchdog : public Component {
 public:
  void set_controller(ZeroMouse *controller) { this->controller_ = controller; }
  void set_blocker(switch_::Switch *blocker) { this->blocker_ = blocker; }
  Trigger<std::string> *get_trigger() { return &this->trigger_; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  void check_();
  void release_(const char *rule, bool event_stuck);

  ZeroMouse *controller_{nullptr};
  switch_::Switch *blocker_{nullptr};
  Trigger<std::string> trigger_;
  int64_t on_since_us_{0};
  int64_t idle_on_since_us_{0};
};

}  // namespace esphome::zeromouse

#endif
