#pragma once

#ifdef USE_ESP32

#include <memory>
#include <string>
#include <vector>

#include "esphome/components/aws_iot/aws_iot.h"
#include "esphome/components/burst_camera/burst_camera.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "settings.h"
#include "shadow_state.h"

namespace esphome::zeromouse_cloud {

/// The ZeroMOUSE cloud protocol on top of an AWS IoT connection.
class ZeroMouseCloud : public Component {
 public:
  static constexpr uint8_t TEST_EVENT_FRAMES = 8;

  struct Binding {
    Setting setting;
    number::Number *number;
    switch_::Switch *sw;
    select::Select *select;
  };

  void set_aws_iot(aws_iot::AwsIotClient *aws_iot) { this->aws_iot_ = aws_iot; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }
  void set_thing_name(const std::string &thing_name);
  void set_owner_id(const std::string &owner_id) { this->shadow_.system.owner_id = owner_id; }
  void set_identity_id(const std::string &identity_id) { this->shadow_.system.identity_id = identity_id; }
  void set_event_count(uint32_t event_count) { this->event_count_ = event_count; }
  void set_sw_version(uint8_t major, uint8_t minor, uint8_t revision);

  void setup() override;
  void dump_config() override;

  void set_accept_settings(switch_::Switch *accept_settings) { this->accept_settings_ = accept_settings; }
  void set_upload_events(switch_::Switch *upload_events) { this->upload_events_ = upload_events; }
  void bind_number(Setting setting, number::Number *number) { this->bindings_.push_back({setting, number, nullptr, nullptr}); }
  void bind_switch(Setting setting, switch_::Switch *sw) { this->bindings_.push_back({setting, nullptr, sw, nullptr}); }
  void bind_select(Setting setting, select::Select *select) {
    this->bindings_.push_back({setting, nullptr, nullptr, select});
  }

  /// The state that is reported to the cloud.
  ShadowState &shadow_state() { return this->shadow_; }

  /// Publishes an event with the given frames. Returns false if not connected or on failure.
  bool publish_event(const std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> &frames);
  /// Captures a burst and publishes it as an event.
  void publish_test_event(burst_camera::BurstCamera *camera);

  Trigger<std::string, int, bool> *get_classification_trigger() { return &this->classification_trigger_; }
  /// Arguments are the key as "section.jsonKey", the previous value, the requested value and whether it was applied.
  Trigger<std::string, std::string, std::string, bool> *get_setting_change_trigger() {
    return &this->setting_change_trigger_;
  }

 protected:
  void on_connect_();
  void on_shadow_(const std::string &payload);
  void on_delta_(const std::string &payload);
  void apply_delta_(JsonObject state);
  void apply_setting_(const char *section, const char *key, JsonVariantConst raw);
  void on_binding_state_(Setting setting);
  void schedule_report_();
  void on_command_(const std::string &payload);
  void publish_shadow_();
  void publish_metrics_();
  std::string device_topic_(const char *name) const { return "device/" + this->thing_name_ + "/" + name; }
  std::string shadow_topic_(const char *name) const { return "$aws/things/" + this->thing_name_ + "/shadow/" + name; }

  aws_iot::AwsIotClient *aws_iot_{nullptr};
  time::RealTimeClock *time_{nullptr};
  std::string thing_name_;
  std::string device_id_;
  std::string sw_version_{"0.17.12"};
  ShadowState shadow_;
  switch_::Switch *accept_settings_{nullptr};
  switch_::Switch *upload_events_{nullptr};
  std::vector<Binding> bindings_;
  int64_t shadow_version_{-1};

  ESPPreferenceObject event_count_pref_;
  ESPPreferenceObject boot_count_pref_;
  uint32_t event_count_{0};
  uint32_t boot_count_{0};
  bool has_previous_event_{false};
  uint32_t previous_event_id_{0};
  time_t previous_event_time_{0};

  Trigger<std::string, int, bool> classification_trigger_;
  Trigger<std::string, std::string, std::string, bool> setting_change_trigger_;
};

template<typename... Ts> class PublishTestEventAction : public Action<Ts...> {
 public:
  PublishTestEventAction(ZeroMouseCloud *parent, burst_camera::BurstCamera *camera)
      : parent_(parent), camera_(camera) {}

  void play(const Ts &...x) override { this->parent_->publish_test_event(this->camera_); }

 protected:
  ZeroMouseCloud *parent_;
  burst_camera::BurstCamera *camera_;
};

}  // namespace esphome::zeromouse_cloud

#endif
