#ifdef USE_ESP32

#include "zeromouse_cloud.h"

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "classification.h"
#include "event_payload.h"
#include "shadow_json.h"

namespace esphome::zeromouse_cloud {

static const char *const TAG = "zeromouse_cloud";
static const char *const DEVICE_PREFIX = "MBR-PTF-H1-";

void ZeroMouseCloud::set_thing_name(const std::string &thing_name) {
  this->thing_name_ = thing_name;
  const std::string prefix = DEVICE_PREFIX;
  this->device_id_ = thing_name.compare(0, prefix.size(), prefix) == 0 ? thing_name.substr(prefix.size()) : thing_name;
}

void ZeroMouseCloud::set_sw_version(uint8_t major, uint8_t minor, uint8_t revision) {
  this->shadow_.system.ver_major = major;
  this->shadow_.system.ver_minor = minor;
  this->shadow_.system.ver_revision = revision;
  this->sw_version_ = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(revision);
}

void ZeroMouseCloud::setup() {
  this->event_count_pref_ = global_preferences->make_preference<uint32_t>(fnv1_hash("zeromouse_cloud.event_count"));
  this->boot_count_pref_ = global_preferences->make_preference<uint32_t>(fnv1_hash("zeromouse_cloud.boot_count"));

  uint32_t stored;
  if (this->event_count_pref_.load(&stored))
    this->event_count_ = stored;
  if (this->boot_count_pref_.load(&stored))
    this->boot_count_ = stored;
  this->boot_count_++;
  this->boot_count_pref_.save(&this->boot_count_);

  this->aws_iot_->subscribe(this->device_topic_("command"),
                            [this](const std::string &, const std::string &payload) { this->on_command_(payload); });
  this->aws_iot_->subscribe(this->shadow_topic_("get/accepted"), [this](const std::string &, const std::string &) {
    ESP_LOGD(TAG, "Shadow received, ignoring desired and delta");
    this->publish_shadow_();
  });
  this->aws_iot_->subscribe(this->shadow_topic_("get/rejected"),
                            [this](const std::string &, const std::string &payload) {
                              ESP_LOGW(TAG, "Shadow get rejected, reporting a new shadow: %s", payload.c_str());
                              this->publish_shadow_();
                            });
  this->aws_iot_->subscribe(this->shadow_topic_("update/accepted"),
                            [this](const std::string &, const std::string &) { this->publish_metrics_(); });
  this->aws_iot_->subscribe(this->shadow_topic_("update/rejected"),
                            [](const std::string &, const std::string &payload) {
                              ESP_LOGW(TAG, "Shadow update rejected: %s", payload.c_str());
                            });
  this->aws_iot_->add_on_connect_callback([this]() { this->on_connect_(); });
}

void ZeroMouseCloud::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ZeroMOUSE Cloud:\n"
                "  Thing Name: %s\n"
                "  Device ID: %s\n"
                "  Software Version: %s\n"
                "  Event Count: %u\n"
                "  Boot Count: %u",
                this->thing_name_.c_str(), this->device_id_.c_str(), this->sw_version_.c_str(),
                static_cast<unsigned>(this->event_count_), static_cast<unsigned>(this->boot_count_));
}

void ZeroMouseCloud::on_connect_() {
  const std::string topic = this->shadow_topic_("get");
  this->aws_iot_->publish(topic, "{}", 2);
}

void ZeroMouseCloud::publish_shadow_() {
  const std::string payload = build_reported_shadow(this->shadow_);
  this->aws_iot_->publish(this->shadow_topic_("update"), payload.data(), payload.size());
}

void ZeroMouseCloud::publish_metrics_() {
  const std::string payload =
      build_metrics(this->shadow_, {this->device_id_, this->thing_name_, this->boot_count_, this->event_count_});
  this->aws_iot_->publish(this->device_topic_("flaps_dev"), payload.data(), payload.size());
}

void ZeroMouseCloud::on_command_(const std::string &payload) {
  std::string action;
  std::string class_name;
  bool block = false;
  const bool valid = json::parse_json(payload, [&](JsonObject root) {
    action = root["action"] | "";
    class_name = root["class"] | "";
    block = root["block"].is<bool>() && root["block"].as<bool>();
    return true;
  });
  if (!valid) {
    ESP_LOGW(TAG, "Ignoring command with invalid JSON");
    return;
  }
  if (action != "event_classification") {
    ESP_LOGD(TAG, "Ignoring command action '%s'", action.c_str());
    return;
  }
  const int index = class_index(class_name);
  ESP_LOGI(TAG, "Classification: class=%s index=%d block=%d", class_name.c_str(), index, block);
  this->classification_trigger_.trigger(class_name, index, block);
}

bool ZeroMouseCloud::publish_event(const std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> &frames) {
  if (!this->aws_iot_->is_connected()) {
    ESP_LOGW(TAG, "Not connected, dropping event");
    return false;
  }
  if (frames.empty()) {
    ESP_LOGW(TAG, "No frames, dropping event");
    return false;
  }

  this->event_count_++;
  this->event_count_pref_.save(&this->event_count_);

  const time_t now = this->time_->timestamp_now();
  const CameraShadow &camera = this->shadow_.camera;
  EventInfo info{this->device_id_,
                 this->shadow_.system.owner_id,
                 this->shadow_.system.identity_id,
                 this->sw_version_,
                 this->event_count_,
                 now,
                 0,
                 camera.brightness,
                 camera.exposure_ctrl,
                 this->has_previous_event_,
                 this->previous_event_id_,
                 this->previous_event_time_};

  std::vector<EventImage> images;
  const timeval first = frames.front()->get_timestamp();
  for (const auto &frame : frames) {
    const timeval time = frame->get_timestamp();
    const int64_t ms =
        (static_cast<int64_t>(time.tv_sec) - first.tv_sec) * 1000 + (time.tv_usec - first.tv_usec) / 1000;
    images.push_back({frame->get_data_buffer(), frame->get_data_length(), static_cast<uint32_t>(ms < 0 ? 0 : ms)});
  }

  EventPayload payload;
  if (!payload.build(info, images))
    return false;
  ESP_LOGI(TAG, "Event %u: %u bytes, %u of %u images", static_cast<unsigned>(this->event_count_),
           static_cast<unsigned>(payload.size()), static_cast<unsigned>(payload.image_count()),
           static_cast<unsigned>(images.size()));

  if (!this->aws_iot_->publish(this->device_topic_("events_full_dev"), payload.data(), payload.size()))
    return false;
  this->has_previous_event_ = true;
  this->previous_event_id_ = this->event_count_;
  this->previous_event_time_ = now;
  return true;
}

void ZeroMouseCloud::publish_test_event(burst_camera::BurstCamera *camera) {
  const bool started = camera->capture_burst(
      TEST_EVENT_FRAMES,
      [this](std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> frames) { this->publish_event(frames); });
  if (!started)
    ESP_LOGW(TAG, "Camera is busy or failed, no test event");
}

}  // namespace esphome::zeromouse_cloud

#endif
