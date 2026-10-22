#ifdef USE_ESP32

#include "zeromouse_cloud.h"

#include <cmath>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "classification.h"
#include "event_payload.h"
#include "shadow_json.h"

namespace esphome::zeromouse_cloud {

static const char *const TAG = "zeromouse_cloud";
static const char *const DEVICE_PREFIX = "MBR-PTF-H1-";
static const uint32_t REPORT_DEBOUNCE_MS = 1000;
static constexpr uint32_t WORKER_STACK_SIZE = 8192;
static constexpr UBaseType_t WORKER_PRIORITY = 1;
static constexpr BaseType_t WORKER_CORE = 0;

namespace {

bool read_binding(const ZeroMouseCloud::Binding &binding, int64_t &value) {
  if (binding.number != nullptr) {
    if (std::isnan(binding.number->state))
      return false;
    value = std::llround(binding.number->state);
    return true;
  }
  if (binding.sw != nullptr) {
    value = binding.sw->state ? 1 : 0;
    return true;
  }
  if (binding.select != nullptr) {
    const auto index = binding.select->active_index();
    if (!index.has_value())
      return false;
    value = static_cast<int64_t>(*index);
    return true;
  }
  return false;
}

/// Converts the value to something the entity accepts. Returns false if it is not applicable.
bool fit_binding(const ZeroMouseCloud::Binding &binding, const char *name, int64_t &value) {
  if (binding.number != nullptr) {
    const auto &traits = binding.number->traits;
    const int64_t min = std::llround(traits.get_min_value());
    const int64_t max = std::llround(traits.get_max_value());
    const int64_t clamped = value < min ? min : (value > max ? max : value);
    if (clamped != value)
      ESP_LOGW(TAG, "Setting %s value %lld is out of range, using %lld", name, static_cast<long long>(value),
               static_cast<long long>(clamped));
    value = clamped;
    return true;
  }
  if (binding.sw != nullptr) {
    value = value != 0 ? 1 : 0;
    return true;
  }
  return binding.select != nullptr && value >= 0 && static_cast<size_t>(value) < binding.select->size();
}

void write_binding(const ZeroMouseCloud::Binding &binding, int64_t value) {
  if (binding.number != nullptr) {
    binding.number->make_call().set_value(static_cast<float>(value)).perform();
  } else if (binding.sw != nullptr) {
    if (value != 0)
      binding.sw->turn_on();
    else
      binding.sw->turn_off();
  } else if (binding.select != nullptr) {
    binding.select->make_call().set_index(static_cast<size_t>(value)).perform();
  }
}

bool to_integer(JsonVariantConst raw, int64_t &value) {
  if (raw.is<bool>()) {
    value = raw.as<bool>() ? 1 : 0;
    return true;
  }
  if (raw.is<double>()) {
    value = std::llround(raw.as<double>());
    return true;
  }
  return false;
}

std::string to_text(JsonVariantConst raw) {
  if (raw.is<const char *>())
    return raw.as<std::string>();
  if (raw.is<bool>())
    return raw.as<bool>() ? "true" : "false";
  std::string out;
  serializeJson(raw, out);
  return out;
}

}  // namespace

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

  this->job_queue_ = xQueueCreate(1, sizeof(Job *));
  this->result_queue_ = xQueueCreate(1, sizeof(bool));
  if (this->job_queue_ == nullptr || this->result_queue_ == nullptr ||
      xTaskCreatePinnedToCore(&ZeroMouseCloud::worker_task_, "zm_event", WORKER_STACK_SIZE, this, WORKER_PRIORITY,
                              &this->worker_, WORKER_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the event worker");
    this->mark_failed();
    return;
  }
  this->disable_loop();

  this->aws_iot_->subscribe(this->device_topic_("command"),
                            [this](const std::string &, const std::string &payload) { this->on_command_(payload); });
  this->aws_iot_->subscribe(this->shadow_topic_("get/accepted"),
                            [this](const std::string &, const std::string &payload) { this->on_shadow_(payload); });
  this->aws_iot_->subscribe(this->shadow_topic_("update/delta"),
                            [this](const std::string &, const std::string &payload) { this->on_delta_(payload); });
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

  for (const Binding &binding : this->bindings_) {
    const Setting setting = binding.setting;
    if (binding.number != nullptr)
      binding.number->add_on_state_callback([this, setting](float) { this->on_binding_state_(setting); });
    else if (binding.sw != nullptr)
      binding.sw->add_on_state_callback([this, setting](bool) { this->on_binding_state_(setting); });
    else if (binding.select != nullptr)
      binding.select->add_on_state_callback([this, setting](size_t) { this->on_binding_state_(setting); });
    int64_t value;
    if (read_binding(binding, value))
      set_setting(this->shadow_, setting, value);
  }
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

void ZeroMouseCloud::schedule_report_() {
  this->set_timeout("report", REPORT_DEBOUNCE_MS, [this]() {
    if (this->aws_iot_->is_connected())
      this->publish_shadow_();
  });
}

void ZeroMouseCloud::on_binding_state_(Setting setting) {
  for (const Binding &binding : this->bindings_) {
    int64_t value;
    if (binding.setting == setting && read_binding(binding, value)) {
      set_setting(this->shadow_, setting, value);
      this->schedule_report_();
      return;
    }
  }
}

void ZeroMouseCloud::on_shadow_(const std::string &payload) {
  const bool valid = json::parse_json(payload, [&](JsonObject root) {
    const int64_t version = root["version"] | static_cast<int64_t>(-1);
    if (version < this->shadow_version_) {
      ESP_LOGD(TAG, "Ignoring stale shadow version %lld", static_cast<long long>(version));
      return true;
    }
    this->shadow_version_ = version;
    JsonObject delta = root["state"]["delta"];
    if (!delta.isNull())
      this->apply_delta_(delta);
    return true;
  });
  if (!valid)
    ESP_LOGW(TAG, "Shadow has invalid JSON");
  this->cancel_timeout("report");
  this->publish_shadow_();
}

void ZeroMouseCloud::on_delta_(const std::string &payload) {
  const bool valid = json::parse_json(payload, [&](JsonObject root) {
    const int64_t version = root["version"] | static_cast<int64_t>(-1);
    if (version <= this->shadow_version_) {
      ESP_LOGD(TAG, "Ignoring old delta version %lld", static_cast<long long>(version));
      return true;
    }
    this->shadow_version_ = version;
    JsonObject state = root["state"];
    if (!state.isNull())
      this->apply_delta_(state);
    return true;
  });
  if (!valid)
    ESP_LOGW(TAG, "Ignoring delta with invalid JSON");
}

void ZeroMouseCloud::apply_delta_(JsonObject state) {
  for (JsonPair section : state) {
    if (!section.value().is<JsonObject>()) {
      ESP_LOGD(TAG, "Ignoring delta entry %s", section.key().c_str());
      continue;
    }
    for (JsonPair entry : section.value().as<JsonObject>())
      this->apply_setting_(section.key().c_str(), entry.key().c_str(), entry.value());
  }
}

void ZeroMouseCloud::apply_setting_(const char *section, const char *key, JsonVariantConst raw) {
  const std::string name = std::string(section) + "." + key;
  const std::string value = to_text(raw);
  const SettingInfo *info = find_setting(section, key);
  const Binding *binding = nullptr;
  if (info != nullptr) {
    for (const Binding &candidate : this->bindings_) {
      if (candidate.setting == info->setting)
        binding = &candidate;
    }
  }
  if (binding == nullptr) {
    ESP_LOGI(TAG, "Setting %s = %s is not supported, ignoring", name.c_str(), value.c_str());
    const std::string previous = info != nullptr ? std::to_string(get_setting(this->shadow_, info->setting)) : "";
    this->setting_change_trigger_.trigger(name, previous, value, false);
    return;
  }

  int64_t current = 0;
  const bool has_current = read_binding(*binding, current);
  const std::string previous = has_current ? std::to_string(current) : "";
  int64_t target;
  if (!to_integer(raw, target) || !fit_binding(*binding, name.c_str(), target)) {
    ESP_LOGW(TAG, "Setting %s = %s is not applicable", name.c_str(), value.c_str());
    this->setting_change_trigger_.trigger(name, previous, value, false);
    return;
  }
  if (has_current && target == current)
    return;
  if (this->accept_settings_ != nullptr && !this->accept_settings_->state) {
    ESP_LOGI(TAG, "Cloud settings are not accepted, ignoring %s = %s", name.c_str(), value.c_str());
    this->setting_change_trigger_.trigger(name, previous, value, false);
    return;
  }
  ESP_LOGI(TAG, "Setting %s: %s -> %s", name.c_str(), previous.c_str(), value.c_str());
  write_binding(*binding, target);
  this->setting_change_trigger_.trigger(name, previous, value, true);
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
  for (const auto &callback : this->classification_callbacks_)
    callback(class_name, index, block);
  this->classification_trigger_.trigger(class_name, index, block);
}

void ZeroMouseCloud::fail_async_(PublishCallback callback) {
  this->defer([callback = std::move(callback)]() {
    if (callback)
      callback(false, 0);
  });
}

void ZeroMouseCloud::publish_event_async(EventData data, PublishCallback callback) {
  if (this->is_failed() || this->building_) {
    ESP_LOGW(TAG, "Event worker is not available, dropping event");
    this->fail_async_(std::move(callback));
    return;
  }
  if (this->upload_events_ != nullptr && !this->upload_events_->state) {
    ESP_LOGI(TAG, "Event upload is off, not publishing");
    this->fail_async_(std::move(callback));
    return;
  }
  if (!this->aws_iot_->is_connected()) {
    ESP_LOGW(TAG, "Not connected, dropping event");
    this->fail_async_(std::move(callback));
    return;
  }

  this->event_count_++;
  this->event_count_pref_.save(&this->event_count_);

  const time_t now = this->time_->timestamp_now();
  const CameraShadow &camera = this->shadow_.camera;
  auto *job = new Job{std::move(data),
                      {this->device_id_,
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
                       this->previous_event_time_,
                       {}},
                      this->device_topic_("events_full_dev")};
  job->info.pir_time_to_time0_ms = job->data.pir_to_t0_ms;
  job->info.air = job->data.air;

  this->pending_event_id_ = this->event_count_;
  this->pending_event_time_ = now;
  this->pending_callback_ = std::move(callback);
  this->building_ = true;
  if (xQueueSend(this->job_queue_, &job, 0) != pdTRUE) {
    delete job;
    this->building_ = false;
    this->fail_async_(std::move(this->pending_callback_));
  }
}

void ZeroMouseCloud::worker_task_(void *arg) {
  auto *that = static_cast<ZeroMouseCloud *>(arg);
  while (true) {
    Job *job;
    xQueueReceive(that->job_queue_, &job, portMAX_DELAY);
    std::unique_ptr<Job> owner(job);
    bool ok = false;
    that->build_and_publish_(*owner, ok);
    owner.reset();
    xQueueSend(that->result_queue_, &ok, portMAX_DELAY);
    that->enable_loop_soon_any_context();
    App.wake_loop_threadsafe();
  }
}

void ZeroMouseCloud::build_and_publish_(Job &job, bool &ok) {
  std::vector<EventImage> images;
  images.reserve(job.data.frames.size());
  for (const auto &frame : job.data.frames) {
    const timeval time = frame->get_timestamp();
    const int64_t us = static_cast<int64_t>(time.tv_sec) * 1000000 + time.tv_usec - job.data.time_zero_us;
    images.push_back({frame->get_data_buffer(), frame->get_data_length(), static_cast<int32_t>(us / 1000)});
  }

  EventPayload payload;
  if (!payload.build(job.info, images))
    return;
  ESP_LOGI(TAG, "Event %u: %u bytes, %u of %u images", static_cast<unsigned>(job.info.event_id),
           static_cast<unsigned>(payload.size()), static_cast<unsigned>(payload.image_count()),
           static_cast<unsigned>(images.size()));
  ok = this->aws_iot_->publish(job.topic, payload.data(), payload.size());
}

void ZeroMouseCloud::loop() {
  bool ok;
  while (xQueueReceive(this->result_queue_, &ok, 0) == pdTRUE) {
    this->building_ = false;
    if (ok) {
      this->has_previous_event_ = true;
      this->previous_event_id_ = this->pending_event_id_;
      this->previous_event_time_ = this->pending_event_time_;
    }
    auto callback = std::move(this->pending_callback_);
    this->pending_callback_ = nullptr;
    if (callback)
      callback(ok, this->pending_event_id_);
  }
  this->disable_loop();
}

void ZeroMouseCloud::publish_test_event(burst_camera::BurstCamera *camera) {
  const bool started = camera->capture_burst(
      TEST_EVENT_FRAMES, [this](std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> frames) {
        if (frames.empty()) {
          ESP_LOGW(TAG, "No frames, dropping test event");
          return;
        }
        const timeval first = frames.front()->get_timestamp();
        const int64_t time_zero_us = static_cast<int64_t>(first.tv_sec) * 1000000 + first.tv_usec;
        this->publish_event_async({std::move(frames), time_zero_us, 0, {}}, [](bool published, uint32_t) {
          if (!published)
            ESP_LOGW(TAG, "Test event was not published");
        });
      });
  if (!started)
    ESP_LOGW(TAG, "Camera is busy or failed, no test event");
}

}  // namespace esphome::zeromouse_cloud

#endif
