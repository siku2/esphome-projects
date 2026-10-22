#pragma once

#ifdef USE_ESP32

#include <mqtt_client.h>

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "message_reassembler.h"

namespace esphome::aws_iot {

/// Called in the main loop.
using MessageCallback = std::function<void(const std::string &topic, const std::string &payload)>;

/// MQTT client for AWS IoT Core with mutual TLS. Independent of the `mqtt` component.
class AwsIotClient : public Component {
 public:
  void set_endpoint(const std::string &endpoint) { this->endpoint_ = endpoint; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_client_id(const std::string &client_id) { this->client_id_ = client_id; }
  void set_certificate(const std::string &certificate) { this->certificate_ = certificate; }
  void set_private_key(const std::string &private_key) { this->private_key_ = private_key; }
  void set_root_ca(const std::string &root_ca) { this->root_ca_ = root_ca; }
  void set_username(const std::string &username) { this->username_ = username; }
  void set_time(time::RealTimeClock *time) { this->time_ = time; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  /// Publishes with QoS 1. The data is copied, so the caller can free it right away.
  /// Large payloads are sent by the MQTT task. Returns false if not connected or out of memory.
  bool publish(const std::string &topic, const char *data, size_t len);
  /// Subscribes with QoS 1 on every connect. The callback runs in the main loop.
  void subscribe(const std::string &topic, MessageCallback callback);

  template<typename F> void add_on_connect_callback(F &&callback) {
    this->connect_callbacks_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_disconnect_callback(F &&callback) {
    this->disconnect_callbacks_.add(std::forward<F>(callback));
  }
  bool is_connected() const { return this->connected_; }

  /// Starts the connection again if it was disabled.
  void enable();
  /// Closes the connection and keeps it closed until enable() is called.
  void disable();
  bool is_enabled() const { return this->enabled_; }

 protected:
  struct Subscription {
    std::string topic;
    MessageCallback callback;
  };

  struct Event {
    enum class Type : uint8_t { CONNECTED, DISCONNECTED, MESSAGE } type;
    std::string topic;
    std::string payload;
  };

  static void event_handler_(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);
  void handle_mqtt_event_(esp_mqtt_event_handle_t event);
  void push_event_(Event &&event);
  void start_client_();
  void on_connected_();
  void on_message_(const std::string &topic, const std::string &payload);

  std::string endpoint_;
  uint16_t port_{8883};
  std::string client_id_;
  std::string certificate_;
  std::string private_key_;
  std::string root_ca_;
  std::string username_;
  time::RealTimeClock *time_{nullptr};

  esp_mqtt_client_handle_t client_{nullptr};
  bool connected_{false};
  bool enabled_{true};
  bool setup_done_{false};
  std::vector<Subscription> subscriptions_;
  LazyCallbackManager<void()> connect_callbacks_;
  LazyCallbackManager<void()> disconnect_callbacks_;

  Mutex events_mutex_;
  std::deque<Event> events_;
  MessageReassembler reassembler_;
};

template<typename... Ts> class EnableAction : public Action<Ts...> {
 public:
  explicit EnableAction(AwsIotClient *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->enable(); }

 protected:
  AwsIotClient *parent_;
};

template<typename... Ts> class DisableAction : public Action<Ts...> {
 public:
  explicit DisableAction(AwsIotClient *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->disable(); }

 protected:
  AwsIotClient *parent_;
};

class ConnectTrigger : public Trigger<> {
 public:
  explicit ConnectTrigger(AwsIotClient *client) {
    client->add_on_connect_callback([this]() { this->trigger(); });
  }
};

class DisconnectTrigger : public Trigger<> {
 public:
  explicit DisconnectTrigger(AwsIotClient *client) {
    client->add_on_disconnect_callback([this]() { this->trigger(); });
  }
};

}  // namespace esphome::aws_iot

#endif
