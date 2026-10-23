#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <mqtt_client.h>

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "esphome/components/task_util/task_util.h"
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

  /// Runs in the main loop once the sender task is done with a publish. The argument tells if it was written.
  using PublishCallback = std::function<void(bool sent)>;

  /// Publishes with QoS 1 and returns once the message is written to the socket. Never call this from the main loop.
  bool publish_blocking(const std::string &topic, const char *data, size_t len);
  /// Hands a QoS 1 publish to the sender task. The data is copied. Safe to call from any task.
  /// Returns false if the sender queue is full. Otherwise the callback gets the result in the main loop.
  bool publish_async(const std::string &topic, const char *data, size_t len, PublishCallback done = nullptr);
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
    enum class Type : uint8_t { SUBSCRIBING, CONNECTED, DISCONNECTED, MESSAGE, PUBLISHED, STOPPED, START_FAILED } type;
    std::string topic;
    std::string payload;
    PublishCallback done;
    bool sent{false};
  };

  struct Outgoing {
    enum class Kind : uint8_t { PUBLISH, SUBSCRIBE } kind;
    std::string topic;
    std::string payload;
    PublishCallback done;
  };

  static void event_handler_(void *arg, esp_event_base_t base, int32_t event_id, void *event_data);
  static void sender_task_(void *arg);
  static void start_task_(void *arg);
  static void stop_task_(void *arg);
  void handle_mqtt_event_(esp_mqtt_event_handle_t event);
  void push_event_(Event &&event);
  esp_mqtt_client_config_t make_config_(bool clean_session) const;
  void open_session_(esp_mqtt_client_handle_t client);
  void subscribe_all_(esp_mqtt_client_handle_t client);
  void send_(Outgoing &job);
  void start_client_();
  void request_stop_();
  void on_subscribe_timeout_();
  void retry_start_();
  void on_connected_();
  void on_message_(const std::string &topic, const std::string &payload);
  bool has_credentials_() const {
    return !this->client_id_.empty() && !this->certificate_.empty() && !this->private_key_.empty();
  }

  std::string endpoint_;
  uint16_t port_{8883};
  std::string client_id_;
  std::string certificate_;
  std::string private_key_;
  std::string root_ca_;
  std::string username_;
  time::RealTimeClock *time_{nullptr};

  Mutex client_mutex_;
  std::atomic<esp_mqtt_client_handle_t> client_{nullptr};
  std::atomic<bool> connected_{false};
  std::atomic<bool> enabled_{true};
  /// Set while the start or stop task owns the client. Serializes the two.
  std::atomic<bool> busy_{false};
  std::atomic<bool> session_opened_{false};
  bool setup_done_{false};
  bool warned_no_credentials_{false};

  Mutex subscriptions_mutex_;
  std::vector<Subscription> subscriptions_;
  bool subscribed_{false};

  /// Only touched from the MQTT task.
  std::vector<int> pending_subscriptions_;
  std::atomic<bool> awaiting_suback_{false};

  task_util::TaskInbox<std::unique_ptr<Outgoing>> send_queue_{8};
  LazyCallbackManager<void()> connect_callbacks_;
  LazyCallbackManager<void()> disconnect_callbacks_;

  task_util::LoopInbox<Event> events_{this, SIZE_MAX};
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
