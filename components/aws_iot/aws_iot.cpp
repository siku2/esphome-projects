#ifdef USE_ESP32

#include "aws_iot.h"

#include <algorithm>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "root_ca.h"

namespace esphome::aws_iot {

static const char *const TAG = "aws_iot";
static constexpr int KEEPALIVE_S = 60;
static constexpr int BUFFER_SIZE = 8 * 1024;
static constexpr int TASK_STACK_SIZE = 8 * 1024;
// esp-mqtt resends an unacknowledged message after this time. The default of 1 s is too short for large events.
static constexpr int RETRANSMIT_TIMEOUT_MS = 15000;
static constexpr int QOS = 1;

// Matches a topic against a filter that may contain + and # wildcards.
static bool topic_matches(const std::string &filter, const std::string &topic) {
  size_t f = 0;
  size_t t = 0;
  while (true) {
    const size_t f_end = std::min(filter.find('/', f), filter.size());
    const size_t t_end = std::min(topic.find('/', t), topic.size());
    const std::string level = filter.substr(f, f_end - f);
    if (level == "#")
      return true;
    if (level != "+" && level != topic.substr(t, t_end - t))
      return false;
    if (f_end == filter.size() || t_end == topic.size())
      return f_end == filter.size() && t_end == topic.size();
    f = f_end + 1;
    t = t_end + 1;
  }
}

void AwsIotClient::setup() {
  if (this->root_ca_.empty())
    this->root_ca_ = AMAZON_ROOT_CA_1;

  this->time_->add_on_time_sync_callback([this]() { this->start_client_(); });
  if (this->time_->utcnow().is_valid())
    this->start_client_();
  this->setup_done_ = true;
  this->disable_loop();
}

void AwsIotClient::enable() {
  this->enabled_ = true;
  ESP_LOGI(TAG, "Enabled");
  if (this->setup_done_ && this->time_->utcnow().is_valid())
    this->start_client_();
}

void AwsIotClient::disable() {
  this->enabled_ = false;
  {
    LockGuard guard(this->client_mutex_);
    if (this->client_ != nullptr) {
      esp_mqtt_client_stop(this->client_);
      esp_mqtt_client_destroy(this->client_);
      this->client_ = nullptr;
    }
  }
  {
    LockGuard guard(this->events_mutex_);
    this->events_.clear();
  }
  this->reassembler_.reset();
  if (this->connected_) {
    this->connected_ = false;
    ESP_LOGW(TAG, "Disconnected");
    this->disconnect_callbacks_.call();
  }
  ESP_LOGI(TAG, "Disabled");
}

void AwsIotClient::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AWS IoT:\n"
                "  Endpoint: %s:%u\n"
                "  Client ID: %s\n"
                "  Username: %s\n"
                "  Enabled: %s",
                this->endpoint_.c_str(), this->port_, this->client_id_.c_str(), this->username_.c_str(),
                YESNO(this->enabled_));
}

void AwsIotClient::start_client_() {
  if (!this->enabled_ || this->client_ != nullptr)
    return;

  esp_mqtt_client_config_t config{};
  config.broker.address.hostname = this->endpoint_.c_str();
  config.broker.address.port = this->port_;
  config.broker.address.transport = MQTT_TRANSPORT_OVER_SSL;
  config.broker.verification.certificate = this->root_ca_.c_str();
  config.credentials.client_id = this->client_id_.c_str();
  if (!this->username_.empty())
    config.credentials.username = this->username_.c_str();
  config.credentials.authentication.certificate = this->certificate_.c_str();
  config.credentials.authentication.key = this->private_key_.c_str();
  config.session.keepalive = KEEPALIVE_S;
  config.session.disable_clean_session = false;
  config.session.message_retransmit_timeout = RETRANSMIT_TIMEOUT_MS;
  config.buffer.size = BUFFER_SIZE;
  config.buffer.out_size = BUFFER_SIZE;
  config.task.stack_size = TASK_STACK_SIZE;

  this->client_ = esp_mqtt_client_init(&config);
  if (this->client_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize the MQTT client");
    this->mark_failed();
    return;
  }
  esp_mqtt_client_register_event(this->client_, MQTT_EVENT_ANY, &AwsIotClient::event_handler_, this);
  const esp_err_t err = esp_mqtt_client_start(this->client_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start the MQTT client: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Connecting to %s", this->endpoint_.c_str());
}

bool AwsIotClient::publish(const std::string &topic, const char *data, size_t len) {
  if (!this->connected_) {
    ESP_LOGW(TAG, "Not connected, dropping publish on %s", topic.c_str());
    return false;
  }
  LockGuard guard(this->client_mutex_);
  if (this->client_ == nullptr) {
    return false;
  }
  // Enqueue copies the message into the outbox and lets the MQTT task send it.
  const int id = esp_mqtt_client_enqueue(this->client_, topic.c_str(), data, static_cast<int>(len), QOS, 0, true);
  if (id < 0) {
    ESP_LOGE(TAG, "Failed to enqueue %u bytes on %s", static_cast<unsigned>(len), topic.c_str());
    return false;
  }
  ESP_LOGD(TAG, "Queued %u bytes on %s", static_cast<unsigned>(len), topic.c_str());
  return true;
}

void AwsIotClient::subscribe(const std::string &topic, MessageCallback callback) {
  this->subscriptions_.push_back({topic, std::move(callback)});
  if (this->connected_ && esp_mqtt_client_subscribe_single(this->client_, topic.c_str(), QOS) < 0)
    ESP_LOGW(TAG, "Failed to subscribe to %s", topic.c_str());
}

void AwsIotClient::loop() {
  while (true) {
    Event event;
    {
      LockGuard guard(this->events_mutex_);
      if (this->events_.empty())
        break;
      event = std::move(this->events_.front());
      this->events_.pop_front();
    }
    switch (event.type) {
      case Event::Type::CONNECTED:
        this->on_connected_();
        break;
      case Event::Type::DISCONNECTED:
        if (this->connected_) {
          this->connected_ = false;
          ESP_LOGW(TAG, "Disconnected");
          this->disconnect_callbacks_.call();
        }
        break;
      case Event::Type::MESSAGE:
        this->on_message_(event.topic, event.payload);
        break;
    }
  }
  this->disable_loop();
}

void AwsIotClient::on_connected_() {
  ESP_LOGI(TAG, "Connected");
  this->connected_ = true;
  for (size_t i = 0; i < this->subscriptions_.size(); i++) {
    const std::string &topic = this->subscriptions_[i].topic;
    bool duplicate = false;
    for (size_t j = 0; j < i; j++)
      duplicate = duplicate || this->subscriptions_[j].topic == topic;
    if (!duplicate && esp_mqtt_client_subscribe_single(this->client_, topic.c_str(), QOS) < 0)
      ESP_LOGW(TAG, "Failed to subscribe to %s", topic.c_str());
  }
  this->connect_callbacks_.call();
}

void AwsIotClient::on_message_(const std::string &topic, const std::string &payload) {
  ESP_LOGV(TAG, "Received %u bytes on %s", static_cast<unsigned>(payload.size()), topic.c_str());
  // Copy the matching callbacks first, a callback may subscribe.
  std::vector<MessageCallback> matches;
  for (const auto &subscription : this->subscriptions_) {
    if (topic_matches(subscription.topic, topic))
      matches.push_back(subscription.callback);
  }
  for (const auto &callback : matches)
    callback(topic, payload);
}

void AwsIotClient::event_handler_(void *arg, esp_event_base_t base, int32_t event_id, void *event_data) {
  static_cast<AwsIotClient *>(arg)->handle_mqtt_event_(static_cast<esp_mqtt_event_handle_t>(event_data));
}

// Runs in the MQTT task. Everything the main loop needs goes through the event queue.
void AwsIotClient::handle_mqtt_event_(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      this->reassembler_.reset();
      this->push_event_({Event::Type::CONNECTED, {}, {}});
      break;
    case MQTT_EVENT_DISCONNECTED:
      this->reassembler_.reset();
      this->push_event_({Event::Type::DISCONNECTED, {}, {}});
      break;
    case MQTT_EVENT_DATA:
      if (this->reassembler_.add(event->topic, event->topic_len, event->data, event->data_len,
                                 event->current_data_offset, event->total_data_len)) {
        this->push_event_({Event::Type::MESSAGE, this->reassembler_.topic(), this->reassembler_.payload()});
        this->reassembler_.reset();
      }
      break;
    case MQTT_EVENT_ERROR:
      if (event->error_handle != nullptr) {
        ESP_LOGW(TAG, "Error type %d, TLS error 0x%x, TLS stack error 0x%x, CONNACK code %d, errno %d",
                 event->error_handle->error_type, event->error_handle->esp_tls_last_esp_err,
                 event->error_handle->esp_tls_stack_err, event->error_handle->connect_return_code,
                 event->error_handle->esp_transport_sock_errno);
      }
      break;
    default:
      break;
  }
}

void AwsIotClient::push_event_(Event &&event) {
  {
    LockGuard guard(this->events_mutex_);
    this->events_.push_back(std::move(event));
  }
  this->enable_loop_soon_any_context();
  App.wake_loop_threadsafe();
}

}  // namespace esphome::aws_iot

#endif
