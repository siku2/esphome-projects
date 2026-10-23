#ifdef USE_ESP32

#include "aws_iot.h"

#include <algorithm>
#include <memory>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include "root_ca.h"

namespace esphome::aws_iot {

static const char *const TAG = "aws_iot";
static constexpr int KEEPALIVE_S = 60;
static constexpr int BUFFER_SIZE = 8 * 1024;
static constexpr int TASK_STACK_SIZE = 8 * 1024;
// esp-mqtt drops unacknowledged messages after 30 s, so a larger timeout means no resend while connected.
static constexpr int RETRANSMIT_TIMEOUT_MS = 60000;
static constexpr uint32_t START_RETRY_MS = 30000;
static constexpr uint32_t SUBACK_TIMEOUT_MS = 10000;
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

  TaskHandle_t sender = nullptr;
  if (xTaskCreate(&AwsIotClient::sender_task_, "aws_iot_tx", TASK_STACK_SIZE, this, 1, &sender) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the sender task");
    this->mark_failed();
    return;
  }
  this->send_queue_.set_task(sender);

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
  this->cancel_timeout("start");
  if (this->connected_) {
    this->connected_ = false;
    ESP_LOGW(TAG, "Disconnected");
    this->disconnect_callbacks_.call();
  }
  this->request_stop_();
  ESP_LOGI(TAG, "Disabled");
}

void AwsIotClient::request_stop_() {
  if (this->client_ == nullptr || this->busy_.exchange(true))
    return;
  if (xTaskCreate(&AwsIotClient::stop_task_, "aws_iot_stop", TASK_STACK_SIZE, this, 1, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the stop task");
    this->busy_ = false;
  }
}

void AwsIotClient::stop_task_(void *arg) {
  auto *that = static_cast<AwsIotClient *>(arg);
  esp_mqtt_client_handle_t client = that->client_.exchange(nullptr);
  // A publish that is still running holds the mutex. Wait for it before the client goes away.
  {
    LockGuard guard(that->client_mutex_);
  }
  if (client != nullptr) {
    esp_mqtt_client_stop(client);
    esp_mqtt_client_destroy(client);
  }
  that->reassembler_.reset();
  {
    LockGuard guard(that->subscriptions_mutex_);
    that->subscribed_ = false;
  }
  that->awaiting_suback_ = false;
  that->events_.remove_if([](const Event &event) { return event.type != Event::Type::PUBLISHED; });
  that->busy_ = false;
  that->push_event_({Event::Type::STOPPED, {}, {}});
  vTaskDelete(nullptr);
}

void AwsIotClient::dump_config() {
  ESP_LOGCONFIG(TAG,
                "AWS IoT:\n"
                "  Endpoint: %s:%u\n"
                "  Username: %s\n"
                "  Credentials: %s\n"
                "  Enabled: %s",
                this->endpoint_.c_str(), this->port_, this->username_.c_str(), YESNO(this->has_credentials_()),
                YESNO(this->enabled_));
  if (!this->client_id_.empty())
    ESP_LOGCONFIG(TAG, "  Client ID: %s", this->client_id_.c_str());
}

esp_mqtt_client_config_t AwsIotClient::make_config_(bool clean_session) const {
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
  config.session.disable_clean_session = !clean_session;
  config.session.message_retransmit_timeout = RETRANSMIT_TIMEOUT_MS;
  config.buffer.size = BUFFER_SIZE;
  config.buffer.out_size = BUFFER_SIZE;
  config.task.stack_size = TASK_STACK_SIZE;
  return config;
}

void AwsIotClient::start_client_() {
  if (!this->enabled_ || this->client_ != nullptr)
    return;
  if (!this->has_credentials_()) {
    if (!this->warned_no_credentials_) {
      ESP_LOGW(TAG, "No credentials, not connecting");
      this->warned_no_credentials_ = true;
    }
    return;
  }
  if (this->busy_.exchange(true))
    return;
  this->cancel_timeout("start");
  if (xTaskCreate(&AwsIotClient::start_task_, "aws_iot_start", TASK_STACK_SIZE, this, 1, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the start task");
    this->busy_ = false;
    this->retry_start_();
  }
}

void AwsIotClient::start_task_(void *arg) {
  auto *that = static_cast<AwsIotClient *>(arg);
  const esp_mqtt_client_config_t config = that->make_config_(!that->session_opened_);
  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&config);
  if (client == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize the MQTT client");
  } else {
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, &AwsIotClient::event_handler_, that);
    const esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start the MQTT client: %s", esp_err_to_name(err));
      esp_mqtt_client_destroy(client);
      client = nullptr;
    } else {
      that->client_ = client;
      ESP_LOGI(TAG, "Connecting to %s", that->endpoint_.c_str());
    }
  }
  that->busy_ = false;
  if (client == nullptr)
    that->push_event_({Event::Type::START_FAILED, {}, {}});
  else if (!that->enabled_)
    that->request_stop_();
  vTaskDelete(nullptr);
}

void AwsIotClient::retry_start_() {
  ESP_LOGW(TAG, "Trying again in %u s", static_cast<unsigned>(START_RETRY_MS / 1000));
  this->set_timeout("start", START_RETRY_MS, [this]() { this->start_client_(); });
}

bool AwsIotClient::publish_blocking(const std::string &topic, const char *data, size_t len) {
  if (!this->connected_) {
    ESP_LOGW(TAG, "Not connected, dropping publish on %s", topic.c_str());
    return false;
  }
  LockGuard guard(this->client_mutex_);
  esp_mqtt_client_handle_t client = this->client_;
  if (client == nullptr)
    return false;
  // Returns after the whole message is written to the socket.
  const int id = esp_mqtt_client_publish(client, topic.c_str(), data, static_cast<int>(len), QOS, 0);
  if (id < 0) {
    ESP_LOGE(TAG, "Failed to publish %u bytes on %s", static_cast<unsigned>(len), topic.c_str());
    return false;
  }
  ESP_LOGD(TAG, "Published %u bytes on %s", static_cast<unsigned>(len), topic.c_str());
  return true;
}

bool AwsIotClient::publish_async(const std::string &topic, const char *data, size_t len, PublishCallback done) {
  auto *job = new Outgoing{Outgoing::Kind::PUBLISH, topic, std::string(data, len), std::move(done)};
  if (!this->send_queue_.post(std::unique_ptr<Outgoing>(job))) {
    ESP_LOGW(TAG, "Sender queue is full, dropping publish on %s", topic.c_str());
    return false;
  }
  return true;
}

void AwsIotClient::subscribe(const std::string &topic, MessageCallback callback) {
  bool known = false;
  bool live;
  {
    LockGuard guard(this->subscriptions_mutex_);
    for (const auto &subscription : this->subscriptions_)
      known = known || subscription.topic == topic;
    this->subscriptions_.push_back({topic, std::move(callback)});
    live = this->subscribed_;
  }
  if (!live || known)
    return;
  auto *job = new Outgoing{Outgoing::Kind::SUBSCRIBE, topic, {}, nullptr};
  if (!this->send_queue_.post(std::unique_ptr<Outgoing>(job)))
    ESP_LOGW(TAG, "Sender queue is full, not subscribing to %s", topic.c_str());
}

void AwsIotClient::sender_task_(void *arg) {
  auto *that = static_cast<AwsIotClient *>(arg);
  while (true) {
    that->send_queue_.wait();
    std::unique_ptr<Outgoing> job;
    while (that->send_queue_.pop(job))
      that->send_(*job);
  }
}

void AwsIotClient::send_(Outgoing &job) {
  bool ok = false;
  if (job.kind == Outgoing::Kind::PUBLISH) {
    ok = this->publish_blocking(job.topic, job.payload.data(), job.payload.size());
  } else {
    LockGuard guard(this->client_mutex_);
    esp_mqtt_client_handle_t client = this->client_;
    ok = client != nullptr && esp_mqtt_client_subscribe_single(client, job.topic.c_str(), QOS) >= 0;
    if (!ok)
      ESP_LOGW(TAG, "Failed to subscribe to %s", job.topic.c_str());
  }
  if (job.done)
    this->push_event_({Event::Type::PUBLISHED, {}, {}, std::move(job.done), ok});
}

void AwsIotClient::loop() {
  this->events_.drain([this](Event event) {
    switch (event.type) {
      case Event::Type::SUBSCRIBING:
        this->set_timeout("suback", SUBACK_TIMEOUT_MS, [this]() { this->on_subscribe_timeout_(); });
        break;
      case Event::Type::CONNECTED:
        this->cancel_timeout("suback");
        if (this->enabled_)
          this->on_connected_();
        break;
      case Event::Type::DISCONNECTED:
        this->cancel_timeout("suback");
        if (this->connected_) {
          this->connected_ = false;
          ESP_LOGW(TAG, "Disconnected");
          this->disconnect_callbacks_.call();
        }
        break;
      case Event::Type::MESSAGE:
        this->on_message_(event.topic, event.payload);
        break;
      case Event::Type::PUBLISHED:
        event.done(event.sent);
        break;
      case Event::Type::STOPPED:
        this->start_client_();
        break;
      case Event::Type::START_FAILED:
        this->retry_start_();
        break;
    }
  });
  this->disable_loop();
}

void AwsIotClient::on_connected_() {
  ESP_LOGI(TAG, "Connected");
  this->connected_ = true;
  this->connect_callbacks_.call();
}

void AwsIotClient::on_subscribe_timeout_() {
  if (!this->awaiting_suback_.exchange(false) || !this->enabled_)
    return;
  ESP_LOGW(TAG, "Subscriptions were not acknowledged in time, continuing");
  this->on_connected_();
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

// Runs in the MQTT task with the esp-mqtt API lock held. Everything the main loop needs goes through the event queue.
void AwsIotClient::handle_mqtt_event_(esp_mqtt_event_handle_t event) {
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      this->reassembler_.reset();
      this->open_session_(event->client);
      this->subscribe_all_(event->client);
      break;
    case MQTT_EVENT_SUBSCRIBED:
      if (event->error_handle != nullptr && event->error_handle->error_type == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED)
        ESP_LOGW(TAG, "The broker rejected a subscription (message id %d)", event->msg_id);
      this->pending_subscriptions_.erase(
          std::remove(this->pending_subscriptions_.begin(), this->pending_subscriptions_.end(), event->msg_id),
          this->pending_subscriptions_.end());
      if (this->pending_subscriptions_.empty() && this->awaiting_suback_.exchange(false))
        this->push_event_({Event::Type::CONNECTED, {}, {}});
      break;
    case MQTT_EVENT_DISCONNECTED:
      this->reassembler_.reset();
      this->pending_subscriptions_.clear();
      this->awaiting_suback_ = false;
      {
        LockGuard guard(this->subscriptions_mutex_);
        this->subscribed_ = false;
      }
      this->push_event_({Event::Type::DISCONNECTED, {}, {}});
      break;
    case MQTT_EVENT_DATA:
      if (this->reassembler_.add(event->topic, event->topic_len, event->data, event->data_len,
                                 event->current_data_offset, event->total_data_len)) {
        Event message{Event::Type::MESSAGE, this->reassembler_.take_topic(), this->reassembler_.take_payload()};
        this->reassembler_.reset();
        this->push_event_(std::move(message));
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

// Only the first connect after boot starts a clean session. Nothing is in flight right after the CONNACK,
// so esp-mqtt can replace its buffers here.
void AwsIotClient::open_session_(esp_mqtt_client_handle_t client) {
  if (this->session_opened_.exchange(true))
    return;
  const esp_mqtt_client_config_t config = this->make_config_(false);
  const esp_err_t err = esp_mqtt_set_config(client, &config);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "Failed to keep the session for reconnects: %s", esp_err_to_name(err));
}

void AwsIotClient::subscribe_all_(esp_mqtt_client_handle_t client) {
  std::vector<std::string> topics;
  {
    LockGuard guard(this->subscriptions_mutex_);
    for (const auto &subscription : this->subscriptions_) {
      if (std::find(topics.begin(), topics.end(), subscription.topic) == topics.end())
        topics.push_back(subscription.topic);
    }
    this->subscribed_ = true;
  }
  this->pending_subscriptions_.clear();
  for (const auto &topic : topics) {
    const int id = esp_mqtt_client_subscribe_single(client, topic.c_str(), QOS);
    if (id < 0)
      ESP_LOGW(TAG, "Failed to subscribe to %s", topic.c_str());
    else
      this->pending_subscriptions_.push_back(id);
  }
  // The SUBACKs cannot be handled before this returns, so the flag is set in time.
  if (this->pending_subscriptions_.empty()) {
    this->push_event_({Event::Type::CONNECTED, {}, {}});
  } else {
    this->awaiting_suback_ = true;
    this->push_event_({Event::Type::SUBSCRIBING, {}, {}});
  }
}

void AwsIotClient::push_event_(Event &&event) { this->events_.post(std::move(event)); }

}  // namespace esphome::aws_iot

#endif
