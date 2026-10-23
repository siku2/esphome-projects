#ifdef USE_ESP32

#include "webdav.h"

#include <sdkconfig.h>

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include <esp_crt_bundle.h>
#endif

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::webdav {

static const char *const TAG = "webdav";

static constexpr int TIMEOUT_MS = 15000;
static constexpr int RX_BUFFER_SIZE = 2048;
static constexpr int TX_BUFFER_SIZE = 1024;
// Enough for the TLS handshake with mbedtls. esp-mqtt in aws_iot uses 8 KiB.
static constexpr uint32_t TASK_STACK_SIZE = 10 * 1024;
static constexpr BaseType_t TASK_CORE = 0;
static constexpr UBaseType_t TASK_PRIORITY = 1;

void WebDav::setup() {
  const std::string credentials = this->username_ + ":" + this->password_;
  this->authorization_ =
      "Basic " + base64_encode(reinterpret_cast<const uint8_t *>(credentials.data()), credentials.size());
  if (xTaskCreatePinnedToCore(task_entry_, "webdav", TASK_STACK_SIZE, this, TASK_PRIORITY, &this->task_, TASK_CORE) !=
      pdPASS) {
    ESP_LOGE(TAG, "Creating the worker task failed");
    this->mark_failed();
    return;
  }
  this->disable_loop();
}

void WebDav::dump_config() {
  ESP_LOGCONFIG(TAG,
                "WebDAV:\n"
                "  URL: %s\n"
                "  Username: %s\n"
                "  Custom root CA: %s",
                this->url_.c_str(), this->username_.c_str(), YESNO(!this->root_ca_.empty()));
}

void WebDav::mkcol(const std::string &path, DoneCallback done) {
  this->enqueue_({Method::MKCOL, path, nullptr, 0, nullptr, nullptr, std::move(done)});
}

void WebDav::put(const std::string &path, std::shared_ptr<const std::string> body, const char *content_type,
                 DoneCallback done) {
  const auto *data = reinterpret_cast<const uint8_t *>(body->data());
  const size_t length = body->size();
  this->enqueue_(
      {Method::PUT, path, data, length, std::shared_ptr<void>(body, nullptr), content_type, std::move(done)});
}

void WebDav::put(const std::string &path, const uint8_t *data, size_t length, std::shared_ptr<void> keep_alive,
                 const char *content_type, DoneCallback done) {
  this->enqueue_({Method::PUT, path, data, length, std::move(keep_alive), content_type, std::move(done)});
}

void WebDav::enqueue_(Request &&request) {
  bool accepted = !this->is_failed();
  if (accepted) {
    LockGuard guard(this->requests_mutex_);
    accepted = this->requests_.size() < MAX_QUEUED;
    if (accepted)
      this->requests_.push_back(std::move(request));
  }
  if (accepted) {
    xTaskNotifyGive(this->task_);
    return;
  }
  ESP_LOGW(TAG, "Request queue is full or the worker failed, rejecting %s", request.path.c_str());
  this->defer([done = std::move(request.done)]() { done(false, 0); });
}

void WebDav::loop() {
  while (true) {
    Result result{};
    {
      LockGuard guard(this->results_mutex_);
      if (this->results_.empty())
        break;
      result = std::move(this->results_.front());
      this->results_.pop_front();
    }
    result.done(result.ok, result.status);
  }
  this->disable_loop();
}

void WebDav::task_entry_(void *arg) { static_cast<WebDav *>(arg)->task_main_(); }

void WebDav::task_main_() {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (true) {
      Request request{};
      {
        LockGuard guard(this->requests_mutex_);
        if (this->requests_.empty())
          break;
        request = std::move(this->requests_.front());
        this->requests_.pop_front();
      }
      Result result = this->execute_(request);
      {
        LockGuard guard(this->results_mutex_);
        this->results_.push_back(std::move(result));
      }
      this->enable_loop_soon_any_context();
      App.wake_loop_threadsafe();
    }
  }
}

bool WebDav::create_client_() {
  esp_http_client_config_t config{};
  config.url = this->url_.c_str();
  config.transport_type = HTTP_TRANSPORT_OVER_SSL;
  config.timeout_ms = TIMEOUT_MS;
  config.buffer_size = RX_BUFFER_SIZE;
  config.buffer_size_tx = TX_BUFFER_SIZE;
  config.disable_auto_redirect = true;
  config.keep_alive_enable = true;
  if (!this->root_ca_.empty()) {
    config.cert_pem = this->root_ca_.c_str();
  } else {
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  }
  this->client_ = esp_http_client_init(&config);
  if (this->client_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize the HTTP client");
    return false;
  }
  esp_http_client_set_header(this->client_, "Authorization", this->authorization_.c_str());
  this->client_used_ = false;
  return true;
}

void WebDav::destroy_client_() {
  if (this->client_ != nullptr) {
    esp_http_client_cleanup(this->client_);
    this->client_ = nullptr;
  }
}

esp_err_t WebDav::perform_(const Request &request, int &status) {
  const std::string url = this->url_ + "/" + request.path;
  esp_http_client_set_url(this->client_, url.c_str());
  if (request.method == Method::MKCOL) {
    esp_http_client_set_method(this->client_, HTTP_METHOD_MKCOL);
    esp_http_client_delete_header(this->client_, "Content-Type");
  } else {
    esp_http_client_set_method(this->client_, HTTP_METHOD_PUT);
    esp_http_client_set_header(this->client_, "Content-Type", request.content_type);
  }
  esp_http_client_set_post_field(this->client_, reinterpret_cast<const char *>(request.data),
                                 static_cast<int>(request.length));
  const esp_err_t err = esp_http_client_perform(this->client_);
  status = err == ESP_OK ? esp_http_client_get_status_code(this->client_) : 0;
  return err;
}

WebDav::Result WebDav::execute_(Request &request) {
  Result result{std::move(request.done), std::move(request.keep_alive), false, 0};
  if (this->client_ == nullptr && !this->create_client_())
    return result;

  esp_err_t err = this->perform_(request, result.status);
  if (err != ESP_OK && this->client_used_) {
    // The server may have closed the idle connection, so try once more on a new one.
    this->destroy_client_();
    if (!this->create_client_())
      return result;
    err = this->perform_(request, result.status);
  }
  this->client_used_ = true;
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "%s %s failed: %s", request.method == Method::MKCOL ? "MKCOL" : "PUT", request.path.c_str(),
             esp_err_to_name(err));
    this->destroy_client_();
    return result;
  }

  if (request.method == Method::MKCOL)
    result.ok = result.status == 201 || result.status == 405;
  else
    result.ok = result.status >= 200 && result.status < 300;
  if (!result.ok)
    ESP_LOGW(TAG, "%s %s returned %d", request.method == Method::MKCOL ? "MKCOL" : "PUT", request.path.c_str(),
             result.status);
  return result;
}

}  // namespace esphome::webdav

#endif
