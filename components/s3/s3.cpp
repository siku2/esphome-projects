#ifdef USE_ESP32

#include "s3.h"

#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <sdkconfig.h>

#include <ctime>

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include <esp_crt_bundle.h>
#endif

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::s3 {

static const char *const TAG = "s3";

static constexpr int TIMEOUT_MS = 15000;
static constexpr int RX_BUFFER_SIZE = 2048;
static constexpr int TX_BUFFER_SIZE = 1024;
// Enough for the TLS handshake with mbedtls. esp-mqtt in aws_iot uses 8 KiB.
static constexpr uint32_t TASK_STACK_SIZE = 10 * 1024;
static constexpr BaseType_t TASK_CORE = 0;
static constexpr UBaseType_t TASK_PRIORITY = 1;
static constexpr int MIN_VALID_YEAR = 2024;
static constexpr size_t DIGEST_SIZE = 32;
static constexpr const char *SIGNED_HEADERS = "host;x-amz-content-sha256;x-amz-date";

static std::string to_hex(const uint8_t *data, size_t length) {
  static constexpr char DIGITS[] = "0123456789abcdef";
  std::string out;
  out.reserve(length * 2);
  for (size_t i = 0; i < length; i++) {
    out += DIGITS[data[i] >> 4];
    out += DIGITS[data[i] & 0x0f];
  }
  return out;
}

static std::string sha256_hex(const uint8_t *data, size_t length) {
  uint8_t digest[DIGEST_SIZE];
  mbedtls_sha256(data, length, digest, 0);
  return to_hex(digest, DIGEST_SIZE);
}

static void hmac_sha256(const uint8_t *key, size_t key_length, const std::string &message, uint8_t *out) {
  mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, key_length,
                  reinterpret_cast<const uint8_t *>(message.data()), message.size(), out);
}

static std::string uri_encode(const std::string &value, bool keep_slash) {
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' ||
        c == '.' || c == '~' || (keep_slash && c == '/')) {
      out += c;
    } else {
      const uint8_t byte = static_cast<uint8_t>(c);
      out += '%';
      out += "0123456789ABCDEF"[byte >> 4];
      out += "0123456789ABCDEF"[byte & 0x0f];
    }
  }
  return out;
}

static void derive_signing_key(const std::string &secret, const std::string &date, const std::string &region,
                               uint8_t *out) {
  uint8_t key[DIGEST_SIZE];
  const std::string seed = "AWS4" + secret;
  hmac_sha256(reinterpret_cast<const uint8_t *>(seed.data()), seed.size(), date, key);
  hmac_sha256(key, DIGEST_SIZE, region, out);
  hmac_sha256(out, DIGEST_SIZE, "s3", key);
  hmac_sha256(key, DIGEST_SIZE, "aws4_request", out);
}

static std::string build_authorization(const std::string &access_key_id, const std::string &region,
                                       const std::string &host, const std::string &path, const std::string &date_time,
                                       const std::string &payload_hash, const uint8_t *signing_key) {
  const std::string canonical_request = "PUT\n" + path + "\n\nhost:" + host + "\nx-amz-content-sha256:" + payload_hash +
                                        "\nx-amz-date:" + date_time + "\n\n" + SIGNED_HEADERS + "\n" + payload_hash;
  const std::string scope = date_time.substr(0, 8) + "/" + region + "/s3/aws4_request";
  const std::string canonical_hash =
      sha256_hex(reinterpret_cast<const uint8_t *>(canonical_request.data()), canonical_request.size());
  const std::string string_to_sign = "AWS4-HMAC-SHA256\n" + date_time + "\n" + scope + "\n" + canonical_hash;
  uint8_t signature[DIGEST_SIZE];
  hmac_sha256(signing_key, DIGEST_SIZE, string_to_sign, signature);
  return "AWS4-HMAC-SHA256 Credential=" + access_key_id + "/" + scope + ", SignedHeaders=" + SIGNED_HEADERS +
         ", Signature=" + to_hex(signature, DIGEST_SIZE);
}

// Matches the Host header of esp_http_client, which leaves out the default ports.
static std::string host_header(const std::string &endpoint) {
  const size_t start = endpoint.find("://") + 3;
  size_t end = endpoint.find('/', start);
  if (end == std::string::npos)
    end = endpoint.size();
  std::string host = endpoint.substr(start, end - start);
  const size_t colon = host.rfind(':');
  if (colon != std::string::npos && host.find(']', colon) == std::string::npos) {
    const std::string port = host.substr(colon + 1);
    if (port == "80" || port == "443")
      host.erase(colon);
  }
  return host;
}

void S3Client::setup() {
  this->host_ = host_header(this->endpoint_);
  if (xTaskCreatePinnedToCore(task_entry_, "s3", TASK_STACK_SIZE, this, TASK_PRIORITY, &this->task_, TASK_CORE) !=
      pdPASS) {
    ESP_LOGE(TAG, "Creating the worker task failed");
    this->mark_failed();
    return;
  }
  this->disable_loop();
}

void S3Client::dump_config() {
  ESP_LOGCONFIG(TAG,
                "S3:\n"
                "  Endpoint: %s\n"
                "  Bucket: %s\n"
                "  Region: %s\n"
                "  Access key ID: %s\n"
                "  Custom root CA: %s",
                this->endpoint_.c_str(), this->bucket_.c_str(), this->region_.c_str(), this->access_key_id_.c_str(),
                YESNO(!this->root_ca_.empty()));
}

void S3Client::put(const std::string &key, std::shared_ptr<const std::string> body, const char *content_type,
                   DoneCallback done) {
  const auto *data = reinterpret_cast<const uint8_t *>(body->data());
  const size_t length = body->size();
  this->enqueue_({key, data, length, std::shared_ptr<void>(body, nullptr), content_type, std::move(done)});
}

void S3Client::put(const std::string &key, const uint8_t *data, size_t length, std::shared_ptr<void> keep_alive,
                   const char *content_type, DoneCallback done) {
  this->enqueue_({key, data, length, std::move(keep_alive), content_type, std::move(done)});
}

void S3Client::enqueue_(Request &&request) {
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
  ESP_LOGW(TAG, "Request queue is full or the worker failed, rejecting %s", request.key.c_str());
  this->defer([done = std::move(request.done)]() { done(false, 0); });
}

void S3Client::loop() {
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

void S3Client::task_entry_(void *arg) { static_cast<S3Client *>(arg)->task_main_(); }

void S3Client::task_main_() {
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

bool S3Client::create_client_() {
  esp_http_client_config_t config{};
  config.url = this->endpoint_.c_str();
  const bool tls = this->endpoint_.rfind("https://", 0) == 0;
  config.transport_type = tls ? HTTP_TRANSPORT_OVER_SSL : HTTP_TRANSPORT_OVER_TCP;
  config.timeout_ms = TIMEOUT_MS;
  config.buffer_size = RX_BUFFER_SIZE;
  config.buffer_size_tx = TX_BUFFER_SIZE;
  config.disable_auto_redirect = true;
  config.keep_alive_enable = true;
  if (!this->root_ca_.empty()) {
    config.cert_pem = this->root_ca_.c_str();
  } else if (tls) {
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  }
  this->client_ = esp_http_client_init(&config);
  if (this->client_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize the HTTP client");
    return false;
  }
  this->client_used_ = false;
  return true;
}

void S3Client::destroy_client_() {
  if (this->client_ != nullptr) {
    esp_http_client_cleanup(this->client_);
    this->client_ = nullptr;
  }
}

bool S3Client::sign_(const Request &request, const std::string &path) {
  const time_t now = ::time(nullptr);
  tm utc{};
  gmtime_r(&now, &utc);
  if (utc.tm_year + 1900 < MIN_VALID_YEAR) {
    ESP_LOGW(TAG, "Clock is not set, not sending %s", request.key.c_str());
    return false;
  }
  char stamp[17];
  strftime(stamp, sizeof(stamp), "%Y%m%dT%H%M%SZ", &utc);
  const std::string date_time = stamp;
  const std::string date = date_time.substr(0, 8);

  if (date != this->date_stamp_) {
    derive_signing_key(this->secret_access_key_, date, this->region_, this->signing_key_);
    this->date_stamp_ = date;
  }

  const std::string payload_hash = sha256_hex(request.data, request.length);
  const std::string authorization = build_authorization(this->access_key_id_, this->region_, this->host_, path,
                                                        date_time, payload_hash, this->signing_key_);

  esp_http_client_set_header(this->client_, "x-amz-date", date_time.c_str());
  esp_http_client_set_header(this->client_, "x-amz-content-sha256", payload_hash.c_str());
  esp_http_client_set_header(this->client_, "Authorization", authorization.c_str());
  return true;
}

esp_err_t S3Client::perform_(const Request &request, int &status) {
  status = 0;
  const std::string path = "/" + uri_encode(this->bucket_, false) + "/" + uri_encode(request.key, true);
  if (!this->sign_(request, path))
    return ESP_ERR_INVALID_STATE;
  const std::string url = this->endpoint_ + path;
  esp_http_client_set_url(this->client_, url.c_str());
  esp_http_client_set_method(this->client_, HTTP_METHOD_PUT);
  esp_http_client_set_header(this->client_, "Content-Type", request.content_type);
  esp_http_client_set_post_field(this->client_, reinterpret_cast<const char *>(request.data),
                                 static_cast<int>(request.length));
  const esp_err_t err = esp_http_client_perform(this->client_);
  status = err == ESP_OK ? esp_http_client_get_status_code(this->client_) : 0;
  return err;
}

S3Client::Result S3Client::execute_(Request &request) {
  Result result{std::move(request.done), std::move(request.keep_alive), false, 0};
  if (this->client_ == nullptr && !this->create_client_())
    return result;

  esp_err_t err = this->perform_(request, result.status);
  if (err == ESP_ERR_INVALID_STATE)
    return result;
  if (err != ESP_OK && this->client_used_) {
    // The server may have closed the idle connection, so try once more on a new one.
    this->destroy_client_();
    if (!this->create_client_())
      return result;
    err = this->perform_(request, result.status);
  }
  this->client_used_ = true;
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "PUT %s failed: %s", request.key.c_str(), esp_err_to_name(err));
    this->destroy_client_();
    return result;
  }

  result.ok = result.status >= 200 && result.status < 300;
  if (!result.ok)
    ESP_LOGW(TAG, "PUT %s returned %d", request.key.c_str(), result.status);
  return result;
}

}  // namespace esphome::s3

#endif
