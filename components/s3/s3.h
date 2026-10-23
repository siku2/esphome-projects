#pragma once

#ifdef USE_ESP32

#include <esp_http_client.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "esphome/components/task_util/task_util.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome::s3 {

/// S3 client with AWS Signature Version 4 and path-style addressing. One worker task runs the requests in order.
class S3Client : public Component {
 public:
  /// Runs in the main loop. The status is 0 if the request did not reach the server.
  using DoneCallback = std::function<void(bool ok, int status)>;

  /// Requests that wait for the worker. More are rejected.
  static constexpr size_t MAX_QUEUED = 64;

  void set_endpoint(const std::string &endpoint) { this->set_location(endpoint, this->bucket_); }
  void set_bucket(const std::string &bucket) { this->set_location(this->endpoint_, bucket); }
  void set_region(const std::string &region) { this->region_ = region; }
  void set_access_key_id(const std::string &access_key_id) {
    this->access_key_id_ = access_key_id;
    this->config_callbacks_.call();
  }
  void set_secret_access_key(const std::string &secret_access_key) {
    this->secret_access_key_ = secret_access_key;
    this->config_callbacks_.call();
  }
  void set_root_ca(const std::string &root_ca) { this->root_ca_ = root_ca; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  bool has_credentials() const { return !this->access_key_id_.empty() && !this->secret_access_key_.empty(); }
  bool is_configured() const { return this->has_credentials() && !this->endpoint_.empty() && !this->bucket_.empty(); }

  /// Sets the endpoint and bucket. Call it from the main loop at any time. An invalid endpoint counts as empty.
  void set_location(const std::string &endpoint, const std::string &bucket);

  /// Runs in the main loop when the location or the credentials change.
  template<typename F> void add_on_config_change_callback(F &&callback) {
    this->config_callbacks_.add(std::forward<F>(callback));
  }

  /// Uploads an object. Any 2xx status counts as ok. The key is relative to the bucket.
  void put(const std::string &key, std::shared_ptr<const std::string> body, const char *content_type,
           DoneCallback done);
  /// Uploads an object from memory that stays valid while keep_alive lives.
  void put(const std::string &key, const uint8_t *data, size_t length, std::shared_ptr<void> keep_alive,
           const char *content_type, DoneCallback done);

 protected:
  struct Request {
    std::string key;
    const uint8_t *data;
    size_t length;
    std::shared_ptr<void> keep_alive;
    const char *content_type;
    DoneCallback done;
  };

  /// What the worker needs from the location for one request.
  struct Location {
    std::string endpoint;
    std::string bucket;
    std::string host;
  };

  struct Result {
    DoneCallback done;
    std::shared_ptr<void> keep_alive;
    bool ok;
    int status;
  };

  static void task_entry_(void *arg);
  void task_main_();
  void enqueue_(Request &&request);
  bool create_client_(const Location &location);
  void destroy_client_();
  bool sign_(const Request &request, const Location &location, const std::string &path);
  esp_err_t perform_(const Request &request, const Location &location, int &status);
  Result execute_(Request &request);

  // Only the main loop writes the location. The worker reads it through snapshots under location_mutex_.
  Mutex location_mutex_;
  std::string endpoint_;
  std::string bucket_;
  std::string host_;
  bool recreate_client_{false};
  std::string region_;
  std::string access_key_id_;
  std::string secret_access_key_;
  std::string root_ca_;

  std::string date_stamp_;
  uint8_t signing_key_[32]{};

  esp_http_client_handle_t client_{nullptr};
  bool client_used_{false};
  bool warned_not_configured_{false};
  TaskHandle_t task_{nullptr};

  task_util::TaskInbox<Request> requests_{MAX_QUEUED};
  task_util::LoopInbox<Result> results_{this, SIZE_MAX};
  LazyCallbackManager<void()> config_callbacks_;
};

}  // namespace esphome::s3

#endif
