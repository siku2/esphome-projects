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

  void set_endpoint(const std::string &endpoint) { this->endpoint_ = endpoint; }
  void set_bucket(const std::string &bucket) { this->bucket_ = bucket; }
  void set_region(const std::string &region) { this->region_ = region; }
  void set_access_key_id(const std::string &access_key_id) { this->access_key_id_ = access_key_id; }
  void set_secret_access_key(const std::string &secret_access_key) { this->secret_access_key_ = secret_access_key; }
  void set_root_ca(const std::string &root_ca) { this->root_ca_ = root_ca; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

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

  struct Result {
    DoneCallback done;
    std::shared_ptr<void> keep_alive;
    bool ok;
    int status;
  };

  static void task_entry_(void *arg);
  void task_main_();
  void enqueue_(Request &&request);
  bool create_client_();
  void destroy_client_();
  bool sign_(const Request &request, const std::string &path);
  esp_err_t perform_(const Request &request, int &status);
  Result execute_(Request &request);

  std::string endpoint_;
  std::string bucket_;
  std::string region_;
  std::string access_key_id_;
  std::string secret_access_key_;
  std::string root_ca_;
  std::string host_;

  std::string date_stamp_;
  uint8_t signing_key_[32]{};

  esp_http_client_handle_t client_{nullptr};
  bool client_used_{false};
  TaskHandle_t task_{nullptr};

  task_util::TaskInbox<Request> requests_{MAX_QUEUED};
  task_util::LoopInbox<Result> results_{this, SIZE_MAX};
};

}  // namespace esphome::s3

#endif
