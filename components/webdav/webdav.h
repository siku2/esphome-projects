#pragma once

#ifdef USE_ESP32

#include <esp_http_client.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome::webdav {

/// WebDAV client with Basic auth. One worker task runs the requests in order over a single TLS session.
class WebDav : public Component {
 public:
  /// Runs in the main loop. The status is 0 if the request did not reach the server.
  using DoneCallback = std::function<void(bool ok, int status)>;

  /// Requests that wait for the worker. More are rejected.
  static constexpr size_t MAX_QUEUED = 64;

  void set_url(const std::string &url) { this->url_ = url; }
  void set_username(const std::string &username) { this->username_ = username; }
  void set_password(const std::string &password) { this->password_ = password; }
  void set_root_ca(const std::string &root_ca) { this->root_ca_ = root_ca; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  /// Creates a collection. 201 and 405 (exists already) count as ok. Paths are relative to the URL.
  void mkcol(const std::string &path, DoneCallback done);
  /// Uploads a file. Any 2xx status counts as ok.
  void put(const std::string &path, std::shared_ptr<const std::string> body, const char *content_type,
           DoneCallback done);
  /// Uploads a file from memory that stays valid while keep_alive lives.
  void put(const std::string &path, const uint8_t *data, size_t length, std::shared_ptr<void> keep_alive,
           const char *content_type, DoneCallback done);

 protected:
  enum class Method : uint8_t { MKCOL, PUT };

  struct Request {
    Method method;
    std::string path;
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
  esp_err_t perform_(const Request &request, int &status);
  Result execute_(Request &request);

  std::string url_;
  std::string username_;
  std::string password_;
  std::string root_ca_;
  std::string authorization_;

  esp_http_client_handle_t client_{nullptr};
  bool client_used_{false};
  TaskHandle_t task_{nullptr};

  Mutex requests_mutex_;
  std::deque<Request> requests_;
  Mutex results_mutex_;
  std::deque<Result> results_;
};

}  // namespace esphome::webdav

#endif
