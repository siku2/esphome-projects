#pragma once

#ifdef USE_ESP32

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/core/component.h"

#include "camera_image.h"
#include "camera_power.h"
#include "image_tracker.h"

namespace esphome::burst_camera {

using BurstCallback = std::function<void(std::vector<std::shared_ptr<BurstCameraImage>> frames)>;

class BurstCamera final : public camera::Camera {
 public:
  BurstCamera();

  void set_data_pins(std::array<uint8_t, 8> pins);
  void set_vsync_pin(uint8_t pin) { this->config_.pin_vsync = pin; }
  void set_href_pin(uint8_t pin) { this->config_.pin_href = pin; }
  void set_pixel_clock_pin(uint8_t pin) { this->config_.pin_pclk = pin; }
  void set_external_clock(uint8_t pin, uint32_t frequency);
  void set_i2c_pins(uint8_t sda, uint8_t scl);
  void set_frame_size(framesize_t size) { this->config_.frame_size = size; }
  void set_pixel_format(pixformat_t format) { this->config_.pixel_format = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_frame_buffer_count(uint8_t count) { this->config_.fb_count = count; }
  void set_max_update_interval(uint32_t interval) { this->max_update_interval_ = interval; }
  void set_idle_update_interval(uint32_t interval) { this->idle_update_interval_ = interval; }

  CameraPower &power() { return this->power_; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new BurstCameraImageReader; }
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

  /// Captures `count` consecutive frames, JPEG encoded. Calls back in the main loop.
  /// Returns false if busy or failed. Call from the main loop only.
  bool capture_burst(uint8_t count, BurstCallback callback);

 protected:
  /// An encoded frame on its way from the framebuffer task to the main loop.
  struct QueuedImage {
    std::shared_ptr<BurstCameraImage> image;
    bool burst;
  };

  static void framebuffer_task(void *pv);

  bool has_requested_image_() const { return this->single_requesters_ != 0 || this->stream_requesters_ != 0; }
  bool wants_frame_() const;
  void process_frame_(camera_fb_t *fb);
  void push_image_(std::shared_ptr<BurstCameraImage> image, bool burst);
  void dispatch_(const QueuedImage &queued);
  void finish_burst_();
  void wake_task_();
  void update_light_();
  void note_request_();

  camera_config_t config_{};
  CameraPower power_;
  uint8_t jpeg_quality_{30};
  uint32_t max_update_interval_{100};
  uint32_t idle_update_interval_{0};
  esp_err_t init_error_{ESP_OK};

  TaskHandle_t task_{nullptr};
  QueueHandle_t image_queue_{nullptr};
  ImageTracker tracker_;
  std::vector<camera::CameraListener *> listeners_;
  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};
  std::atomic<int64_t> min_frame_time_us_{0};
  int64_t last_encode_us_{0};

  std::atomic<uint8_t> burst_remaining_{0};
  bool burst_active_{false};
  uint8_t burst_count_{0};
  std::vector<std::shared_ptr<BurstCameraImage>> burst_frames_;
  BurstCallback burst_callback_;
};

}  // namespace esphome::burst_camera

#endif
