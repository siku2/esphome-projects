#pragma once

#ifdef USE_ESP32

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <array>
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/core/component.h"

#include "camera_image.h"
#include "camera_power.h"
#include "image_tracker.h"

namespace esphome::burst_camera {

enum class SensorKind : uint8_t { UNKNOWN, OV7670, GC032A };

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
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  /// Also encodes event frames with this JPEG quality. 0 turns it off.
  void set_event_secondary_quality(uint8_t quality) { this->event_secondary_quality_ = quality; }
  void set_frame_buffer_count(uint8_t count) { this->config_.fb_count = count; }
  void set_max_update_interval(uint32_t interval) { this->max_update_interval_ = interval; }
  void set_idle_update_interval(uint32_t interval) { this->idle_update_interval_ = interval; }

  CameraPower &power() { return this->power_; }

  /// Raw sensor register access over SCCB. Safe from any task. Use a SensorLock to keep several accesses together.
  bool write_sensor_reg(uint8_t reg, uint8_t value);
  /// The SCCB read cannot report errors, so this only fails if there is no sensor.
  bool read_sensor_reg(uint8_t reg, uint8_t &value);
  /// Recursive. The framebuffer task reads the exposure registers between the unlocked accesses.
  void lock_sensor() { xSemaphoreTakeRecursive(this->sccb_mutex_, portMAX_DELAY); }
  void unlock_sensor() { xSemaphoreGiveRecursive(this->sccb_mutex_); }
  uint16_t sensor_pid() const { return this->sensor_pid_; }
  SensorKind sensor_kind() const;
  uint32_t external_clock_frequency() const { return this->config_.xclk_freq_hz; }
  bool is_sensor_ready() const { return this->sensor_ready_; }
  /// Runs in the main loop once the sensor is initialized. Runs right away if it already is.
  void add_on_sensor_ready(std::function<void()> callback);

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

  /// Turns the light on and keeps the latest `pre_frames` + 1 raw frames until capture_event or disarm_event. Main loop
  /// only. Finishes a running burst early. Event frames start at least `frame_spacing_us` apart (with a small tolerance
  /// for timestamp jitter). 0 keeps every frame.
  bool arm_event(uint8_t pre_frames, uint32_t frame_spacing_us = 0);
  /// Uses up to `pre_frames` held frames that started before t0_us, then grabs frames that start at or after t0_us
  /// until `total` frames are held, JPEG encodes them in the framebuffer task and calls back in the main loop with the
  /// frames sorted by timestamp. Main loop only. Returns false if not armed or busy.
  bool capture_event(int64_t t0_us, uint8_t total, BurstCallback callback);
  /// Runs in the main loop once the second encodings of an event are ready or have failed. The event itself was
  /// delivered earlier, and its images report the second encoding through get_secondary_data().
  void add_on_event_secondary_ready(std::function<void()> callback) {
    this->secondary_ready_callbacks_.push_back(std::move(callback));
  }
  /// Returns all held frames and releases the light. Safe to call at any time.
  void disarm_event();
  bool is_armed() const { return this->event_state_.load() == EventState::ARMED; }

 protected:
  enum class EventState : uint8_t { IDLE, ARMED, CAPTURING, DELIVERING };
  enum class ImageKind : uint8_t { NORMAL, BURST, EVENT, EVENT_END, SECONDARY_READY };

  /// An encoded frame on its way from the framebuffer task to the main loop.
  struct QueuedImage {
    std::shared_ptr<BurstCameraImage> image;
    ImageKind kind;
    uint32_t generation;
  };

  /// Exposure state of the sensor when a frame was grabbed.
  struct FrameMeta {
    uint32_t gain{0};
    uint32_t exposure{0};
  };

  static void framebuffer_task(void *pv);

  bool has_requested_image_() const { return this->single_requesters_ != 0 || this->stream_requesters_ != 0; }
  bool wants_frame_() const;
  void process_frame_(camera_fb_t *fb);
  void push_image_(std::shared_ptr<BurstCameraImage> image, ImageKind kind, uint32_t generation = 0);
  void dispatch_(const QueuedImage &queued);
  void finish_burst_();
  void finish_event_();
  static int64_t frame_time_us_(const camera_fb_t *fb);
  uint8_t *encode_jpeg_(camera_fb_t *fb, uint8_t quality, size_t &length);
  std::shared_ptr<BurstCameraImage> encode_(camera_fb_t *fb, uint8_t requesters, const FrameMeta &meta);
  FrameMeta read_frame_meta_();
  void note_frame_(camera_fb_t *fb);
  void event_step_();
  void hold_frame_(camera_fb_t *fb, size_t keep);
  void prepare_capture_();
  bool spacing_ok_(int64_t time_us) const;
  void note_kept_(int64_t time_us);
  void finish_capture_(uint32_t generation);
  void release_held_();
  void wake_task_();
  void update_light_();
  void note_request_();

  camera_config_t config_{};
  CameraPower power_;
  uint8_t jpeg_quality_{30};
  std::atomic<uint8_t> event_secondary_quality_{0};
  uint32_t max_update_interval_{100};
  uint32_t idle_update_interval_{0};
  esp_err_t init_error_{ESP_OK};
  uint8_t sensor_address_{0};
  uint16_t sensor_pid_{0};
  std::atomic<bool> sensor_ready_{false};
  SemaphoreHandle_t sccb_mutex_{nullptr};
  std::vector<std::function<void()>> sensor_ready_callbacks_;
  std::vector<std::function<void()>> secondary_ready_callbacks_;

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

  std::atomic<EventState> event_state_{EventState::IDLE};
  std::atomic<uint8_t> event_pre_frames_{0};
  std::atomic<uint8_t> event_hold_frames_{0};
  std::atomic<uint8_t> event_total_{0};
  std::atomic<uint32_t> event_spacing_us_{0};
  std::atomic<int64_t> event_t0_us_{0};
  std::atomic<uint32_t> event_generation_{0};
  std::vector<std::shared_ptr<BurstCameraImage>> event_frames_;
  BurstCallback event_callback_;

  // Only used by the framebuffer task.
  uint8_t *scratch_{nullptr};
  std::vector<camera_fb_t *> held_;
  std::vector<camera_fb_t *> post_;
  std::map<const camera_fb_t *, FrameMeta> frame_meta_;
  uint32_t prepared_generation_{0};
  size_t keep_{0};
  int64_t last_kept_us_{0};
  bool has_last_kept_{false};
  uint32_t kept_generation_{0};
};

/// Holds the SCCB bus for the lifetime of the object.
class SensorLock {
 public:
  explicit SensorLock(BurstCamera &camera) : camera_(camera) { this->camera_.lock_sensor(); }
  ~SensorLock() { this->camera_.unlock_sensor(); }
  SensorLock(const SensorLock &) = delete;
  SensorLock &operator=(const SensorLock &) = delete;

 protected:
  BurstCamera &camera_;
};

}  // namespace esphome::burst_camera

#endif
