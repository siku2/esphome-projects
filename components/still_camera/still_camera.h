#pragma once

#ifdef USE_ESP32

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/components/task_util/task_util.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#ifdef USE_I2C
#include "esphome/components/i2c/i2c_bus.h"
#endif

#include "camera_image.h"
#include "frame.h"
#include "overlay.h"
#include "processor.h"

namespace esphome::still_camera {

class StillCamera;

/// Continues the capture once the before_capture automation has finished.
class BeforeCaptureDoneAction : public Action<> {
 public:
  explicit BeforeCaptureDoneAction(StillCamera *parent) : parent_(parent) {}

  void play() override;

 protected:
  StillCamera *parent_;
};

/// Takes one RGB565 frame per capture and hands it to the processors in a worker task. Serves the last frame with
/// the processors' overlays as JPEG when a client asks for an image.
class StillCamera final : public camera::Camera {
 public:
  StillCamera();

  void set_data_pins(std::array<uint8_t, 8> pins);
  void set_vsync_pin(uint8_t pin) { this->config_.pin_vsync = pin; }
  void set_href_pin(uint8_t pin) { this->config_.pin_href = pin; }
  void set_pixel_clock_pin(uint8_t pin) { this->config_.pin_pclk = pin; }
  void set_external_clock(uint8_t pin, uint32_t frequency);
  void set_i2c_pins(uint8_t sda, uint8_t scl);
#ifdef USE_I2C
  void set_i2c_id(i2c::InternalI2CBus *bus);
#endif
  void set_reset_pin(uint8_t pin) { this->config_.pin_reset = pin; }
  void set_power_down_pin(uint8_t pin) { this->config_.pin_pwdn = pin; }
  void set_frame_size(framesize_t size) { this->config_.frame_size = size; }

  void set_rotation(Rotation rotation) { this->rotation_ = rotation; }
  void set_update_interval(uint32_t interval) { this->update_interval_ = interval; }
  void set_drain_frame_buffer_count(uint8_t count) { this->drain_frame_buffer_count_ = count; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_vertical_flip(bool flip) { this->vertical_flip_ = flip; }
  void set_horizontal_mirror(bool mirror) { this->horizontal_mirror_ = mirror; }
  void set_contrast(int8_t contrast) { this->contrast_ = contrast; }
  void set_brightness(int8_t brightness) { this->brightness_ = brightness; }
  void set_saturation(int8_t saturation) { this->saturation_ = saturation; }
  void set_aec_auto(bool enabled) { this->aec_auto_ = enabled; }
  void set_aec_value(uint16_t value) { this->aec_value_ = value; }
  void set_agc_auto(bool enabled) { this->agc_auto_ = enabled; }
  void set_agc_value(uint8_t value) { this->agc_value_ = value; }
  void set_wb_mode(uint8_t mode) { this->wb_mode_ = mode; }

  Trigger<> *get_before_capture_trigger() { return &this->before_capture_; }
  Action<> *get_before_capture_done_action() { return &this->before_capture_done_; }
  void set_has_before_capture(bool has) { this->has_before_capture_ = has; }

  /// Runs in the main loop after the frame was grabbed, so the flash can go off while the processors run.
  template<typename F> void add_after_capture_callback(F &&callback) {
    this->after_capture_callbacks_.add(std::forward<F>(callback));
  }
  /// Runs in the main loop after every processor of a cycle has published. Not called if the capture failed.
  template<typename F> void add_cycle_end_callback(F &&callback) {
    this->cycle_end_callbacks_.add(std::forward<F>(callback));
  }

  /// Call during setup only.
  void add_processor(FrameProcessor *processor) { this->processors_.push_back(processor); }

  /// Starts a capture cycle. Main loop only. Ignored while a cycle is running.
  void capture();
  void on_before_capture_done();

  void setup() override;
  void loop() override;
  void dump_config() override;

  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new JpegImageReader; }
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

 protected:
  enum class State : uint8_t { IDLE, PREPARING, CAPTURING, PROCESSING };
  enum class JobKind : uint8_t { CAPTURE, SERVE_IMAGE };
  enum class EventKind : uint8_t { CAPTURED, CAPTURE_FAILED, PROCESSED, IMAGE };

  struct Event {
    EventKind kind{EventKind::CAPTURED};
    std::shared_ptr<camera::CameraImage> image;
  };

  static void worker_task_(void *arg);
  void run_worker_();
  void run_cycle_();
  void serve_image_();
  std::shared_ptr<JpegBuffer> encode_jpeg_();
  void release_frame_();
  void apply_sensor_settings_();
  void handle_event_(Event event);
  void post_event_(EventKind kind, std::shared_ptr<camera::CameraImage> image = nullptr);

  camera_config_t config_{};
  Rotation rotation_{Rotation::ROTATE_0};
  uint32_t update_interval_{60000};
  uint8_t drain_frame_buffer_count_{0};
  uint8_t jpeg_quality_{80};
  bool vertical_flip_{true};
  bool horizontal_mirror_{true};
  int8_t contrast_{0};
  int8_t brightness_{0};
  int8_t saturation_{0};
  bool aec_auto_{true};
  uint16_t aec_value_{300};
  bool agc_auto_{true};
  uint8_t agc_value_{0};
  uint8_t wb_mode_{0};
  esp_err_t init_error_{ESP_OK};
#ifdef USE_I2C
  i2c::InternalI2CBus *i2c_bus_{nullptr};
#endif

  State state_{State::IDLE};
  bool has_before_capture_{false};
  Trigger<> before_capture_;
  BeforeCaptureDoneAction before_capture_done_{this};
  LazyCallbackManager<void()> after_capture_callbacks_{};
  LazyCallbackManager<void()> cycle_end_callbacks_{};
  std::vector<FrameProcessor *> processors_;
  std::vector<camera::CameraListener *> listeners_;

  TaskHandle_t task_{nullptr};
  task_util::TaskInbox<JobKind> jobs_{4};
  task_util::LoopInbox<Event> events_{this, 8};
  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};

  // Worker task only.
  camera_fb_t *fb_{nullptr};
  std::optional<Frame> frame_;
  std::vector<Overlay> overlays_;
  uint16_t *render_buffer_{nullptr};
  size_t render_buffer_size_{0};
  std::shared_ptr<JpegBuffer> jpeg_;
};

template<typename... Ts> class CaptureAction : public Action<Ts...> {
 public:
  explicit CaptureAction(StillCamera *parent) : parent_(parent) {}

  void play(const Ts &...x) override { this->parent_->capture(); }

 protected:
  StillCamera *parent_;
};

}  // namespace esphome::still_camera

#endif
