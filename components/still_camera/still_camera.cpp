#ifdef USE_ESP32

#include "still_camera.h"

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <img_converters.h>

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::still_camera {

static const char *const TAG = "still_camera";
static constexpr uint32_t WORKER_STACK_SIZE = 8192;
static constexpr UBaseType_t WORKER_PRIORITY = 1;
static constexpr BaseType_t WORKER_CORE = 1;
static constexpr size_t JPEG_INITIAL_CAPACITY = 64 * 1024;

struct JpegSink {
  uint8_t *data{nullptr};
  size_t capacity{0};
  size_t length{0};
  bool failed{false};
};

static size_t write_jpeg_chunk(void *arg, size_t index, const void *data, size_t length) {
  auto *sink = static_cast<JpegSink *>(arg);
  if (sink->failed || data == nullptr || length == 0)
    return 0;
  if (index + length > sink->capacity) {
    const size_t capacity = std::max(sink->capacity * 2, index + length);
    auto *grown = static_cast<uint8_t *>(heap_caps_realloc(sink->data, capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (grown == nullptr) {
      sink->failed = true;
      return 0;
    }
    sink->data = grown;
    sink->capacity = capacity;
  }
  memcpy(sink->data + index, data, length);
  sink->length = index + length;
  return length;
}

void BeforeCaptureDoneAction::play() { this->parent_->on_before_capture_done(); }

StillCamera::StillCamera() {
  this->config_.pin_pwdn = -1;
  this->config_.pin_reset = -1;
  this->config_.pin_xclk = -1;
  this->config_.ledc_timer = LEDC_TIMER_0;
  this->config_.ledc_channel = LEDC_CHANNEL_0;
  this->config_.pixel_format = PIXFORMAT_RGB565;
  this->config_.frame_size = FRAMESIZE_VGA;
  this->config_.jpeg_quality = 12;
  this->config_.fb_count = 1;
  this->config_.fb_location = CAMERA_FB_IN_PSRAM;
  this->config_.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
}

void StillCamera::set_data_pins(std::array<uint8_t, 8> pins) {
  this->config_.pin_d0 = pins[0];
  this->config_.pin_d1 = pins[1];
  this->config_.pin_d2 = pins[2];
  this->config_.pin_d3 = pins[3];
  this->config_.pin_d4 = pins[4];
  this->config_.pin_d5 = pins[5];
  this->config_.pin_d6 = pins[6];
  this->config_.pin_d7 = pins[7];
}

void StillCamera::set_external_clock(uint8_t pin, uint32_t frequency) {
  this->config_.pin_xclk = pin;
  this->config_.xclk_freq_hz = frequency;
}

void StillCamera::set_i2c_pins(uint8_t sda, uint8_t scl) {
  this->config_.pin_sccb_sda = sda;
  this->config_.pin_sccb_scl = scl;
}

#ifdef USE_I2C
void StillCamera::set_i2c_id(i2c::InternalI2CBus *bus) {
  this->i2c_bus_ = bus;
  this->config_.pin_sccb_sda = -1;
  this->config_.pin_sccb_scl = -1;
}
#endif

void StillCamera::setup() {
#ifdef USE_I2C
  if (this->i2c_bus_ != nullptr)
    this->config_.sccb_i2c_port = this->i2c_bus_->get_port();
#endif

  this->init_error_ = esp_camera_init(&this->config_);
  if (this->init_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(this->init_error_));
    this->mark_failed();
    return;
  }
  this->apply_sensor_settings_();

  if (xTaskCreatePinnedToCore(&StillCamera::worker_task_, "still_camera", WORKER_STACK_SIZE, this, WORKER_PRIORITY,
                              &this->task_, WORKER_CORE) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start the worker task");
    this->mark_failed();
    return;
  }
  this->jobs_.set_task(this->task_);
  this->disable_loop();

  if (this->update_interval_ != SCHEDULER_DONT_RUN)
    this->set_interval("capture", this->update_interval_, [this]() { this->capture(); });
}

void StillCamera::apply_sensor_settings_() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr)
    return;
  sensor->set_vflip(sensor, this->vertical_flip_);
  sensor->set_hmirror(sensor, this->horizontal_mirror_);
  sensor->set_contrast(sensor, this->contrast_);
  sensor->set_brightness(sensor, this->brightness_);
  sensor->set_saturation(sensor, this->saturation_);
  sensor->set_special_effect(sensor, 0);
  sensor->set_exposure_ctrl(sensor, this->aec_auto_);
  sensor->set_aec2(sensor, false);
  sensor->set_ae_level(sensor, 0);
  sensor->set_aec_value(sensor, this->aec_value_);
  sensor->set_gain_ctrl(sensor, this->agc_auto_);
  sensor->set_agc_gain(sensor, this->agc_value_);
  sensor->set_gainceiling(sensor, GAINCEILING_2X);
  sensor->set_wb_mode(sensor, this->wb_mode_);
  sensor->set_colorbar(sensor, false);
}

void StillCamera::dump_config() {
  const auto &conf = this->config_;
  static const char *const ROTATIONS[] = {"0", "90", "180", "270"};
  ESP_LOGCONFIG(TAG,
                "Still Camera:\n"
                "  Name: %s\n"
                "  Data Pins: D0:%d D1:%d D2:%d D3:%d D4:%d D5:%d D6:%d D7:%d\n"
                "  VSYNC Pin: %d\n"
                "  HREF Pin: %d\n"
                "  Pixel Clock Pin: %d\n"
                "  External Clock: Pin:%d Frequency:%u\n"
                "  I2C Pins: SDA:%d SCL:%d\n"
                "  Reset Pin: %d\n"
                "  Power Down Pin: %d\n"
                "  Rotation: %s degrees\n"
                "  Capture Interval: %" PRIu32 " ms\n"
                "  Drained Frames: %u\n"
                "  Preview JPEG Quality: %u",
                this->name_.c_str(), conf.pin_d0, conf.pin_d1, conf.pin_d2, conf.pin_d3, conf.pin_d4, conf.pin_d5,
                conf.pin_d6, conf.pin_d7, conf.pin_vsync, conf.pin_href, conf.pin_pclk, conf.pin_xclk,
                conf.xclk_freq_hz, conf.pin_sccb_sda, conf.pin_sccb_scl, conf.pin_reset, conf.pin_pwdn,
                ROTATIONS[static_cast<uint8_t>(this->rotation_)], this->update_interval_,
                static_cast<unsigned>(this->drain_frame_buffer_count_), static_cast<unsigned>(this->jpeg_quality_));

  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup Failed: %s", esp_err_to_name(this->init_error_));
    return;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    ESP_LOGE(TAG, "  Camera sensor not available");
    return;
  }
  const camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
  const auto &status = sensor->status;
  ESP_LOGCONFIG(TAG,
                "  Sensor PID: 0x%04X\n"
                "  Sensor Name: %s\n"
                "  Resolution: %ux%u\n"
                "  Contrast: %d\n"
                "  Brightness: %d\n"
                "  Saturation: %d\n"
                "  Vertical Flip: %s\n"
                "  Horizontal Mirror: %s\n"
                "  White Balance Mode: %u\n"
                "  Auto Exposure Control: %s\n"
                "  Exposure Value: %u\n"
                "  Auto Gain Control: %s\n"
                "  Gain: %u",
                sensor->id.PID, info != nullptr ? info->name : "unknown", resolution[status.framesize].width,
                resolution[status.framesize].height, status.contrast, status.brightness, status.saturation,
                ONOFF(status.vflip), ONOFF(status.hmirror), status.wb_mode, ONOFF(status.aec), status.aec_value,
                ONOFF(status.agc), status.agc_gain);
}

void StillCamera::capture() {
  if (this->is_failed())
    return;
  if (this->state_ != State::IDLE) {
    ESP_LOGW(TAG, "A capture is still running, skipping this one");
    return;
  }
  this->state_ = State::PREPARING;
  if (this->has_before_capture_)
    this->before_capture_.trigger();
  else
    this->on_before_capture_done();
}

void StillCamera::on_before_capture_done() {
  if (this->state_ != State::PREPARING)
    return;
  this->state_ = State::CAPTURING;
  if (!this->jobs_.post(JobKind::CAPTURE)) {
    ESP_LOGE(TAG, "Could not hand the capture to the worker");
    this->state_ = State::IDLE;
    this->after_capture_callbacks_.call();
  }
}

void StillCamera::loop() {
  this->events_.drain([this](Event event) { this->handle_event_(std::move(event)); });
  this->disable_loop();
}

void StillCamera::handle_event_(Event event) {
  switch (event.kind) {
    case EventKind::CAPTURED:
      this->state_ = State::PROCESSING;
      this->after_capture_callbacks_.call();
      break;
    case EventKind::CAPTURE_FAILED:
      this->state_ = State::IDLE;
      this->after_capture_callbacks_.call();
      break;
    case EventKind::PROCESSED:
      for (auto *processor : this->processors_)
        processor->publish();
      this->state_ = State::IDLE;
      this->cycle_end_callbacks_.call();
      break;
    case EventKind::IMAGE:
      for (auto *listener : this->listeners_)
        listener->on_camera_image(event.image);
      break;
  }
}

void StillCamera::post_event_(EventKind kind, std::shared_ptr<camera::CameraImage> image) {
  if (!this->events_.post(Event{kind, std::move(image)}))
    ESP_LOGW(TAG, "Dropped an event because the main loop is busy");
}

void StillCamera::request_image(camera::CameraRequester requester) {
  this->single_requesters_ |= (1U << requester);
  this->jobs_.post(JobKind::SERVE_IMAGE);
}

void StillCamera::start_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_start();
  this->stream_requesters_ |= (1U << requester);
  this->jobs_.post(JobKind::SERVE_IMAGE);
}

void StillCamera::stop_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_stop();
  this->stream_requesters_ &= ~(1U << requester);
}

void StillCamera::worker_task_(void *arg) { static_cast<StillCamera *>(arg)->run_worker_(); }

void StillCamera::run_worker_() {
  while (true) {
    JobKind job;
    if (!this->jobs_.pop(job)) {
      this->jobs_.wait();
      continue;
    }
    if (job == JobKind::CAPTURE)
      this->run_cycle_();
    else
      this->serve_image_();
  }
}

void StillCamera::release_frame_() {
  this->frame_.reset();
  this->jpeg_.reset();
  if (this->fb_ != nullptr) {
    esp_camera_fb_return(this->fb_);
    this->fb_ = nullptr;
  }
}

void StillCamera::run_cycle_() {
  const int64_t start_us = esp_timer_get_time();
  // The driver only fills its single buffer again once the previous frame is back.
  this->release_frame_();
  for (uint8_t i = 0; i < this->drain_frame_buffer_count_; i++) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale != nullptr)
      esp_camera_fb_return(stale);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    ESP_LOGW(TAG, "Got no frame from the camera");
    this->post_event_(EventKind::CAPTURE_FAILED);
    return;
  }
  if (fb->format != PIXFORMAT_RGB565 || fb->len < static_cast<size_t>(fb->width) * fb->height * 2) {
    ESP_LOGE(TAG, "Unexpected frame: format %d, %u bytes", static_cast<int>(fb->format),
             static_cast<unsigned>(fb->len));
    esp_camera_fb_return(fb);
    this->post_event_(EventKind::CAPTURE_FAILED);
    return;
  }
  this->fb_ = fb;
  this->frame_.emplace(fb->buf, fb->width, fb->height, this->rotation_);
  this->post_event_(EventKind::CAPTURED);
  const int64_t captured_us = esp_timer_get_time();

  this->overlays_.clear();
  for (auto *processor : this->processors_) {
    processor->process(*this->frame_);
    Overlay overlay;
    if (processor->get_overlay(overlay))
      this->overlays_.push_back(overlay);
  }
  this->post_event_(EventKind::PROCESSED);
  ESP_LOGD(TAG, "Captured in %d ms, processed in %d ms", static_cast<int>((captured_us - start_us) / 1000),
           static_cast<int>((esp_timer_get_time() - captured_us) / 1000));

  this->serve_image_();
}

void StillCamera::serve_image_() {
  if (!this->frame_.has_value())
    return;
  const uint8_t single = this->single_requesters_.exchange(0);
  const uint8_t requesters = single | this->stream_requesters_;
  if (requesters == 0)
    return;
  if (!this->jpeg_)
    this->jpeg_ = this->encode_jpeg_();
  if (!this->jpeg_)
    return;
  this->post_event_(EventKind::IMAGE, std::make_shared<JpegImage>(this->jpeg_, requesters));
}

std::shared_ptr<JpegBuffer> StillCamera::encode_jpeg_() {
  const int64_t start_us = esp_timer_get_time();
  const Frame &frame = *this->frame_;
  const size_t pixels = static_cast<size_t>(frame.width()) * frame.height();
  if (this->render_buffer_size_ < pixels) {
    heap_caps_free(this->render_buffer_);
    this->render_buffer_ =
        static_cast<uint16_t *>(heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    this->render_buffer_size_ = this->render_buffer_ != nullptr ? pixels : 0;
    if (this->render_buffer_ == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate the preview buffer");
      return nullptr;
    }
  }
  render_overlays(frame, this->overlays_, this->render_buffer_);

  JpegSink sink;
  sink.data = static_cast<uint8_t *>(heap_caps_malloc(JPEG_INITIAL_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (sink.data == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate the JPEG buffer");
    return nullptr;
  }
  sink.capacity = JPEG_INITIAL_CAPACITY;
  const bool ok =
      fmt2jpg_cb(reinterpret_cast<uint8_t *>(this->render_buffer_), pixels * sizeof(uint16_t), frame.width(),
                 frame.height(), PIXFORMAT_RGB565, this->jpeg_quality_, write_jpeg_chunk, &sink);
  if (!ok || sink.failed || sink.length == 0) {
    ESP_LOGE(TAG, "Failed to encode the preview JPEG");
    heap_caps_free(sink.data);
    return nullptr;
  }
  ESP_LOGD(TAG, "Encoded a %u byte preview in %d ms", static_cast<unsigned>(sink.length),
           static_cast<int>((esp_timer_get_time() - start_us) / 1000));
  return std::make_shared<JpegBuffer>(sink.data, sink.length);
}

}  // namespace esphome::still_camera

#endif
