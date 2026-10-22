#ifdef USE_ESP32

#include "burst_camera.h"

#include <driver/gpio.h>
#include <esp_timer.h>
#include <img_converters.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::burst_camera {

static const char *const TAG = "burst_camera";
static constexpr uint32_t FRAMEBUFFER_TASK_STACK_SIZE = 8192;
static constexpr UBaseType_t IMAGE_QUEUE_LENGTH = 8;
static constexpr uint32_t IMAGE_QUEUE_TIMEOUT_MS = 1000;
static constexpr uint32_t BURST_BASE_TIMEOUT_MS = 5000;
static constexpr uint32_t BURST_FRAME_TIMEOUT_MS = 500;
static constexpr uint8_t IDLE_MASK = 1U << camera::IDLE;

BurstCamera::BurstCamera() {
  this->config_.pin_pwdn = -1;
  this->config_.pin_reset = -1;
  this->config_.pin_xclk = -1;
  this->config_.ledc_timer = LEDC_TIMER_0;
  this->config_.ledc_channel = LEDC_CHANNEL_0;
  this->config_.pixel_format = PIXFORMAT_GRAYSCALE;
  this->config_.frame_size = FRAMESIZE_VGA;
  this->config_.jpeg_quality = 0;
  this->config_.fb_count = 2;
  this->config_.fb_location = CAMERA_FB_IN_PSRAM;
  this->config_.grab_mode = CAMERA_GRAB_LATEST;
}

void BurstCamera::set_data_pins(std::array<uint8_t, 8> pins) {
  this->config_.pin_d0 = pins[0];
  this->config_.pin_d1 = pins[1];
  this->config_.pin_d2 = pins[2];
  this->config_.pin_d3 = pins[3];
  this->config_.pin_d4 = pins[4];
  this->config_.pin_d5 = pins[5];
  this->config_.pin_d6 = pins[6];
  this->config_.pin_d7 = pins[7];
}

void BurstCamera::set_external_clock(uint8_t pin, uint32_t frequency) {
  this->config_.pin_xclk = pin;
  this->config_.xclk_freq_hz = frequency;
}

void BurstCamera::set_i2c_pins(uint8_t sda, uint8_t scl) {
  this->config_.pin_sccb_sda = sda;
  this->config_.pin_sccb_scl = scl;
}

void BurstCamera::setup() {
  this->power_.power_up();

  this->init_error_ = esp_camera_init(&this->config_);
  if (this->init_error_ != ESP_OK) {
    ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(this->init_error_));
    this->mark_failed();
    return;
  }

  // The weakest drive level keeps the clock edges clean on this board.
  if (gpio_set_drive_capability(static_cast<gpio_num_t>(this->config_.pin_xclk), GPIO_DRIVE_CAP_0) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set the XCLK drive strength");
  }

  this->image_queue_ = xQueueCreate(IMAGE_QUEUE_LENGTH, sizeof(QueuedImage *));
  xTaskCreatePinnedToCore(&BurstCamera::framebuffer_task, "burst_camera", FRAMEBUFFER_TASK_STACK_SIZE, this, 1,
                          &this->task_, 1);
  this->tracker_.set_task(this->task_);

  if (this->idle_update_interval_ != 0) {
    this->set_interval("idle", this->idle_update_interval_, [this]() { this->request_image(camera::IDLE); });
  }
  this->disable_loop();
}

void BurstCamera::dump_config() {
  const auto &conf = this->config_;
  ESP_LOGCONFIG(TAG,
                "Burst Camera:\n"
                "  Name: %s\n"
                "  Data Pins: D0:%d D1:%d D2:%d D3:%d D4:%d D5:%d D6:%d D7:%d\n"
                "  VSYNC Pin: %d\n"
                "  HREF Pin: %d\n"
                "  Pixel Clock Pin: %d\n"
                "  External Clock: Pin:%d Frequency:%d\n"
                "  I2C Pins: SDA:%d SCL:%d\n"
                "  JPEG Quality: %u\n"
                "  Frame Buffer Count: %u",
                this->name_.c_str(), conf.pin_d0, conf.pin_d1, conf.pin_d2, conf.pin_d3, conf.pin_d4, conf.pin_d5,
                conf.pin_d6, conf.pin_d7, conf.pin_vsync, conf.pin_href, conf.pin_pclk, conf.pin_xclk,
                conf.xclk_freq_hz, conf.pin_sccb_sda, conf.pin_sccb_scl, this->jpeg_quality_,
                static_cast<unsigned>(conf.fb_count));
  this->power_.dump_config();

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
  ESP_LOGCONFIG(TAG,
                "  Sensor PID: 0x%04X\n"
                "  Sensor Name: %s\n"
                "  Resolution: %ux%u",
                sensor->id.PID, info != nullptr ? info->name : "unknown", resolution[sensor->status.framesize].width,
                resolution[sensor->status.framesize].height);
}

void BurstCamera::loop() {
  this->update_light_();
  QueuedImage *queued;
  while (xQueueReceive(this->image_queue_, &queued, 0) == pdTRUE) {
    std::unique_ptr<QueuedImage> owner(queued);
    this->dispatch_(*owner);
  }
  this->update_light_();
  if (!this->has_requested_image_() && !this->burst_active_ && uxQueueMessagesWaiting(this->image_queue_) == 0) {
    this->disable_loop();
  }
}

void BurstCamera::dispatch_(const QueuedImage &queued) {
  if (!queued.burst) {
    for (auto *listener : this->listeners_) {
      listener->on_camera_image(queued.image);
    }
    return;
  }
  if (!this->burst_active_)
    return;
  this->burst_frames_.push_back(queued.image);
  if (this->burst_frames_.size() >= this->burst_count_)
    this->finish_burst_();
}

bool BurstCamera::capture_burst(uint8_t count, BurstCallback callback) {
  if (this->is_failed() || this->task_ == nullptr || count == 0 || !callback || this->burst_active_)
    return false;

  this->burst_active_ = true;
  this->burst_count_ = count;
  this->burst_frames_.clear();
  this->burst_frames_.reserve(count);
  this->burst_callback_ = std::move(callback);

  this->update_light_();
  // The frame being exposed right now may predate the light.
  this->min_frame_time_us_ = esp_timer_get_time();
  this->burst_remaining_ = count;
  this->set_timeout("burst", BURST_BASE_TIMEOUT_MS + count * BURST_FRAME_TIMEOUT_MS, [this]() {
    ESP_LOGW(TAG, "Burst timed out with %u of %u frames", static_cast<unsigned>(this->burst_frames_.size()),
             static_cast<unsigned>(this->burst_count_));
    this->finish_burst_();
  });
  this->enable_loop_soon_any_context();
  this->wake_task_();
  return true;
}

void BurstCamera::finish_burst_() {
  this->cancel_timeout("burst");
  this->burst_remaining_ = 0;
  this->burst_active_ = false;
  auto frames = std::move(this->burst_frames_);
  auto callback = std::move(this->burst_callback_);
  this->burst_frames_.clear();
  this->burst_callback_ = nullptr;
  this->update_light_();
  callback(std::move(frames));
}

bool BurstCamera::wants_frame_() const {
  if (this->burst_remaining_ != 0)
    return true;
  // Hold back while a listener still uses the previous image.
  return this->has_requested_image_() && this->tracker_.idle();
}

void BurstCamera::process_frame_(camera_fb_t *fb) {
  const int64_t frame_time_us = static_cast<int64_t>(fb->timestamp.tv_sec) * 1000000 + fb->timestamp.tv_usec;
  const bool burst = this->burst_remaining_ != 0;
  if (frame_time_us < this->min_frame_time_us_ || !this->wants_frame_()) {
    esp_camera_fb_return(fb);
    return;
  }

  uint8_t single = 0;
  uint8_t requesters = 0;
  if (!burst) {
    single = this->single_requesters_.exchange(0);
    requesters = single | this->stream_requesters_;
  }

  uint8_t *jpeg = nullptr;
  size_t length = 0;
  const bool ok = frame2jpg(fb, this->jpeg_quality_, &jpeg, &length);
  const uint16_t width = fb->width;
  const uint16_t height = fb->height;
  const timeval timestamp = fb->timestamp;
  esp_camera_fb_return(fb);
  this->last_encode_us_ = esp_timer_get_time();
  if (!ok) {
    ESP_LOGE(TAG, "Failed to convert frame to JPEG");
    this->single_requesters_ |= single;
    return;
  }
  this->push_image_(
      std::make_shared<BurstCameraImage>(jpeg, length, width, height, timestamp, requesters, this->tracker_), burst);
}

void BurstCamera::push_image_(std::shared_ptr<BurstCameraImage> image, bool burst) {
  auto *queued = new QueuedImage{std::move(image), burst};
  this->enable_loop_soon_any_context();
  App.wake_loop_threadsafe();
  if (xQueueSend(this->image_queue_, &queued, pdMS_TO_TICKS(IMAGE_QUEUE_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGW(TAG, "Dropped a frame because the main loop is busy");
    delete queued;
    return;
  }
  if (burst)
    this->burst_remaining_--;
  this->enable_loop_soon_any_context();
  App.wake_loop_threadsafe();
}

void BurstCamera::update_light_() {
  const bool wanted = this->burst_active_ || ((this->single_requesters_ | this->stream_requesters_) & ~IDLE_MASK) != 0;
  if (this->power_.set_light_on(wanted) && wanted)
    this->min_frame_time_us_ = esp_timer_get_time();
}

void BurstCamera::note_request_() {
  // A frame that was grabbed before the request or the light may show a stale or dark scene.
  if (!this->has_requested_image_())
    this->min_frame_time_us_ = esp_timer_get_time();
}

void BurstCamera::request_image(camera::CameraRequester requester) {
  this->note_request_();
  this->single_requesters_ |= (1U << requester);
  this->enable_loop_soon_any_context();
  this->wake_task_();
}

void BurstCamera::start_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_) {
    listener->on_stream_start();
  }
  this->note_request_();
  this->stream_requesters_ |= (1U << requester);
  this->enable_loop_soon_any_context();
  this->wake_task_();
}

void BurstCamera::stop_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_) {
    listener->on_stream_stop();
  }
  this->stream_requesters_ &= ~(1U << requester);
  this->enable_loop_soon_any_context();
}

void BurstCamera::wake_task_() {
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void BurstCamera::framebuffer_task(void *pv) {
  auto *that = static_cast<BurstCamera *>(pv);
  while (true) {
    if (!that->wants_frame_()) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }
    if (that->burst_remaining_ == 0) {
      const int64_t wait_us =
          that->last_encode_us_ + static_cast<int64_t>(that->max_update_interval_) * 1000 - esp_timer_get_time();
      if (wait_us > 0) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_us / 1000) + 1);
        continue;
      }
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
      ESP_LOGW(TAG, "Got invalid frame from camera");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    that->process_frame_(fb);
  }
}

}  // namespace esphome::burst_camera

#endif
