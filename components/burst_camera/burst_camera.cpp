#ifdef USE_ESP32

#include "burst_camera.h"

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <img_converters.h>

#include <algorithm>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

extern "C" {
// Private to esp32-camera. Implemented in driver/sccb-ng.c.
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);
}

namespace esphome::burst_camera {

static const char *const TAG = "burst_camera";
static constexpr uint32_t FRAMEBUFFER_TASK_STACK_SIZE = 8192;
static constexpr UBaseType_t IMAGE_QUEUE_LENGTH = 8;
static constexpr uint32_t IMAGE_QUEUE_TIMEOUT_MS = 1000;
static constexpr uint32_t BURST_BASE_TIMEOUT_MS = 5000;
static constexpr uint32_t BURST_FRAME_TIMEOUT_MS = 500;
// Room for a q80 VGA grayscale frame with noise. Output that does not fit counts as a failed encode.
static constexpr size_t JPEG_SCRATCH_SIZE = 256 * 1024;
static constexpr int64_t SPACING_TOLERANCE_US = 5000;
static constexpr uint8_t IDLE_MASK = 1U << camera::IDLE;

struct JpegSink {
  uint8_t *buffer;
  size_t capacity;
  size_t size;
};

static size_t write_jpeg_chunk(void *arg, size_t index, const void *data, size_t length) {
  auto *sink = static_cast<JpegSink *>(arg);
  if (data == nullptr)
    return 0;
  if (index + length <= sink->capacity)
    memcpy(sink->buffer + index, data, length);
  sink->size = index + length;
  return length;
}

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
  this->scratch_ = static_cast<uint8_t *>(heap_caps_malloc(JPEG_SCRATCH_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->scratch_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate the JPEG scratch buffer");
    this->mark_failed();
    return;
  }
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

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    this->sensor_address_ = sensor->slv_addr;
    this->sensor_pid_ = sensor->id.PID;
  }

  this->image_queue_ = xQueueCreate(IMAGE_QUEUE_LENGTH, sizeof(QueuedImage *));
  xTaskCreatePinnedToCore(&BurstCamera::framebuffer_task, "burst_camera", FRAMEBUFFER_TASK_STACK_SIZE, this, 1,
                          &this->task_, 1);
  this->tracker_.set_task(this->task_);

  if (this->idle_update_interval_ != 0) {
    this->set_interval("idle", this->idle_update_interval_, [this]() { this->request_image(camera::IDLE); });
  }
  this->disable_loop();

  if (sensor != nullptr) {
    this->defer([this]() {
      this->sensor_ready_ = true;
      for (const auto &callback : this->sensor_ready_callbacks_)
        callback();
    });
  }
}

bool BurstCamera::write_sensor_reg(uint8_t reg, uint8_t value) {
  if (!this->sensor_ready_)
    return false;
  return SCCB_Write(this->sensor_address_, reg, value) == 0;
}

bool BurstCamera::read_sensor_reg(uint8_t reg, uint8_t &value) {
  if (!this->sensor_ready_)
    return false;
  value = SCCB_Read(this->sensor_address_, reg);
  return true;
}

void BurstCamera::add_on_sensor_ready(std::function<void()> callback) {
  if (this->sensor_ready_)
    callback();
  else
    this->sensor_ready_callbacks_.push_back(std::move(callback));
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
  switch (queued.kind) {
    case ImageKind::NORMAL:
      for (auto *listener : this->listeners_) {
        listener->on_camera_image(queued.image);
      }
      break;
    case ImageKind::BURST:
      if (!this->burst_active_)
        break;
      this->burst_frames_.push_back(queued.image);
      if (this->burst_frames_.size() >= this->burst_count_)
        this->finish_burst_();
      break;
    case ImageKind::EVENT:
      if (queued.generation == this->event_generation_ && this->event_state_ != EventState::IDLE)
        this->event_frames_.push_back(queued.image);
      break;
    case ImageKind::EVENT_END:
      if (queued.generation == this->event_generation_ && this->event_state_ != EventState::IDLE)
        this->finish_event_();
      break;
    case ImageKind::SECONDARY_READY:
      for (const auto &callback : this->secondary_ready_callbacks_)
        callback();
      break;
  }
}

bool BurstCamera::capture_burst(uint8_t count, BurstCallback callback) {
  if (this->is_failed() || this->task_ == nullptr || count == 0 || !callback || this->burst_active_ ||
      this->event_state_ != EventState::IDLE)
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

bool BurstCamera::arm_event(uint8_t pre_frames, uint32_t frame_spacing_us) {
  if (this->is_failed() || this->task_ == nullptr || this->event_state_ != EventState::IDLE)
    return false;
  if (this->burst_active_) {
    ESP_LOGW(TAG, "Finishing the burst early for an event");
    this->finish_burst_();
  }
  if (static_cast<unsigned>(pre_frames) + 1 > this->config_.fb_count) {
    ESP_LOGE(TAG, "Cannot hold %u frames with %u frame buffers", static_cast<unsigned>(pre_frames),
             static_cast<unsigned>(this->config_.fb_count));
    return false;
  }
  this->event_pre_frames_ = pre_frames;
  this->event_hold_frames_ = std::min<unsigned>(pre_frames + 1U, this->config_.fb_count - 1U);
  this->event_spacing_us_ = frame_spacing_us;
  this->event_generation_++;
  this->event_frames_.clear();
  this->event_state_ = EventState::ARMED;
  this->update_light_();
  this->min_frame_time_us_ = esp_timer_get_time();
  this->wake_task_();
  return true;
}

bool BurstCamera::capture_event(int64_t t0_us, uint8_t total, BurstCallback callback) {
  if (this->event_state_ != EventState::ARMED || total == 0 || !callback)
    return false;
  if (static_cast<unsigned>(total) + 1 > this->config_.fb_count) {
    ESP_LOGE(TAG, "Cannot capture %u frames with %u frame buffers", static_cast<unsigned>(total),
             static_cast<unsigned>(this->config_.fb_count));
    return false;
  }
  this->event_callback_ = std::move(callback);
  this->event_frames_.clear();
  this->event_t0_us_ = t0_us;
  this->event_total_ = total;
  const uint32_t frame_timeout_ms = BURST_FRAME_TIMEOUT_MS + this->event_spacing_us_ / 1000;
  this->set_timeout("event", BURST_BASE_TIMEOUT_MS + total * frame_timeout_ms, [this]() {
    ESP_LOGW(TAG, "Event capture timed out with %u frames", static_cast<unsigned>(this->event_frames_.size()));
    this->finish_event_();
  });
  this->event_state_ = EventState::CAPTURING;
  this->enable_loop_soon_any_context();
  this->wake_task_();
  return true;
}

void BurstCamera::disarm_event() {
  if (this->event_state_ == EventState::IDLE)
    return;
  this->cancel_timeout("event");
  this->event_generation_++;
  this->event_state_ = EventState::IDLE;
  this->event_frames_.clear();
  this->event_callback_ = nullptr;
  this->update_light_();
  this->wake_task_();
}

void BurstCamera::finish_event_() {
  this->cancel_timeout("event");
  this->event_generation_++;
  this->event_state_ = EventState::IDLE;
  auto frames = std::move(this->event_frames_);
  auto callback = std::move(this->event_callback_);
  this->event_frames_.clear();
  this->event_callback_ = nullptr;
  std::sort(frames.begin(), frames.end(), [](const auto &a, const auto &b) {
    const timeval x = a->get_timestamp();
    const timeval y = b->get_timestamp();
    return x.tv_sec != y.tv_sec ? x.tv_sec < y.tv_sec : x.tv_usec < y.tv_usec;
  });
  this->update_light_();
  this->wake_task_();
  if (callback)
    callback(std::move(frames));
}

bool BurstCamera::wants_frame_() const {
  if (this->event_state_ != EventState::IDLE)
    return false;
  if (this->burst_remaining_ != 0)
    return true;
  // Hold back while a listener still uses the previous image.
  return this->has_requested_image_() && this->tracker_.idle();
}

void BurstCamera::process_frame_(camera_fb_t *fb) {
  const int64_t frame_time_us = frame_time_us_(fb);
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

  auto image = this->encode_(fb, requesters);
  esp_camera_fb_return(fb);
  this->last_encode_us_ = esp_timer_get_time();
  if (!image) {
    this->single_requesters_ |= single;
    return;
  }
  this->push_image_(std::move(image), burst ? ImageKind::BURST : ImageKind::NORMAL);
}

int64_t BurstCamera::frame_time_us_(const camera_fb_t *fb) {
  return static_cast<int64_t>(fb->timestamp.tv_sec) * 1000000 + fb->timestamp.tv_usec;
}

uint8_t *BurstCamera::encode_jpeg_(camera_fb_t *fb, uint8_t quality, size_t &length) {
  JpegSink sink{this->scratch_, JPEG_SCRATCH_SIZE, 0};
  if (!frame2jpg_cb(fb, quality, write_jpeg_chunk, &sink)) {
    ESP_LOGE(TAG, "Failed to convert frame to JPEG");
    return nullptr;
  }
  if (sink.size == 0 || sink.size > sink.capacity) {
    ESP_LOGE(TAG, "JPEG of %u bytes does not fit the scratch buffer", static_cast<unsigned>(sink.size));
    return nullptr;
  }
  auto *jpeg = static_cast<uint8_t *>(heap_caps_malloc(sink.size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (jpeg == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u bytes for a JPEG", static_cast<unsigned>(sink.size));
    return nullptr;
  }
  memcpy(jpeg, this->scratch_, sink.size);
  length = sink.size;
  return jpeg;
}

std::shared_ptr<BurstCameraImage> BurstCamera::encode_(camera_fb_t *fb, uint8_t requesters) {
  size_t length = 0;
  uint8_t *jpeg = this->encode_jpeg_(fb, this->jpeg_quality_, length);
  if (jpeg == nullptr)
    return nullptr;
  return std::make_shared<BurstCameraImage>(jpeg, length, fb->width, fb->height, fb->timestamp, requesters,
                                            this->tracker_);
}

void BurstCamera::push_image_(std::shared_ptr<BurstCameraImage> image, ImageKind kind, uint32_t generation) {
  if (kind == ImageKind::BURST) {
    uint8_t remaining = this->burst_remaining_;
    do {
      if (remaining == 0)
        return;
    } while (!this->burst_remaining_.compare_exchange_weak(remaining, remaining - 1));
  }
  auto *queued = new QueuedImage{std::move(image), kind, generation};
  this->enable_loop_soon_any_context();
  App.wake_loop_threadsafe();
  if (xQueueSend(this->image_queue_, &queued, pdMS_TO_TICKS(IMAGE_QUEUE_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGW(TAG, "Dropped a frame because the main loop is busy");
    delete queued;
    return;
  }
  this->enable_loop_soon_any_context();
  App.wake_loop_threadsafe();
}

void BurstCamera::update_light_() {
  const bool wanted = this->burst_active_ || this->event_state_ != EventState::IDLE || ((this->single_requesters_ | this->stream_requesters_) & ~IDLE_MASK) != 0;
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

void BurstCamera::release_held_() {
  for (auto *fb : this->held_)
    esp_camera_fb_return(fb);
  for (auto *fb : this->post_)
    esp_camera_fb_return(fb);
  this->held_.clear();
  this->post_.clear();
}

void BurstCamera::hold_frame_(camera_fb_t *fb, size_t keep) {
  this->held_.push_back(fb);
  while (this->held_.size() > keep) {
    esp_camera_fb_return(this->held_.front());
    this->held_.erase(this->held_.begin());
  }
}

bool BurstCamera::spacing_ok_(int64_t time_us) const {
  const int64_t spacing_us = this->event_spacing_us_;
  return spacing_us == 0 || !this->has_last_kept_ ||
         time_us - this->last_kept_us_ >= std::max<int64_t>(spacing_us - SPACING_TOLERANCE_US, 0);
}

void BurstCamera::note_kept_(int64_t time_us) {
  this->last_kept_us_ = time_us;
  this->has_last_kept_ = true;
}

void BurstCamera::prepare_capture_() {
  const int64_t t0 = this->event_t0_us_;
  const uint8_t total = this->event_total_;
  const uint8_t pre = this->event_pre_frames_;
  this->keep_ = std::min<size_t>(pre, total - 1);
  const int64_t min_time = this->min_frame_time_us_;
  std::vector<camera_fb_t *> before;
  std::vector<camera_fb_t *> after;
  for (auto *fb : this->held_) {
    const int64_t time = frame_time_us_(fb);
    if (time < min_time)
      esp_camera_fb_return(fb);
    else if (time < t0)
      before.push_back(fb);
    else
      after.push_back(fb);
  }
  this->held_.clear();
  for (auto *fb : before)
    this->hold_frame_(fb, this->keep_);
  this->has_last_kept_ = !this->held_.empty();
  if (this->has_last_kept_)
    this->last_kept_us_ = frame_time_us_(this->held_.back());
  for (auto *fb : after) {
    const int64_t time = frame_time_us_(fb);
    if (this->held_.size() + this->post_.size() < total && this->spacing_ok_(time)) {
      this->note_kept_(time);
      this->post_.push_back(fb);
    } else {
      esp_camera_fb_return(fb);
    }
  }
}

void BurstCamera::finish_capture_(uint32_t generation) {
  EventState expected = EventState::CAPTURING;
  if (!this->event_state_.compare_exchange_strong(expected, EventState::DELIVERING)) {
    this->release_held_();
    return;
  }
  std::vector<camera_fb_t *> frames = std::move(this->held_);
  frames.insert(frames.end(), this->post_.begin(), this->post_.end());
  this->held_.clear();
  this->post_.clear();
  const uint8_t secondary_quality = this->event_secondary_quality_;
  const int64_t encode_start_us = esp_timer_get_time();
  const size_t frame_count = frames.size();
  if (frame_count != 0) {
    const int64_t t0 = this->event_t0_us_;
    const int64_t first_us = frame_time_us_(frames.front());
    const int64_t last_us = frame_time_us_(frames.back());
    ESP_LOGD(TAG, "Event frames: %u, first %d ms, last %d ms from T0, mean spacing %d ms",
             static_cast<unsigned>(frame_count), static_cast<int>((first_us - t0) / 1000),
             static_cast<int>((last_us - t0) / 1000),
             frame_count > 1 ? static_cast<int>((last_us - first_us) / 1000 / static_cast<int64_t>(frame_count - 1)) : 0);
  }
  std::vector<std::weak_ptr<BurstCameraImage>> images(frame_count);
  for (size_t i = 0; i < frame_count; i++) {
    auto image = this->encode_(frames[i], 0);
    if (!image)
      continue;
    if (secondary_quality != 0)
      image->expect_secondary();
    images[i] = image;
    this->push_image_(std::move(image), ImageKind::EVENT, generation);
  }
  this->push_image_(nullptr, ImageKind::EVENT_END, generation);
  ESP_LOGD(TAG, "Encoded %u event frames in %d ms", static_cast<unsigned>(frame_count),
           static_cast<int>((esp_timer_get_time() - encode_start_us) / 1000));

  if (secondary_quality != 0) {
    for (size_t i = 0; i < frame_count; i++) {
      auto image = images[i].lock();
      if (!image)
        continue;
      size_t length = 0;
      uint8_t *jpeg = this->encode_jpeg_(frames[i], secondary_quality, length);
      if (jpeg != nullptr)
        image->set_secondary(jpeg, length);
      else
        image->discard_secondary();
    }
    this->push_image_(nullptr, ImageKind::SECONDARY_READY, generation);
    ESP_LOGD(TAG, "Encoded the secondary frames in %d ms (quality %u)",
             static_cast<int>((esp_timer_get_time() - encode_start_us) / 1000), secondary_quality);
  }
  for (auto *fb : frames)
    esp_camera_fb_return(fb);
}

void BurstCamera::event_step_() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    ESP_LOGW(TAG, "Got invalid frame from camera");
    vTaskDelay(pdMS_TO_TICKS(100));
    return;
  }
  const EventState state = this->event_state_;
  const int64_t time = frame_time_us_(fb);
  if ((state != EventState::ARMED && state != EventState::CAPTURING) || time < this->min_frame_time_us_) {
    esp_camera_fb_return(fb);
    return;
  }
  const uint32_t generation = this->event_generation_;
  if (this->kept_generation_ != generation) {
    this->has_last_kept_ = false;
    this->kept_generation_ = generation;
  }
  if (state == EventState::ARMED) {
    if (!this->spacing_ok_(time)) {
      esp_camera_fb_return(fb);
      return;
    }
    for (auto *old : this->post_)
      esp_camera_fb_return(old);
    this->post_.clear();
    this->note_kept_(time);
    this->hold_frame_(fb, this->event_hold_frames_);
    return;
  }

  if (this->prepared_generation_ != generation) {
    this->prepare_capture_();
    this->prepared_generation_ = generation;
    if (this->held_.size() + this->post_.size() >= this->event_total_) {
      esp_camera_fb_return(fb);
      this->finish_capture_(generation);
      return;
    }
  }
  if (!this->spacing_ok_(time) || (time < this->event_t0_us_ && !this->post_.empty())) {
    esp_camera_fb_return(fb);
  } else if (time >= this->event_t0_us_) {
    this->note_kept_(time);
    this->post_.push_back(fb);
  } else {
    this->note_kept_(time);
    this->hold_frame_(fb, this->keep_);
  }
  if (this->held_.size() + this->post_.size() >= this->event_total_)
    this->finish_capture_(generation);
}

void BurstCamera::framebuffer_task(void *pv) {
  auto *that = static_cast<BurstCamera *>(pv);
  while (true) {
    const EventState state = that->event_state_;
    if (state == EventState::ARMED || state == EventState::CAPTURING) {
      that->event_step_();
      continue;
    }
    if (state == EventState::IDLE)
      that->release_held_();
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
