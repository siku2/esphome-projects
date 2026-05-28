#include "snapshotter.h"
#include "esp_jpeg_common.h"

static const char *const TAG = "camera_snapshot";

using namespace esphome::camera::snapshot;

Decoder Snapshotter::global_decoder = Decoder(JPEG_PIXEL_FORMAT_RGB888);

void Snapshotter::setup() {
  this->camera_ = Camera::instance();

  if (this->camera_ == nullptr) {
    this->mark_failed();
    return;
  }

  this->camera_->add_listener(this);
}

void Snapshotter::dump_config() {
  ESP_LOGCONFIG(TAG, "Snapshotter:");
  LOG_UPDATE_INTERVAL(this);
}

void Snapshotter::loop() {
  switch (this->phase_) {
    case Phase::IDLE:
      // Nothing to do, wait for update()
      break;
    case Phase::PRIME:
      if (this->pending_image_ != nullptr) {
        this->pending_image_.reset();
        this->pre_snapshot_.trigger();
        this->phase_ = Phase::PRE_SNAPSHOT;
      }
      break;
    case Phase::PRE_SNAPSHOT:
      // Wait for the before_snapshot action to finish.
      if (!this->pre_snapshot_.is_action_running()) {
        if (this->camera_ != nullptr)
          this->camera_->request_image(IDLE);
        this->phase_ = Phase::WAITING_FOR_IMAGE;
      }
      break;
    case Phase::WAITING_FOR_IMAGE:
      // Wait for the camera to give us an image.
      if (this->pending_image_ != nullptr) {
        jpeg_error_t ret = Snapshotter::global_decoder.decode(this->pending_image_->get_data_buffer(),
                                                              this->pending_image_->get_data_length(), this->rotate_);
        this->pending_image_.reset();
        if (ret == JPEG_ERR_OK) {
          this->snapshot_ = Snapshotter::global_decoder.get_snapshot();
        } else {
          this->snapshot_.reset();
          ESP_LOGE(TAG, "Failed to decode JPEG image: %d", ret);
        }
        this->post_snapshot_.trigger();
        this->phase_ = Phase::POST_SNAPSHOT;
      }
      break;
    case Phase::POST_SNAPSHOT:
      if (!this->post_snapshot_.is_action_running()) {
        this->notifying_index_ = 0;
        this->phase_ = Phase::NOTIFYING;
      }
      break;
    case Phase::NOTIFYING:
      if (this->snapshot_.has_value() && this->notifying_index_ < this->listeners_.size()) {
        this->listeners_[this->notifying_index_]->on_snapshot(this->snapshot_.value());
        ++this->notifying_index_;
      } else {
        if (this->snapshot_.has_value())
          this->on_snapshot_.trigger(this->snapshot_.value());
        this->phase_ = Phase::ON_SNAPSHOT;
      }
      break;
    case Phase::ON_SNAPSHOT:
      // Wait for the on_snapshot action to finish.
      if (!this->on_snapshot_.is_action_running()) {
        this->phase_ = Phase::IDLE;
      }
      break;
  }
}

void Snapshotter::update() {
  if (this->phase_ != Phase::IDLE)
    return;

  // The camera keeps one frame buffered at idle_framerate.  Consume and
  // discard it now so the hardware buffer is free to capture a fresh
  // frame.
  if (this->camera_ != nullptr)
    this->camera_->request_image(camera::IDLE);
  this->phase_ = Phase::PRIME;
}

void Snapshotter::on_camera_image(const std::shared_ptr<CameraImage> &image) {
  if (this->phase_ != Phase::WAITING_FOR_IMAGE && this->phase_ != Phase::PRIME)
    return;
  this->pending_image_ = image;
}

void Snapshotter::on_stream_start() {}
void Snapshotter::on_stream_stop() {}
