#ifdef USE_ESP32

#include "camera_image.h"

#include <esp_heap_caps.h>

namespace esphome::burst_camera {

BurstCameraImage::~BurstCameraImage() {
  heap_caps_free(this->jpeg_);
  if (this->secondary_state_ == SecondaryState::READY)
    heap_caps_free(this->secondary_);
  if (this->tracked_)
    this->tracker_.release();
}

void BurstCameraImage::set_secondary(uint8_t *jpeg, size_t length) {
  this->secondary_ = jpeg;
  this->secondary_length_ = length;
  SecondaryState expected = SecondaryState::PENDING;
  if (!this->secondary_state_.compare_exchange_strong(expected, SecondaryState::READY))
    heap_caps_free(jpeg);
}

void BurstCameraImage::discard_secondary() {
  if (this->secondary_state_.exchange(SecondaryState::NONE) == SecondaryState::READY)
    heap_caps_free(this->secondary_);
}

void BurstCameraImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<BurstCameraImage>(image);
  this->offset_ = 0;
}

size_t BurstCameraImageReader::available() const {
  if (!this->image_)
    return 0;
  return this->image_->get_data_length() - this->offset_;
}

uint8_t *BurstCameraImageReader::peek_data_buffer() { return this->image_->get_data_buffer() + this->offset_; }

void BurstCameraImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }

void BurstCameraImageReader::return_image() { this->image_.reset(); }

}  // namespace esphome::burst_camera

#endif
