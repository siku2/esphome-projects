#ifdef USE_ESP32

#include "camera_image.h"

#include <cstdlib>

namespace esphome::burst_camera {

BurstCameraImage::~BurstCameraImage() {
  free(this->jpeg_);       // NOLINT(cppcoreguidelines-no-malloc)
  free(this->secondary_);  // NOLINT(cppcoreguidelines-no-malloc)
  this->tracker_.release();
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
