#ifdef USE_ESP32

#include "camera_image.h"

#include <esp_heap_caps.h>

namespace esphome::still_camera {

JpegBuffer::~JpegBuffer() { heap_caps_free(this->data_); }

void JpegImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::static_pointer_cast<JpegImage>(image);
  this->offset_ = 0;
}

size_t JpegImageReader::available() const {
  if (!this->image_)
    return 0;
  return this->image_->get_data_length() - this->offset_;
}

uint8_t *JpegImageReader::peek_data_buffer() { return this->image_->get_data_buffer() + this->offset_; }

void JpegImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }

void JpegImageReader::return_image() { this->image_.reset(); }

}  // namespace esphome::still_camera

#endif
