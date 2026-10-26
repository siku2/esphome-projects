#pragma once

#ifdef USE_ESP32

#include <memory>

#include "esphome/components/camera/camera.h"

namespace esphome::still_camera {

/// A JPEG in PSRAM. Shared by every image that is served from the same frame.
class JpegBuffer {
 public:
  JpegBuffer(uint8_t *data, size_t length) : data_(data), length_(length) {}
  ~JpegBuffer();
  JpegBuffer(const JpegBuffer &) = delete;
  JpegBuffer &operator=(const JpegBuffer &) = delete;

  uint8_t *data() const { return this->data_; }
  size_t length() const { return this->length_; }

 protected:
  uint8_t *data_;
  size_t length_;
};

class JpegImage : public camera::CameraImage {
 public:
  JpegImage(std::shared_ptr<JpegBuffer> buffer, uint8_t requesters)
      : buffer_(std::move(buffer)), requesters_(requesters) {}

  const std::shared_ptr<JpegBuffer> &buffer() const { return this->buffer_; }
  uint8_t *get_data_buffer() override { return this->buffer_->data(); }
  size_t get_data_length() override { return this->buffer_->length(); }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1U << requester)) != 0;
  }

 protected:
  std::shared_ptr<JpegBuffer> buffer_;
  uint8_t requesters_;
};

class JpegImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<JpegImage> image_;
  size_t offset_{0};
};

}  // namespace esphome::still_camera

#endif
