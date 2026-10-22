#pragma once

#ifdef USE_ESP32

#include <sys/time.h>

#include <cstdint>
#include <memory>

#include "esphome/components/camera/camera.h"

#include "image_tracker.h"

namespace esphome::burst_camera {

/// A JPEG encoded frame. Owns its buffer, so the sensor frame buffer is free as soon as it is encoded.
class BurstCameraImage : public camera::CameraImage {
 public:
  BurstCameraImage(uint8_t *jpeg, size_t length, uint16_t width, uint16_t height, timeval timestamp, uint8_t requesters,
                   ImageTracker &tracker)
      : jpeg_(jpeg),
        length_(length),
        width_(width),
        height_(height),
        timestamp_(timestamp),
        requesters_(requesters),
        tracker_(tracker) {
    this->tracker_.acquire();
  }
  ~BurstCameraImage() override;

  /// Takes ownership of a second encoding of the same frame.
  void set_secondary(uint8_t *jpeg, size_t length) {
    this->secondary_ = jpeg;
    this->secondary_length_ = length;
  }
  /// The second encoding or nullptr.
  const uint8_t *get_secondary_data() const { return this->secondary_; }
  size_t get_secondary_length() const { return this->secondary_length_; }

  uint8_t *get_data_buffer() override { return this->jpeg_; }
  size_t get_data_length() override { return this->length_; }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1U << requester)) != 0;
  }

  uint16_t get_width() const { return this->width_; }
  uint16_t get_height() const { return this->height_; }
  /// Time since boot at which the sensor started this frame.
  timeval get_timestamp() const { return this->timestamp_; }

 protected:
  uint8_t *jpeg_;
  size_t length_;
  uint8_t *secondary_{nullptr};
  size_t secondary_length_{0};
  uint16_t width_;
  uint16_t height_;
  timeval timestamp_;
  uint8_t requesters_;
  ImageTracker &tracker_;
};

class BurstCameraImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<BurstCameraImage> image_;
  size_t offset_{0};
};

}  // namespace esphome::burst_camera

#endif
