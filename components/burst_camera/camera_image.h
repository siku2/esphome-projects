#pragma once

#ifdef USE_ESP32

#include <sys/time.h>

#include <atomic>
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
        tracked_(requesters != 0),
        tracker_(tracker) {
    if (this->tracked_)
      this->tracker_.acquire();
  }
  ~BurstCameraImage() override;

  /// Announces a second encoding that the framebuffer task will add later. Call before sharing the image.
  void expect_secondary() { this->secondary_state_ = SecondaryState::PENDING; }
  /// Framebuffer task only. Takes ownership of the second encoding (a heap_caps_malloc buffer).
  void set_secondary(uint8_t *jpeg, size_t length);
  /// Frees the second encoding or cancels a pending one. Safe from any thread.
  void discard_secondary();
  bool is_secondary_pending() const { return this->secondary_state_ == SecondaryState::PENDING; }
  /// The second encoding or nullptr if it is missing or not ready yet.
  const uint8_t *get_secondary_data() const {
    return this->secondary_state_ == SecondaryState::READY ? this->secondary_ : nullptr;
  }
  size_t get_secondary_length() const {
    return this->secondary_state_ == SecondaryState::READY ? this->secondary_length_ : 0;
  }

  uint8_t *get_data_buffer() override { return this->jpeg_; }
  size_t get_data_length() override { return this->length_; }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1U << requester)) != 0;
  }

  void set_sensor_meta(uint32_t gain, uint32_t exposure) {
    this->gain_ = gain;
    this->exposure_ = exposure;
  }
  /// Sensor gain (16 is 1x) and exposure rows when the frame was grabbed. 0 if unknown.
  uint32_t get_gain() const { return this->gain_; }
  uint32_t get_exposure() const { return this->exposure_; }

  uint16_t get_width() const { return this->width_; }
  uint16_t get_height() const { return this->height_; }
  /// Time since boot at which the sensor started this frame.
  timeval get_timestamp() const { return this->timestamp_; }

 protected:
  enum class SecondaryState : uint8_t { NONE, PENDING, READY };

  uint8_t *jpeg_;
  size_t length_;
  uint8_t *secondary_{nullptr};
  size_t secondary_length_{0};
  std::atomic<SecondaryState> secondary_state_{SecondaryState::NONE};
  uint32_t gain_{0};
  uint32_t exposure_{0};
  uint16_t width_;
  uint16_t height_;
  timeval timestamp_;
  uint8_t requesters_;
  bool tracked_;
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
