#pragma once

#ifdef USE_ESP32

#include <cstdint>

#include "esphome/core/color.h"

namespace esphome::still_camera {

enum class Rotation : uint8_t {
  ROTATE_0,
  ROTATE_90,
  ROTATE_180,
  ROTATE_270,
};

/// A captured RGB565 image seen through a clockwise rotation. Reads the sensor buffer in place.
class Frame {
 public:
  Frame(const uint8_t *data, uint16_t raw_width, uint16_t raw_height, Rotation rotation);

  /// Size after rotation.
  uint16_t width() const { return this->width_; }
  uint16_t height() const { return this->height_; }

  /// Pixel at rotated coordinates, expanded to 8 bits per channel.
  Color get_pixel(uint16_t x, uint16_t y) const;
  /// Pixel at rotated coordinates as stored in the buffer (high byte first in memory).
  uint16_t get_raw(uint16_t x, uint16_t y) const {
    return this->data_[this->base_ + static_cast<int32_t>(x) * this->step_x_ + static_cast<int32_t>(y) * this->step_y_];
  }

 protected:
  const uint16_t *data_;
  uint16_t width_;
  uint16_t height_;
  int32_t base_;
  int32_t step_x_;
  int32_t step_y_;
};

}  // namespace esphome::still_camera

#endif
