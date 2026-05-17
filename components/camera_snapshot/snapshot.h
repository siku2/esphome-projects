#pragma once

#include <esp_jpeg_common.h>
#include <esphome/core/color.h>
#include <cstdint>

namespace esphome::camera::snapshot {
class Snapshot {
  friend class Decoder;

 public:
  uint16_t width() const { return this->width_; }
  uint16_t height() const { return this->height_; }
  Color get_pixel(uint16_t x, uint16_t y) const;

 protected:
  uint16_t width_{0};
  uint16_t height_{0};
  jpeg_pixel_format_t pixel_format_{};
  uint8_t *data_{nullptr};
  size_t data_len_{0};

  Snapshot() = default;
};
}  // namespace esphome::camera::snapshot
