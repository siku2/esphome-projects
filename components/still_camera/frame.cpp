#ifdef USE_ESP32

#include "frame.h"

namespace esphome::still_camera {

Frame::Frame(const uint8_t *data, uint16_t raw_width, uint16_t raw_height, Rotation rotation)
    : data_(reinterpret_cast<const uint16_t *>(data)) {
  const int32_t w = raw_width;
  const int32_t h = raw_height;
  switch (rotation) {
    case Rotation::ROTATE_0:
      this->width_ = raw_width;
      this->height_ = raw_height;
      this->base_ = 0;
      this->step_x_ = 1;
      this->step_y_ = w;
      break;
    case Rotation::ROTATE_90:
      this->width_ = raw_height;
      this->height_ = raw_width;
      this->base_ = (h - 1) * w;
      this->step_x_ = -w;
      this->step_y_ = 1;
      break;
    case Rotation::ROTATE_180:
      this->width_ = raw_width;
      this->height_ = raw_height;
      this->base_ = w * h - 1;
      this->step_x_ = -1;
      this->step_y_ = -w;
      break;
    case Rotation::ROTATE_270:
      this->width_ = raw_height;
      this->height_ = raw_width;
      this->base_ = w - 1;
      this->step_x_ = w;
      this->step_y_ = -1;
      break;
  }
}

Color Frame::get_pixel(uint16_t x, uint16_t y) const {
  const uint16_t value = __builtin_bswap16(this->get_raw(x, y));
  const uint8_t r = (value >> 11) & 0x1F;
  const uint8_t g = (value >> 5) & 0x3F;
  const uint8_t b = value & 0x1F;
  return Color((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2));
}

}  // namespace esphome::still_camera

#endif
