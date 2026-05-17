#include "snapshot.h"

using namespace esphome::camera::snapshot;

Color Snapshot::get_pixel(uint16_t x, uint16_t y) {
  switch (this->pixel_format_) {
    case JPEG_PIXEL_FORMAT_RGB888: {
      const uint8_t *pos = this->data_ + ((x + (y * this->width_)) * 3);
      return Color(pos[0], pos[1], pos[2]);
    }
    default:
      return Color();
  }
