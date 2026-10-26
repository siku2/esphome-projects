#pragma once

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace esphome::esphome_tflite {

struct Point {
  uint16_t x;
  uint16_t y;
};

struct Rect {
  Point top_left;
  Point bottom_right;

  uint16_t width() const { return this->bottom_right.x - this->top_left.x; }
  uint16_t height() const { return this->bottom_right.y - this->top_left.y; }
  bool empty() const { return this->width() == 0 || this->height() == 0; }
  bool fits(uint16_t frame_width, uint16_t frame_height) const {
    return this->bottom_right.x > this->top_left.x && this->bottom_right.y > this->top_left.y &&
           this->bottom_right.x <= frame_width && this->bottom_right.y <= frame_height;
  }
};

/// Parses "x,y,width,height" into a rect inside a frame of the given size.
inline bool parse_crop(const char *text, uint16_t frame_width, uint16_t frame_height, Rect &out) {
  long values[4];
  const char *cursor = text;
  for (int i = 0; i < 4; i++) {
    char *end = nullptr;
    errno = 0;
    values[i] = std::strtol(cursor, &end, 10);
    if (end == cursor || errno != 0 || values[i] < 0 || values[i] > UINT16_MAX)
      return false;
    cursor = end;
    if (i < 3) {
      if (*cursor != ',')
        return false;
      cursor++;
    }
  }
  if (*cursor != '\0')
    return false;
  const long right = values[0] + values[2];
  const long bottom = values[1] + values[3];
  if (right > UINT16_MAX || bottom > UINT16_MAX)
    return false;
  const Rect rect{{static_cast<uint16_t>(values[0]), static_cast<uint16_t>(values[1])},
                  {static_cast<uint16_t>(right), static_cast<uint16_t>(bottom)}};
  if (!rect.fits(frame_width, frame_height))
    return false;
  out = rect;
  return true;
}

/// Writes "x,y,width,height". Returns the length.
inline int format_crop(const Rect &rect, char *buffer, size_t size) {
  return std::snprintf(buffer, size, "%u,%u,%u,%u", static_cast<unsigned>(rect.top_left.x),
                       static_cast<unsigned>(rect.top_left.y), static_cast<unsigned>(rect.width()),
                       static_cast<unsigned>(rect.height()));
}

}  // namespace esphome::esphome_tflite
