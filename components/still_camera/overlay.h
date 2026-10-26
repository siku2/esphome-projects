#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <vector>

#include "frame.h"

namespace esphome::still_camera {

enum class OverlayState : uint8_t {
  NO_RESULT,
  ACCEPTED,
  REJECTED,
};

/// A box in rotated frame coordinates with a short label. A newline in the label starts a second line.
struct Overlay {
  uint16_t x{0};
  uint16_t y{0};
  uint16_t width{0};
  uint16_t height{0};
  OverlayState state{OverlayState::NO_RESULT};
  char label[16]{};
};

/// Writes the rotated frame and all overlays into dst. The buffer holds frame.width() * frame.height() RGB565 pixels
/// in the same byte order as the sensor buffer.
void render_overlays(const Frame &frame, const std::vector<Overlay> &overlays, uint16_t *dst);

}  // namespace esphome::still_camera

#endif
