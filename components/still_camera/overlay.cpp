#ifdef USE_ESP32

#include "overlay.h"

#include <algorithm>
#include <cstring>

#include "font.h"

namespace esphome::still_camera {

static constexpr uint16_t rgb565_memory(uint8_t r, uint8_t g, uint8_t b) {
  return __builtin_bswap16(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static constexpr uint16_t COLOR_ACCEPTED = rgb565_memory(0, 255, 0);
static constexpr uint16_t COLOR_REJECTED = rgb565_memory(255, 0, 0);
static constexpr uint16_t COLOR_NO_RESULT = rgb565_memory(160, 160, 160);
static constexpr uint16_t COLOR_TEXT_BACKGROUND = rgb565_memory(0, 0, 0);

namespace {

class Canvas {
 public:
  Canvas(uint16_t *pixels, int width, int height) : pixels_(pixels), width_(width), height_(height) {}

  int width() const { return this->width_; }
  int height() const { return this->height_; }

  void put(int x, int y, uint16_t color) {
    if (x >= 0 && x < this->width_ && y >= 0 && y < this->height_)
      this->pixels_[y * this->width_ + x] = color;
  }

  void fill(int x, int y, int width, int height, uint16_t color) {
    for (int row = y; row < y + height; row++) {
      for (int col = x; col < x + width; col++)
        this->put(col, row, color);
    }
  }

  void outline(int x, int y, int width, int height, uint16_t color) {
    for (int col = x; col < x + width; col++) {
      this->put(col, y, color);
      this->put(col, y + height - 1, color);
    }
    for (int row = y; row < y + height; row++) {
      this->put(x, row, color);
      this->put(x + width - 1, row, color);
    }
  }

 protected:
  uint16_t *pixels_;
  int width_;
  int height_;
};

struct TextSize {
  int width;
  int height;
};

constexpr int TEXT_SCALE = 2;

TextSize measure_text(const char *text) {
  int lines = 1;
  int longest = 0;
  int current = 0;
  for (const char *c = text; *c != '\0'; c++) {
    if (*c == '\n') {
      lines++;
      current = 0;
    } else {
      longest = std::max(longest, ++current);
    }
  }
  return {(longest * (FONT_WIDTH + 1) - 1) * TEXT_SCALE, (lines * (FONT_HEIGHT + 1) - 1) * TEXT_SCALE};
}

void draw_text(Canvas &canvas, int x, int y, const char *text, uint16_t color) {
  int col = 0;
  int row = 0;
  for (const char *c = text; *c != '\0'; c++) {
    if (*c == '\n') {
      col = 0;
      row++;
      continue;
    }
    const uint8_t *glyph = font_glyph(*c);
    for (int gy = 0; gy < FONT_HEIGHT; gy++) {
      for (int gx = 0; gx < FONT_WIDTH; gx++) {
        if (glyph[gy] & (1U << (FONT_WIDTH - 1 - gx)))
          canvas.fill(x + (col * (FONT_WIDTH + 1) + gx) * TEXT_SCALE, y + (row * (FONT_HEIGHT + 1) + gy) * TEXT_SCALE,
                      TEXT_SCALE, TEXT_SCALE, color);
      }
    }
    col++;
  }
}

uint16_t state_color(OverlayState state) {
  switch (state) {
    case OverlayState::ACCEPTED:
      return COLOR_ACCEPTED;
    case OverlayState::REJECTED:
      return COLOR_REJECTED;
    default:
      return COLOR_NO_RESULT;
  }
}

void draw_overlay(Canvas &canvas, const Overlay &overlay) {
  const uint16_t color = state_color(overlay.state);
  canvas.outline(overlay.x - 1, overlay.y - 1, overlay.width + 2, overlay.height + 2, color);
  if (overlay.label[0] == '\0')
    return;

  const TextSize size = measure_text(overlay.label);
  int text_y = overlay.y - 2 - size.height;
  if (text_y < 1)
    text_y = overlay.y + overlay.height + 2;
  const int text_x = std::clamp(static_cast<int>(overlay.x), 1, std::max(1, canvas.width() - size.width - 1));
  canvas.fill(text_x - 1, text_y - 1, size.width + 2, size.height + 2, COLOR_TEXT_BACKGROUND);
  draw_text(canvas, text_x, text_y, overlay.label, color);
}

}  // namespace

void render_overlays(const Frame &frame, const std::vector<Overlay> &overlays, uint16_t *dst) {
  const int width = frame.width();
  const int height = frame.height();
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++)
      dst[y * width + x] = frame.get_raw(x, y);
  }
  Canvas canvas(dst, width, height);
  for (const Overlay &overlay : overlays)
    draw_overlay(canvas, overlay);
}

}  // namespace esphome::still_camera

#endif
