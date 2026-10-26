#pragma once

#include <cstdint>

namespace esphome::still_camera {

static constexpr uint8_t FONT_WIDTH = 3;
static constexpr uint8_t FONT_HEIGHT = 5;

/// Returns FONT_HEIGHT rows. The three low bits of a row are its pixels, the highest of them is the left one.
inline const uint8_t *font_glyph(char c) {
  static constexpr uint8_t BLANK[FONT_HEIGHT] = {0, 0, 0, 0, 0};
  static constexpr uint8_t DIGITS[10][FONT_HEIGHT] = {
      {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
      {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 2, 2}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7},
  };
  static constexpr uint8_t DOT[FONT_HEIGHT] = {0, 0, 0, 0, 2};
  static constexpr uint8_t PERCENT[FONT_HEIGHT] = {5, 1, 2, 4, 5};
  static constexpr uint8_t MINUS[FONT_HEIGHT] = {0, 0, 7, 0, 0};
  static constexpr uint8_t LETTER_A[FONT_HEIGHT] = {2, 5, 7, 5, 5};
  static constexpr uint8_t LETTER_E[FONT_HEIGHT] = {7, 4, 7, 4, 7};
  static constexpr uint8_t LETTER_K[FONT_HEIGHT] = {5, 5, 6, 5, 5};
  static constexpr uint8_t LETTER_N[FONT_HEIGHT] = {6, 5, 5, 5, 5};
  static constexpr uint8_t LETTER_R[FONT_HEIGHT] = {6, 5, 6, 5, 5};
  if (c >= '0' && c <= '9')
    return DIGITS[c - '0'];
  switch (c) {
    case '.':
      return DOT;
    case '%':
      return PERCENT;
    case '-':
      return MINUS;
    case 'A':
      return LETTER_A;
    case 'E':
      return LETTER_E;
    case 'K':
      return LETTER_K;
    case 'N':
      return LETTER_N;
    case 'R':
      return LETTER_R;
    default:
      return BLANK;
  }
}

}  // namespace esphome::still_camera
