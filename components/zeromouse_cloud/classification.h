#pragma once

#include <string>

namespace esphome::zeromouse_cloud {

/// Index of an event class name. Unknown names map to 7.
inline int class_index(const std::string &name) {
  static constexpr const char *NAMES[] = {"prey", "clean", "out", "free", "early", "late", "undecidable"};
  for (int i = 0; i < 7; i++) {
    if (name == NAMES[i])
      return i;
  }
  return 7;
}

}  // namespace esphome::zeromouse_cloud
