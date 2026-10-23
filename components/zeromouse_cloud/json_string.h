#pragma once

#include <string>

namespace esphome::zeromouse_cloud {

/// Appends value as a quoted JSON string with escaping.
void append_json_string(std::string &out, const std::string &value);

}  // namespace esphome::zeromouse_cloud
