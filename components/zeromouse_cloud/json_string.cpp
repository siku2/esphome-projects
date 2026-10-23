#include "json_string.h"

#include <cstdio>

namespace esphome::zeromouse_cloud {

void append_json_string(std::string &out, const std::string &value) {
  out += '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buffer[7];
          snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned char>(c));
          out += buffer;
        } else {
          out += c;
        }
    }
  }
  out += '"';
}

}  // namespace esphome::zeromouse_cloud
