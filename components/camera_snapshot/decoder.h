#pragma once

#include <esp_jpeg_dec.h>

#include "snapshot.h"

namespace esphome::camera::snapshot {
class Decoder {
 public:
  Decoder() = delete;
  explicit Decoder(jpeg_pixel_format_t output_format);
  ~Decoder();

  jpeg_error_t decode(uint8_t *buf, size_t len);
  Snapshot get_snapshot() const;

 protected:
  jpeg_pixel_format_t output_format_;
  jpeg_dec_handle_t jpeg_dec_{nullptr};
  jpeg_dec_io_t jpeg_io_{};
  jpeg_dec_header_info_t out_info_{};
  uint8_t *outbuf_{nullptr};
  size_t outbuf_len_{0};
};
}  // namespace esphome::camera::snapshot
