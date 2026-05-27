#include "decoder.h"

#include <esp_heap_caps.h>

static const char *const TAG = "camera_snapshot.decoder";

using namespace esphome::camera::snapshot;

Decoder::Decoder(jpeg_pixel_format_t output_format) : output_format_(output_format) {}

Decoder::~Decoder() {
  if (this->jpeg_dec_ != nullptr) {
    jpeg_dec_close(this->jpeg_dec_);
    this->jpeg_dec_ = nullptr;
  }
  if (this->outbuf_ != nullptr) {
    heap_caps_free(this->outbuf_);
    this->outbuf_ = nullptr;
    this->outbuf_len_ = 0;
  }
}

jpeg_error_t Decoder::decode(uint8_t *buf, size_t len, jpeg_rotate_t rotate) {
  jpeg_error_t ret = JPEG_ERR_OK;

  if (this->last_rotate_ != rotate && this->jpeg_dec_ != nullptr) {
    // We want a different rotation...
    jpeg_dec_close(this->jpeg_dec_);
    this->jpeg_dec_ = nullptr;
  }

  if (this->jpeg_dec_ == nullptr) {
    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = this->output_format_;
    config.rotate = rotate;
    ret = jpeg_dec_open(&config, &this->jpeg_dec_);
    if (ret != JPEG_ERR_OK)
      return ret;
  }

  this->jpeg_io_.inbuf = buf;
  this->jpeg_io_.inbuf_len = len;

  ret = jpeg_dec_parse_header(this->jpeg_dec_, &this->jpeg_io_, &this->out_info_);
  if (ret != JPEG_ERR_OK)
    return ret;

  size_t required_len = this->out_info_.width * this->out_info_.height;
  switch (this->output_format_) {
    case JPEG_PIXEL_FORMAT_RGB888:
      required_len *= 3;
      break;
    case JPEG_PIXEL_FORMAT_RGB565_BE:
    // fallthrough
    case JPEG_PIXEL_FORMAT_RGB565_LE:
      required_len *= 2;
      break;
    default:
      return JPEG_ERR_INVALID_PARAM;
  }

  if (this->outbuf_len_ < required_len) {
    if (this->outbuf_ != nullptr) {
      ESP_LOGI(TAG, "Re-allocating output buffer from %zu to %zu bytes", this->outbuf_len_, required_len);
      heap_caps_free(this->outbuf_);
    }
    this->outbuf_ = (uint8_t *) heap_caps_aligned_alloc(16, required_len, MALLOC_CAP_SPIRAM);
    this->outbuf_len_ = required_len;
  }

  this->jpeg_io_.outbuf = this->outbuf_;
  return jpeg_dec_process(this->jpeg_dec_, &this->jpeg_io_);
}

Snapshot Decoder::get_snapshot() const {
  Snapshot snapshot;
  snapshot.width_ = this->out_info_.width;
  snapshot.height_ = this->out_info_.height;
  snapshot.pixel_format_ = this->output_format_;
  snapshot.data_ = this->outbuf_;
  snapshot.data_len_ = this->outbuf_len_;
  return snapshot;
}
