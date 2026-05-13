#pragma once

#include <esphome/core/component.h>

#include "tflite.h"

namespace esphome::esphome_tflite {
class ModelComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

  void set_buf(const uint8_t *buf, size_t size) {
    this->buf_ = buf;
    this->size_ = size;
  }

  const Model *get_model() const { return this->model_; }

 protected:
  const uint8_t *buf_{};
  size_t size_{};
  const Model *model_{};
};

}  // namespace esphome::esphome_tflite
