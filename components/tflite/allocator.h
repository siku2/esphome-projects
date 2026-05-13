#pragma once

#include <esphome/core/component.h>

#include "tflite.h"

namespace esphome::esphome_tflite {
class AllocatorComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA + 1.0f; }

  void set_size(size_t size) { this->size_ = size; }
  MicroAllocator *get_allocator() const { return this->allocator_; }

 protected:
  size_t size_{0};
  MicroAllocator *allocator_{nullptr};
};

}  // namespace esphome::esphome_tflite
