#include "allocator.h"

using namespace esphome::esphome_tflite;

static const char *const TAG = "tflite.allocator";

void AllocatorComponent::setup() {
  RAMAllocator<uint8_t> allocator;
  uint8_t *buf = allocator.allocate(this->size_);
  if (buf == nullptr) {
    this->mark_failed(LOG_STR("Failed to allocate tensor arena"));
    return;
  }
  this->allocator_ = MicroAllocator::Create(buf, this->size_);
  ESP_LOGI(TAG, "Initialized allocator with %u KiB tensor arena", this->size_ / 1024);
}

void AllocatorComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Allocator:");
  ESP_LOGCONFIG(TAG, "  size: %u KiB", this->size_ / 1024);
}
