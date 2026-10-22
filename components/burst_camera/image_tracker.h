#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>

namespace esphome::burst_camera {

/// Counts live images and wakes the encoder task when one is released.
class ImageTracker {
 public:
  void set_task(TaskHandle_t task) { this->task_ = task; }

  void acquire() { this->count_.fetch_add(1); }
  void release() {
    this->count_.fetch_sub(1);
    if (this->task_ != nullptr)
      xTaskNotifyGive(this->task_);
  }
  bool idle() const { return this->count_.load() == 0; }

 protected:
  std::atomic<uint16_t> count_{0};
  TaskHandle_t task_{nullptr};
};

}  // namespace esphome::burst_camera

#endif
