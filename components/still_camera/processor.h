#pragma once

#ifdef USE_ESP32

#include "frame.h"
#include "overlay.h"

namespace esphome::still_camera {

class FrameProcessor {
 public:
  virtual ~FrameProcessor() = default;

  /// Runs in the worker task, one processor after another. Must not touch main loop state.
  virtual void process(const Frame &frame) = 0;
  /// Worker task, right after process(). Returns false if the processor has nothing to draw.
  virtual bool get_overlay(Overlay &overlay) const { return false; }
  /// Runs in the main loop after every processor of the cycle finished. Publishes what process() found.
  virtual void publish() {}
};

}  // namespace esphome::still_camera

#endif
