#pragma once

#include <esphome/components/camera/buffer_impl.h>
#include <esphome/components/camera/camera.h>
#include <esphome/components/camera_snapshot/snapshot.h>
#include <esphome/components/camera_snapshot/snapshotter.h>
#include <esphome/components/sensor/sensor.h>
#include <esphome/core/component.h>

#include "interpreter.h"

namespace esphome::esphome_tflite {

struct Point {
  uint16_t x;
  uint16_t y;
};

struct Rect {
  Point top_left;
  Point bottom_right;

  uint16_t width() const { return this->bottom_right.x - this->top_left.x; }
  uint16_t height() const { return this->bottom_right.y - this->top_left.y; }
  bool empty() const { return this->width() == 0 || this->height() == 0; }
};

enum class InputFormat {
  NHWC3,
};

enum class OutputFormat {
  DIGIT_SOFTMAX10,
};

class CameraSnapshotSensor : public sensor::Sensor, public Component, public camera::snapshot::SnapshotListener {
 public:
  void set_interpreter_component(InterpreterComponent *interpreter_component) {
    this->interpreter_component_ = interpreter_component;
  }
  void set_snapshotter(camera::snapshot::Snapshotter *snapshotter) { this->snapshotter_ = snapshotter; }
  void set_crop(const Rect &crop) { this->crop_ = crop; }

 protected:
  InterpreterComponent *interpreter_component_{nullptr};
  camera::snapshot::Snapshotter *snapshotter_{nullptr};
  Rect crop_{};
  InputFormat input_format_{InputFormat::NHWC3};
  OutputFormat output_format_{OutputFormat::DIGIT_SOFTMAX10};
  bool input_ready_{false};

  void setup() override;
  void loop() override;

  void on_snapshot(const camera::snapshot::Snapshot &snapshot) override;
};
}  // namespace esphome::esphome_tflite
