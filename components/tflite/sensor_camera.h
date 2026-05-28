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

enum InputFormat {
  INPUT_FORMAT_NHWC3,
};

enum OutputFormat {
  // 10-class softmax digit model (dig-cont): outputs 10 probabilities.
  OUTPUT_FORMAT_DIGIT_SOFTMAX10,
  // 2-output sin/cos pointer model (ana-cont): outputs [sin, cos] of angle.
  OUTPUT_FORMAT_ANALOG_CONTINUOUS_CW,
};

class CameraSnapshotSensor : public sensor::Sensor, public Component, public camera::snapshot::SnapshotListener {
 public:
  void set_interpreter_component(InterpreterComponent *interpreter_component) {
    this->interpreter_component_ = interpreter_component;
  }
  void set_snapshotter(camera::snapshot::Snapshotter *snapshotter) { this->snapshotter_ = snapshotter; }
  void set_crop(const Rect &crop) { this->crop_ = crop; }
  void set_input_format(InputFormat fmt) { this->input_format_ = fmt; }
  void set_output_format(OutputFormat fmt) { this->output_format_ = fmt; }

 protected:
  InterpreterComponent *interpreter_component_{nullptr};
  camera::snapshot::Snapshotter *snapshotter_{nullptr};
  Rect crop_{};
  InputFormat input_format_{INPUT_FORMAT_NHWC3};
  OutputFormat output_format_{OUTPUT_FORMAT_DIGIT_SOFTMAX10};

  void setup() override;

  void on_snapshot(const camera::snapshot::Snapshot &snapshot) override;
};
}  // namespace esphome::esphome_tflite
