#pragma once

#include <atomic>
#include <cmath>
#include <functional>

#include <esphome/components/sensor/sensor.h>
#include <esphome/components/still_camera/still_camera.h>
#include <esphome/components/text/text.h>
#include <esphome/core/component.h>
#include <esphome/core/helpers.h>
#include <esphome/core/preferences.h>

#include "interpreter.h"
#include "rect.h"

namespace esphome::esphome_tflite {

enum InputFormat {
  INPUT_FORMAT_NHWC3,
};

enum OutputFormat {
  // 10-class softmax digit model (dig-cont): outputs 10 probabilities.
  OUTPUT_FORMAT_DIGIT_SOFTMAX10,
  // 2-output sin/cos pointer model (ana-cont): outputs [sin, cos] of angle.
  OUTPUT_FORMAT_ANALOG_CONTINUOUS_CW,
};

class CameraSensor;

class CropText : public text::Text, public Parented<CameraSensor> {
 protected:
  void control(const std::string &value) override;
};

class CameraSensor : public sensor::Sensor, public Component, public still_camera::FrameProcessor {
 public:
  void set_interpreter_component(InterpreterComponent *interpreter_component) {
    this->interpreter_component_ = interpreter_component;
  }
  void set_camera(still_camera::StillCamera *camera) { this->camera_ = camera; }
  void set_crop(const Rect &crop) { this->crop_ = crop; }
  void set_crop_text(CropText *crop_text) { this->crop_text_ = crop_text; }
  /// Main loop only. Applies a crop edited through the crop text entity.
  void apply_crop_text(const std::string &value);
  void set_input_format(InputFormat fmt) { this->input_format_ = fmt; }
  void set_output_format(OutputFormat fmt) { this->output_format_ = fmt; }
  void set_min_fit(float min_fit) { this->min_fit_ = min_fit; }
  void set_max_update_interval(uint32_t max_update_interval) { this->max_update_interval_ = max_update_interval; }

  float get_last_fit() const { return this->last_fit_; }
  float get_published_fit() const { return this->published_fit_; }

  // Fired in the main loop for every completed inference, before the min_fit
  // gate, so consumers see rejected observations too.
  template<typename F> void add_on_inference_callback(F &&callback) {
    this->inference_callbacks_.add(std::forward<F>(callback));
  }

 protected:
  InterpreterComponent *interpreter_component_{nullptr};
  still_camera::StillCamera *camera_{nullptr};
  CropText *crop_text_{nullptr};
  ESPPreferenceObject crop_pref_;
  // Main loop only. The crop last accepted from the text entity.
  Rect text_crop_{};
  // Worker task only after setup.
  Rect crop_{};
  Mutex crop_mutex_;
  Rect next_crop_{};
  bool has_next_crop_{false};
  InputFormat input_format_{INPUT_FORMAT_NHWC3};
  OutputFormat output_format_{OUTPUT_FORMAT_DIGIT_SOFTMAX10};
  float min_fit_{0.0f};
  uint32_t max_update_interval_{0};
  std::atomic<uint32_t> last_update_{0};
  float last_fit_{NAN};
  float published_fit_{NAN};
  LazyCallbackManager<void(float, float, bool)> inference_callbacks_{};

  // Written by the worker task in process(), read by the main loop in publish().
  struct Pending {
    bool valid{false};
    float result{NAN};
    float fit{0.0f};
  } pending_;
  // Worker task only.
  still_camera::Overlay overlay_;

  void setup() override;

  void restore_crop_();
  void publish_crop_text_();
  bool is_due_() const;
  void set_overlay_(float result, float fit, bool has_result);

  void process(const still_camera::Frame &frame) override;
  bool get_overlay(still_camera::Overlay &overlay) const override;
  void publish() override;
};
}  // namespace esphome::esphome_tflite
