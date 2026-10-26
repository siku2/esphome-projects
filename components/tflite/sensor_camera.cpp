#include "sensor_camera.h"

#include <algorithm>
#include <cmath>
#include <esphome/core/log.h>

using namespace esphome::esphome_tflite;
using esphome::Color;
using esphome::still_camera::Frame;
using esphome::still_camera::Overlay;
using esphome::still_camera::OverlayState;

static const char *const TAG = "tflite.sensor.camera";

void CameraSensor::setup() {
  if (this->interpreter_component_ == nullptr || this->camera_ == nullptr) {
    this->mark_failed();
    return;
  }
  this->restore_crop_();
  if (this->crop_.empty()) {
    this->mark_failed(LOG_STR("Crop region is empty"));
    return;
  }
  this->camera_->add_processor(this);
  this->disable_loop();
}

void CropText::control(const std::string &value) { this->parent_->apply_crop_text(value); }

void CameraSensor::publish_crop_text_() {
  char buffer[32];
  format_crop(this->text_crop_, buffer, sizeof(buffer));
  this->crop_text_->publish_state(buffer);
}

void CameraSensor::restore_crop_() {
  this->text_crop_ = this->crop_;
  if (this->crop_text_ == nullptr)
    return;
  this->crop_pref_ = global_preferences->make_preference<Rect>(this->crop_text_->get_preference_hash());
  Rect saved;
  if (this->crop_pref_.load(&saved)) {
    if (saved.fits(this->camera_->get_frame_width(), this->camera_->get_frame_height())) {
      this->crop_ = saved;
      this->text_crop_ = saved;
    } else {
      ESP_LOGW(TAG, "Ignoring the saved crop because it lies outside the frame");
    }
  }
  this->publish_crop_text_();
}

void CameraSensor::apply_crop_text(const std::string &value) {
  Rect rect;
  if (!parse_crop(value.c_str(), this->camera_->get_frame_width(), this->camera_->get_frame_height(), rect)) {
    ESP_LOGW(TAG, "Rejecting crop '%s': expected x,y,width,height inside the %ux%u frame", value.c_str(),
             this->camera_->get_frame_width(), this->camera_->get_frame_height());
    this->publish_crop_text_();
    return;
  }
  this->text_crop_ = rect;
  {
    LockGuard guard(this->crop_mutex_);
    this->next_crop_ = rect;
    this->has_next_crop_ = true;
  }
  this->crop_pref_.save(&rect);
  this->last_update_ = 0;
  this->publish_crop_text_();
  this->camera_->request_capture();
}

static std::tuple<float, float> read_output_digit_softmax10(const TfLiteTensor &output) {
  float result = nanf("");
  float fit = 0.0f;

  if (output.type != kTfLiteFloat32) {
    ESP_LOGE(TAG, "Output tensor expected to be of type float32");
    return {result, fit};
  }
  if (output.dims->size != 2) {
    ESP_LOGE(TAG, "Output tensor expected to have 2 dimensions (batch_size x num_classes)");
    return {result, fit};
  }
  if (output.dims->data[1] != 10) {
    ESP_LOGE(TAG, "Output tensor expected to have 10 classes");
    return {result, fit};
  }

  // Find the class with the highest confidence.
  size_t max_idx = 0;
  for (size_t i = 0; i < 10; ++i) {
    if (output.data.f[i] > output.data.f[max_idx]) {
      max_idx = i;
    }
  }

  float val = output.data.f[max_idx];
  float val_plus = output.data.f[(max_idx + 1) % 10];
  float val_minus = output.data.f[(max_idx + 9) % 10];

  if (val_plus > val_minus) {
    result = static_cast<float>(max_idx) + (val_plus / (val + val_plus));
    fit = val + val_plus;
  } else {
    result = static_cast<float>(max_idx) - (val_minus / (val + val_minus));
    fit = val + val_minus;
  }

  // Round to 1 decimal place
  result = roundf(10.0f * result) / 10.0f;
  if (result >= 10.0f)
    result -= 10.0f;
  if (result < 0.0f)
    result += 10.0f;
  if (result == 0.0f)
    result = 0.0f;

  return {result, fit};
}

static std::tuple<float, float> read_output_analog_continuous(const TfLiteTensor &output, bool ccw) {
  float result = nanf("");
  float fit = 0.0f;
  if (output.type != kTfLiteFloat32) {
    ESP_LOGE(TAG, "Analog output tensor expected to be float32");
    return {result, fit};
  }
  // Expect at least 2 float elements (shape [1,2] or [2])
  if (output.bytes < 2 * sizeof(float)) {
    ESP_LOGE(TAG, "Analog output tensor too small (need at least 2 floats)");
    return {result, fit};
  }

  float f1 = output.data.f[0];
  float f2 = output.data.f[1];
  float val = fmodf(atan2f(f1, f2) / (M_PI * 2.0f) + 2.0f, 1.0f);
  result = ccw ? (10.0f - val * 10.0f) : (val * 10.0f);
  // Round to 2 decimal places
  result = roundf(100.0f * result) / 100.0f;
  fit = std::min(1.0f, sqrtf(f1 * f1 + f2 * f2));
  return {result, fit};
}

static bool feed_tensor_nhwc3(TfLiteTensor &input, const Frame &frame, const Rect &crop) {
  if (input.type != kTfLiteFloat32) {
    ESP_LOGE(TAG, "Input tensor expected to be of type float32");
    return false;
  }
  if (input.dims->size != 4) {
    ESP_LOGE(TAG, "Input tensor expected to have 4 dimensions (NHWC)");
    return false;
  }
  if (input.dims->data[1] <= 0 || input.dims->data[2] <= 0) {
    ESP_LOGE(TAG, "Input tensor has invalid dimensions");
    return false;
  }
  uint16_t target_h = input.dims->data[1];
  uint16_t target_w = input.dims->data[2];
  int channels = input.dims->data[3];
  if (channels != 3) {
    ESP_LOGE(TAG, "Input tensor expected to have 3 channels (RGB)");
    return false;
  }
  if (input.bytes < static_cast<size_t>(target_h) * target_w * channels * sizeof(float)) {
    ESP_LOGE(TAG, "Input tensor is smaller than its dimensions");
    return false;
  }
  if (crop.bottom_right.x <= crop.top_left.x || crop.bottom_right.y <= crop.top_left.y ||
      crop.bottom_right.x > frame.width() || crop.bottom_right.y > frame.height()) {
    ESP_LOGE(TAG, "Crop region lies outside the %ux%u frame", frame.width(), frame.height());
    return false;
  }

  uint16_t crop_w = crop.width();
  uint16_t crop_h = crop.height();

  float target_aspect = (float) target_w / target_h;
  float crop_aspect = (float) crop_w / crop_h;

  float scale;
  if (crop_aspect > target_aspect) {
    scale = (float) target_w / crop_w;
  } else {
    scale = (float) target_h / crop_h;
  }

  uint16_t scaled_w = crop_w * scale;
  uint16_t scaled_h = crop_h * scale;

  uint16_t pad_x = (target_w - scaled_w) / 2;
  uint16_t pad_y = (target_h - scaled_h) / 2;

  float *model_input = input.data.f;
  for (uint16_t y = 0; y < target_h; ++y) {
    for (uint16_t x = 0; x < target_w; ++x) {
      size_t out_pixel_idx = ((y * target_w) + x) * channels;

      // Check if we are in the letterbox padding zone
      if (x < pad_x || x >= (pad_x + scaled_w) || y < pad_y || y >= (pad_y + scaled_h)) {
        model_input[out_pixel_idx + 0] = 0.0f;
        model_input[out_pixel_idx + 1] = 0.0f;
        model_input[out_pixel_idx + 2] = 0.0f;
        continue;
      }

      // Map target coordinates back to crop-relative coordinates (Nearest Neighbor)
      uint16_t rel_x = (x - pad_x) / scale;
      uint16_t rel_y = (y - pad_y) / scale;

      // Guard rails to prevent out-of-bounds mapping due to floating point rounding
      if (rel_x >= crop_w)
        rel_x = crop_w - 1;
      if (rel_y >= crop_h)
        rel_y = crop_h - 1;

      // Map crop-relative coordinates to absolute frame coordinates
      uint16_t abs_x = crop.top_left.x + rel_x;
      uint16_t abs_y = crop.top_left.y + rel_y;

      // Extra safety check to stay within physical frame boundaries
      if (abs_x >= frame.width())
        abs_x = frame.width() - 1;
      if (abs_y >= frame.height())
        abs_y = frame.height() - 1;

      Color color = frame.get_pixel(abs_x, abs_y);
      model_input[out_pixel_idx + 0] = static_cast<float>(color.r);
      model_input[out_pixel_idx + 1] = static_cast<float>(color.g);
      model_input[out_pixel_idx + 2] = static_cast<float>(color.b);
    }
  }
  return true;
}

bool CameraSensor::is_due_() const {
  const uint32_t last_update = this->last_update_;
  if (this->max_update_interval_ == 0 || last_update == 0)
    return true;
  return (millis() - last_update) >= this->max_update_interval_;
}

void CameraSensor::set_overlay_(float result, float fit, bool has_result) {
  this->overlay_.x = this->crop_.top_left.x;
  this->overlay_.y = this->crop_.top_left.y;
  this->overlay_.width = this->crop_.width();
  this->overlay_.height = this->crop_.height();
  this->overlay_.label[0] = '\0';
  if (!has_result) {
    this->overlay_.state = OverlayState::NO_RESULT;
    return;
  }
  this->overlay_.state = fit >= this->min_fit_ ? OverlayState::ACCEPTED : OverlayState::REJECTED;
  const int decimals = this->output_format_ == OUTPUT_FORMAT_ANALOG_CONTINUOUS_CW ? 2 : 1;
  snprintf(this->overlay_.label, sizeof(this->overlay_.label), "%.*f\n%.0f%%", decimals, result, 100.0f * fit);
}

bool CameraSensor::get_overlay(Overlay &overlay) const {
  overlay = this->overlay_;
  return true;
}

void CameraSensor::process(const Frame &frame) {
  {
    LockGuard guard(this->crop_mutex_);
    if (this->has_next_crop_) {
      this->crop_ = this->next_crop_;
      this->has_next_crop_ = false;
    }
  }
  this->pending_.valid = false;
  this->set_overlay_(NAN, 0.0f, false);
  if (this->is_failed() || !this->is_due_())
    return;

  auto &interpreter_opt = this->interpreter_component_->interpreter();
  if (this->interpreter_component_->is_failed() || !interpreter_opt.has_value())
    return;
  MicroInterpreter &interpreter = *interpreter_opt;

  TfLiteTensor *input = interpreter.input(0);
  if (!input) {
    ESP_LOGE(TAG, "Failed to get input tensor");
    return;
  }

  switch (this->input_format_) {
    case INPUT_FORMAT_NHWC3:
      if (!feed_tensor_nhwc3(*input, frame, this->crop_)) {
        ESP_LOGW(TAG, "Skipping inference: could not prepare the input tensor");
        return;
      }
      break;
    default:
      ESP_LOGE(TAG, "Unsupported input tensor format");
      return;
  }

  if (interpreter.Invoke() != kTfLiteOk) {
    ESP_LOGE(TAG, "Failed to invoke interpreter");
    return;
  }

  TfLiteTensor *output = interpreter.output(0);
  if (!output) {
    ESP_LOGE(TAG, "Failed to get output tensor");
    return;
  }

  float result;
  float fit;
  switch (this->output_format_) {
    case OUTPUT_FORMAT_DIGIT_SOFTMAX10:
      std::tie(result, fit) = read_output_digit_softmax10(*output);
      break;
    case OUTPUT_FORMAT_ANALOG_CONTINUOUS_CW:
      std::tie(result, fit) = read_output_analog_continuous(*output, false);
      break;
    default:
      ESP_LOGE(TAG, "Unsupported output tensor format");
      return;
  }

  ESP_LOGD(TAG, "Model output: result=%.6f fit=%.2f", result, fit);
  if (std::isnan(result))
    return;
  this->set_overlay_(result, fit, true);
  this->pending_.result = result;
  this->pending_.fit = fit;
  this->pending_.valid = true;
}

void CameraSensor::publish() {
  if (!this->pending_.valid)
    return;
  this->pending_.valid = false;
  const float result = this->pending_.result;
  const float fit = this->pending_.fit;
  this->last_fit_ = fit;
  this->inference_callbacks_.call(result, fit, fit >= this->min_fit_);
  if (fit < this->min_fit_) {
    ESP_LOGW(TAG, "Rejecting result %.6f: fit %.2f is below min_fit %.2f", result, fit, this->min_fit_);
    return;
  }
  this->published_fit_ = fit;
  this->publish_state(result);
  this->last_update_ = millis();
}
