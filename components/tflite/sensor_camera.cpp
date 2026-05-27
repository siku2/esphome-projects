#include "sensor_camera.h"

#include <esphome/core/log.h>

using namespace esphome::esphome_tflite;
using namespace esphome::camera;
using namespace esphome::camera::snapshot;

static const char *const TAG = "tflite.sensor.camera";

void CameraSnapshotSensor::setup() {
  if (this->interpreter_component_ == nullptr || this->snapshotter_ == nullptr) {
    this->mark_failed();
    return;
  }
  this->snapshotter_->add_listener(this);
}

std::tuple<float, float> read_output_digit_softmax10(const TfLiteTensor &output) {
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
    fit = val + val_minus;
  } else {
    result = static_cast<float>(max_idx) - (val_minus / (val + val_minus));
    fit = val + val_plus;
  }

  if (result >= 10.0f)
    result -= 10.0f;
  if (result < 0.0f)
    result += 10.0f;

  return {result, fit};
}

void CameraSnapshotSensor::loop() {
  if (!this->input_ready_)
    return;

  // SAFETY: interpreter must be valid if input_ready_ is true.
  MicroInterpreter &interpreter = this->interpreter_component_->interpreter().value();
  interpreter.Invoke();

  TfLiteTensor *output = interpreter.output(0);
  if (!output) {
    ESP_LOGE(TAG, "Failed to get output tensor");
    return;
  }

  float result, fit;
  switch (this->output_format_) {
    case OutputFormat::DIGIT_SOFTMAX10:
      std::tie(result, fit) = read_output_digit_softmax10(*output);
      break;
    default:
      ESP_LOGE(TAG, "Unsupported output tensor format");
      return;
  }

  this->input_ready_ = false;

  ESP_LOGI(TAG, "Model output: result=%.2f fit=%.2f", result, fit);
  this->publish_state(result);
}

void feed_tensor_nhwc3(TfLiteTensor &input, const Snapshot &snapshot, const Rect &crop) {
  if (input.type != kTfLiteFloat32) {
    ESP_LOGE(TAG, "Input tensor expected to be of type float32");
    return;
  }
  if (input.dims->size != 4) {
    ESP_LOGE(TAG, "Input tensor expected to have 4 dimensions (NHWC)");
    return;
  }
  if (input.dims->data[1] <= 0 || input.dims->data[2] <= 0) {
    ESP_LOGE(TAG, "Input tensor has invalid dimensions");
    return;
  }
  uint16_t target_h = input.dims->data[1];
  uint16_t target_w = input.dims->data[2];
  int channels = input.dims->data[3];
  if (channels != 3) {
    ESP_LOGE(TAG, "Input tensor expected to have 3 channels (RGB)");
    return;
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

      // Map crop-relative coordinates to absolute snapshot coordinates
      uint16_t abs_x = crop.top_left.x + rel_x;
      uint16_t abs_y = crop.top_left.y + rel_y;

      // Extra safety check to stay within physical snapshot boundaries
      if (abs_x >= snapshot.width())
        abs_x = snapshot.width() - 1;
      if (abs_y >= snapshot.height())
        abs_y = snapshot.height() - 1;

      Color color = snapshot.get_pixel(abs_x, abs_y);
      model_input[out_pixel_idx + 0] = static_cast<float>(color.r);
      model_input[out_pixel_idx + 1] = static_cast<float>(color.g);
      model_input[out_pixel_idx + 2] = static_cast<float>(color.b);
    }
  }
}

void CameraSnapshotSensor::on_snapshot(const Snapshot &snapshot) {
  if (this->input_ready_)
    // Still processing previous snapshot or in an error state.
    return;

  if (!this->interpreter_component_->interpreter())
    return;
  MicroInterpreter &interpreter = this->interpreter_component_->interpreter().value();
  TfLiteTensor *input = interpreter.input(0);
  if (!input) {
    ESP_LOGE(TAG, "Failed to get input tensor");
    return;
  }

  switch (this->input_format_) {
    case InputFormat::NHWC3:
      feed_tensor_nhwc3(*input, snapshot, this->crop_);
      break;
    default:
      ESP_LOGE(TAG, "Unsupported input tensor format");
      return;
  }

  this->input_ready_ = true;
}
