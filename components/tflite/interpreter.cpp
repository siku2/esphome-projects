#include "interpreter.h"

using namespace esphome::esphome_tflite;

static const char *const TAG = "tflite.interpreter";

void InterpreterComponent::setup() {
  if (this->model_component_ == nullptr || this->op_resolver_ == nullptr || this->allocator_component_ == nullptr) {
    this->mark_failed();
    return;
  }

  if (this->allocator_component_->is_failed()) {
    this->mark_failed(LOG_STR("Allocator component failed"));
    return;
  }

  if (this->model_component_->is_failed()) {
    this->mark_failed(LOG_STR("Model component failed"));
    return;
  }

  TfLiteStatus status;
  this->interpreter_.emplace(this->model_component_->get_model(), *this->op_resolver_,
                             this->allocator_component_->get_allocator());
  status = this->interpreter_->AllocateTensors();
  if (status != kTfLiteOk) {
    ESP_LOGE(TAG, "AllocateTensors() failed with error code %d", static_cast<int>(status));
    this->mark_failed(LOG_STR("Failed to allocate tensors"));
    return;
  }
  ESP_LOGI(TAG, "Interpreter setup complete with %d inputs and %d outputs", this->interpreter_->inputs_size(),
           this->interpreter_->outputs_size());
}

void InterpreterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Interpreter:");
  if (this->interpreter_.has_value()) {
    ESP_LOGCONFIG(TAG, "  inputs: %d", this->interpreter_->inputs_size());
  }
}
