#include "model.h"

using namespace esphome::esphome_tflite;

static const char *const TAG = "tflite.model";

void ModelComponent::setup() {
  if (this->buf_ == nullptr) {
    this->mark_failed();
    return;
  }

  flatbuffers::Verifier verifier(this->buf_, this->size_);
  if (!VerifyModelBuffer(verifier)) {
    this->mark_failed(LOG_STR("Model buffer verification failed"));
    return;
  }
  this->model_ = GetModel(this->buf_);
  assert(this->model_ != nullptr);
}

void ModelComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Model:");
  if (this->model_ == nullptr)
    return;

  ESP_LOGCONFIG(TAG, "  version: %u", this->model_->version());
  if (this->model_->description() != nullptr) {
    ESP_LOGCONFIG(TAG, "  description: '%s'", this->model_->description()->c_str());
  }
}
