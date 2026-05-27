#pragma once

#include <esphome/core/component.h>

#include "allocator.h"
#include "model.h"

namespace esphome::esphome_tflite {
class InterpreterComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;

  std::optional<MicroInterpreter> &interpreter() { return this->interpreter_; }

  void set_op_resolver(const MicroOpResolver &op_resolver) { this->op_resolver_ = &op_resolver; }
  void set_model_component(ModelComponent *model_component) { this->model_component_ = model_component; }
  void set_allocator_component(AllocatorComponent *allocator_component) {
    this->allocator_component_ = allocator_component;
  }

 protected:
  ModelComponent *model_component_{};
  const MicroOpResolver *op_resolver_{};
  AllocatorComponent *allocator_component_{};
  std::optional<MicroInterpreter> interpreter_{};
};

}  // namespace esphome::esphome_tflite
