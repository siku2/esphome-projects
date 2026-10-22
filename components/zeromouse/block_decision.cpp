#include "block_decision.h"

namespace esphome::zeromouse {

BlockResult decide_block(RefEvent &ref, int class_index, bool block, uint32_t now_sec, const BlockParams &params) {
  bool override_block = false;
  if (class_index >= 4 && class_index <= 6) {
    if (params.undecidable_mode == 1) {
      class_index = CLASS_CLEAN;
    } else if (params.undecidable_mode == 2) {
      ref = {CLASS_PREY, now_sec, 0};
      override_block = true;
    }
  }
  if (class_index == CLASS_PREY) {
    ref = {CLASS_PREY, now_sec, 0};
  }
  if (class_index == CLASS_CLEAN) {
    ref = {CLASS_CLEAN, 0, 0};
  }
  if (ref.time_sec != 0 && now_sec - ref.time_sec > params.ref_max_time_sec) {
    ref = {CLASS_NONE, 0, 0};
  }
  if (class_index >= 2 && ref.cls == CLASS_PREY && ref.time_sec != 0 && ref.block_count < params.ref_max_count) {
    override_block = true;
    ref.block_count++;
  }
  return {block || override_block, class_index};
}

}  // namespace esphome::zeromouse
