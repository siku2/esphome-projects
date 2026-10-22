#pragma once

#include <cstdint>

namespace esphome::zeromouse {

static constexpr int CLASS_NONE = -1;
static constexpr int CLASS_PREY = 0;
static constexpr int CLASS_CLEAN = 1;

/// The last classification that is remembered across events. It is kept in RAM only.
struct RefEvent {
  int32_t cls{CLASS_NONE};
  uint32_t time_sec{0};
  uint32_t block_count{0};

  bool operator==(const RefEvent &other) const {
    return this->cls == other.cls && this->time_sec == other.time_sec && this->block_count == other.block_count;
  }
};

struct BlockParams {
  /// 0 is normal, 1 treats undecidable results as clean, 2 blocks on them.
  int undecidable_mode{0};
  uint32_t ref_max_time_sec{1800};
  uint32_t ref_max_count{10};
};

struct BlockResult {
  bool block;
  /// The class after the undecidable mode was applied.
  int class_index;
};

/// Decides if the blocker runs for a classification and updates the reference event.
BlockResult decide_block(RefEvent &ref, int class_index, bool block, uint32_t now_sec, const BlockParams &params);

}  // namespace esphome::zeromouse
