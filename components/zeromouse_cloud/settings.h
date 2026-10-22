#pragma once

#include <cstddef>
#include <cstdint>

#include "shadow_state.h"

namespace esphome::zeromouse_cloud {

/// Shadow fields that can be bound to a local entity.
enum class Setting : uint8_t {
  CLEAN_EVENT_TIME,
  UNDECIDABLE_MODE,
  FINALE_INTERVAL_MAX_COUNT,
  FINALE_INTERVAL_DELTA,
  UPLOAD_EMPTY,
  LOG_ENABLED,
  AUTO_EXP_LEVEL,
  GAIN_CEILING,
  BRIGHTNESS,
  AUTO_GAIN_CTRL_MIN,
  AUTO_GAIN_CTRL_MAX,
  AUTO_GAIN_STBL_MIN,
  AUTO_GAIN_STBL_MAX,
  CAM_SHOTS_BEFORE_T0,
  IR_RANGE_IGNORE_EVENT,
  IR_PREY_EVENT_TIME,
  IR_MEDIAN_COUNT,
  IR_AMBIENT,
  IR_SLOPE_THRESH_VALUE,
  IR_MOTION_THRESH,
  IR_PRESENCE_THRESH,
  BLOCK_ENABLED,
  RESPONSE_TIMEOUT,
  LAST_REF_EVENT_MAX_COUNT,
  LAST_REF_EVENT_MAX_TIME_SEC,
};

struct SettingInfo {
  Setting setting;
  const char *section;
  const char *key;
};

/// Returns nullptr if the section and key are not a known setting.
const SettingInfo *find_setting(const char *section, const char *key);

int64_t get_setting(const ShadowState &state, Setting setting);
void set_setting(ShadowState &state, Setting setting, int64_t value);

}  // namespace esphome::zeromouse_cloud
