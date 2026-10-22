#include "settings.h"

#include <cstring>

namespace esphome::zeromouse_cloud {

namespace {

const SettingInfo SETTINGS[] = {
    {Setting::CLEAN_EVENT_TIME, "system", "cleanEventTime"},
    {Setting::UNDECIDABLE_MODE, "system", "undecidableMode"},
    {Setting::FINALE_INTERVAL_MAX_COUNT, "system", "finaleIntervalMaxCount"},
    {Setting::FINALE_INTERVAL_DELTA, "system", "finaleIntervalDelta"},
    {Setting::UPLOAD_EMPTY, "system", "uploadEmpty"},
    {Setting::LOG_ENABLED, "system", "logEnabled"},
    {Setting::AUTO_EXP_LEVEL, "camera", "autoExpLevel"},
    {Setting::GAIN_CEILING, "camera", "gainCeiling"},
    {Setting::BRIGHTNESS, "camera", "brightness"},
    {Setting::AUTO_GAIN_CTRL_MIN, "camera", "autoGainCtrlMin"},
    {Setting::AUTO_GAIN_CTRL_MAX, "camera", "autoGainCtrlMax"},
    {Setting::AUTO_GAIN_STBL_MIN, "camera", "autoGainStblMin"},
    {Setting::AUTO_GAIN_STBL_MAX, "camera", "autoGainStblMax"},
    {Setting::CAM_SHOTS_BEFORE_T0, "camera", "camShotsBeforeT0"},
    {Setting::IR_RANGE_IGNORE_EVENT, "proximity", "irRangeIgnoreEvent"},
    {Setting::IR_PREY_EVENT_TIME, "proximity", "irPreyEventTime"},
    {Setting::IR_MEDIAN_COUNT, "proximity", "irMedianCount"},
    {Setting::IR_AMBIENT, "proximity", "irAmbient"},
    {Setting::IR_SLOPE_THRESH_VALUE, "proximity", "irSlopeThreshValue"},
    {Setting::IR_MOTION_THRESH, "proximity", "irMotionThrsh"},
    {Setting::IR_PRESENCE_THRESH, "proximity", "irPresenceThrsh"},
    {Setting::BLOCK_ENABLED, "rfid", "blockEnabled"},
    {Setting::RESPONSE_TIMEOUT, "rfid", "responseTimeout"},
    {Setting::LAST_REF_EVENT_MAX_COUNT, "rfid", "lastRefEventMaxCount"},
    {Setting::LAST_REF_EVENT_MAX_TIME_SEC, "rfid", "lastRefEventMaxTimeSec"},
};

}  // namespace

const SettingInfo *find_setting(const char *section, const char *key) {
  for (const SettingInfo &info : SETTINGS) {
    if (strcmp(info.section, section) == 0 && strcmp(info.key, key) == 0)
      return &info;
  }
  return nullptr;
}

#define SETTING_FIELDS(X) \
  X(CLEAN_EVENT_TIME, system.clean_event_time) \
  X(UNDECIDABLE_MODE, system.undecidable_mode) \
  X(FINALE_INTERVAL_MAX_COUNT, system.finale_interval_max_count) \
  X(FINALE_INTERVAL_DELTA, system.finale_interval_delta) \
  X(UPLOAD_EMPTY, system.upload_empty) \
  X(LOG_ENABLED, system.log_enabled) \
  X(AUTO_EXP_LEVEL, camera.auto_exp_level) \
  X(GAIN_CEILING, camera.gain_ceiling) \
  X(BRIGHTNESS, camera.brightness) \
  X(AUTO_GAIN_CTRL_MIN, camera.auto_gain_ctrl_min) \
  X(AUTO_GAIN_CTRL_MAX, camera.auto_gain_ctrl_max) \
  X(AUTO_GAIN_STBL_MIN, camera.auto_gain_stbl_min) \
  X(AUTO_GAIN_STBL_MAX, camera.auto_gain_stbl_max) \
  X(CAM_SHOTS_BEFORE_T0, camera.cam_shots_before_t0) \
  X(IR_RANGE_IGNORE_EVENT, proximity.ir_range_ignore_event) \
  X(IR_PREY_EVENT_TIME, proximity.ir_prey_event_time) \
  X(IR_MEDIAN_COUNT, proximity.ir_median_count) \
  X(IR_AMBIENT, proximity.ir_ambient) \
  X(IR_SLOPE_THRESH_VALUE, proximity.ir_slope_thresh_value) \
  X(IR_MOTION_THRESH, proximity.ir_motion_thresh) \
  X(IR_PRESENCE_THRESH, proximity.ir_presence_thresh) \
  X(BLOCK_ENABLED, rfid.block_enabled) \
  X(RESPONSE_TIMEOUT, rfid.response_timeout) \
  X(LAST_REF_EVENT_MAX_COUNT, rfid.last_ref_event_max_count) \
  X(LAST_REF_EVENT_MAX_TIME_SEC, rfid.last_ref_event_max_time_sec)

int64_t get_setting(const ShadowState &state, Setting setting) {
  switch (setting) {
#define X(name, field) \
  case Setting::name: \
    return static_cast<int64_t>(state.field);
    SETTING_FIELDS(X)
#undef X
  }
  return 0;
}

void set_setting(ShadowState &state, Setting setting, int64_t value) {
  switch (setting) {
#define X(name, field) \
  case Setting::name: \
    state.field = static_cast<decltype(state.field)>(value); \
    break;
    SETTING_FIELDS(X)
#undef X
  }
}

}  // namespace esphome::zeromouse_cloud
