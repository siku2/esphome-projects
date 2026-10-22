#pragma once

#include <cstdint>
#include <string>

namespace esphome::zeromouse_cloud {

struct SystemShadow {
  std::string owner_id;
  std::string identity_id;
  uint32_t finale_interval_max_count{12};
  uint32_t finale_interval_delta{5};
  uint32_t clean_event_time{30};
  uint16_t undecidable_mode{0};
  uint8_t ver_major{0};
  uint8_t ver_minor{17};
  uint8_t ver_revision{12};
  bool upload_empty{false};
  bool log_enabled{false};
  uint8_t kkzm_block_mode_in{0};
  uint8_t kkzm_block_mode_out{0};
  uint16_t kkzm_out_close_at{0};
  uint16_t kkzm_out_open_at{0};
  std::string time_zone;
  uint32_t reboot_timer{12};
};

struct CameraShadow {
  int32_t brightness{2};
  uint32_t auto_exp_level{64};
  uint32_t auto_gain_ctrl_min{112};
  uint32_t auto_gain_ctrl_max{144};
  uint32_t auto_gain_stbl_min{120};
  uint32_t auto_gain_stbl_max{136};
  uint32_t gain_ceiling{3};
  uint32_t cam_shots_before_t0{0};
  uint32_t camera_status{0};
  /// 0 unknown, 1 OV7670, 2 GC032A.
  uint32_t sensor_id{1};
  /// Only used for the sensor reference of an event.
  int32_t exposure_ctrl{0};
};

struct ProximityShadow {
  uint32_t ir_free_value{50};
  uint32_t ir_free_value_raw{50};
  uint32_t ir_range_ignore_event{100};
  uint32_t ir_prey_event_time{30};
  uint32_t ir_median_count{5};
  uint32_t ir_ambient{1000};
  int32_t ir_slope_thresh_value{20};
  uint32_t ir_sensor_status{0};
  uint32_t ir_motion_thresh{0};
  uint32_t ir_presence_thresh{0};
};

struct RfidShadow {
  bool block_enabled{true};
  uint32_t response_timeout{25000};
  uint32_t last_ref_event_max_count{10};
  uint32_t last_ref_event_max_time_sec{1800};
};

/// The reported shadow of the device. The defaults are the stock V1 board values.
struct ShadowState {
  SystemShadow system;
  CameraShadow camera;
  ProximityShadow proximity;
  RfidShadow rfid;
};

}  // namespace esphome::zeromouse_cloud
