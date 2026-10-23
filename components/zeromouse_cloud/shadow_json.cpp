#include "shadow_json.h"

#include <cctype>

#include "json_string.h"

namespace esphome::zeromouse_cloud {

namespace {

// Appends `,"name":value` for the second and later fields.
class ObjectWriter {
 public:
  explicit ObjectWriter(std::string &out) : out_(out) {}

  void number(const char *name, int64_t value) {
    this->key_(name);
    this->out_ += std::to_string(value);
  }
  void string(const char *name, const std::string &value) {
    this->key_(name);
    append_json_string(this->out_, value);
  }

 protected:
  void key_(const char *name) {
    if (!this->first_)
      this->out_ += ',';
    this->first_ = false;
    this->out_ += '"';
    this->out_ += name;
    this->out_ += "\":";
  }

  std::string &out_;
  bool first_{true};
};

void write_system(std::string &out, const SystemShadow &s) {
  out += "\"system\":{";
  ObjectWriter w(out);
  w.string("ownerID", s.owner_id);
  w.string("identityID", s.identity_id);
  w.number("finaleIntervalMaxCount", s.finale_interval_max_count);
  w.number("finaleIntervalDelta", s.finale_interval_delta);
  w.number("cleanEventTime", s.clean_event_time);
  w.number("undecidableMode", s.undecidable_mode);
  w.number("verMajor", s.ver_major);
  w.number("verMinor", s.ver_minor);
  w.number("verRevision", s.ver_revision);
  w.number("uploadEmpty", s.upload_empty);
  w.number("logEnabled", s.log_enabled);
  w.number("kkzmBlockModeIn", s.kkzm_block_mode_in);
  w.number("kkzmBlockModeOut", s.kkzm_block_mode_out);
  w.number("kkzmOutCloseAt", s.kkzm_out_close_at);
  w.number("kkzmOutOpenAt", s.kkzm_out_open_at);
  w.string("timeZone", s.time_zone);
  w.number("rebootTimer", s.reboot_timer);
  out += '}';
}

void write_camera(std::string &out, const CameraShadow &c) {
  out += "\"camera\":{";
  ObjectWriter w(out);
  w.number("brightness", c.brightness);
  w.number("autoExpLevel", c.auto_exp_level);
  w.number("autoGainCtrlMin", c.auto_gain_ctrl_min);
  w.number("autoGainCtrlMax", c.auto_gain_ctrl_max);
  w.number("autoGainStblMin", c.auto_gain_stbl_min);
  w.number("autoGainStblMax", c.auto_gain_stbl_max);
  w.number("gainCeiling", c.gain_ceiling);
  w.number("camShotsBeforeT0", c.cam_shots_before_t0);
  w.number("cameraStatus", c.camera_status);
  w.number("sensorId", c.sensor_id);
  out += '}';
}

void write_proximity(std::string &out, const ProximityShadow &p) {
  out += "\"proximity\":{";
  ObjectWriter w(out);
  w.number("irFreeValue", p.ir_free_value);
  w.number("irFreeValueRaw", p.ir_free_value_raw);
  w.number("irRangeIgnoreEvent", p.ir_range_ignore_event);
  w.number("irPreyEventTime", p.ir_prey_event_time);
  w.number("irMedianCount", p.ir_median_count);
  w.number("irAmbient", p.ir_ambient);
  w.number("irSlopeThreshValue", p.ir_slope_thresh_value);
  w.number("irSensorStatus", p.ir_sensor_status);
  w.number("irMotionThrsh", p.ir_motion_thresh);
  w.number("irPresenceThrsh", p.ir_presence_thresh);
  out += '}';
}

void write_rfid(std::string &out, const RfidShadow &r) {
  out += "\"rfid\":{";
  ObjectWriter w(out);
  w.number("blockEnabled", r.block_enabled);
  w.number("responseTimeout", r.response_timeout);
  w.number("lastRefEventMaxCount", r.last_ref_event_max_count);
  w.number("lastRefEventMaxTimeSec", r.last_ref_event_max_time_sec);
  out += '}';
}

}  // namespace

std::string build_reported_shadow(const ShadowState &state) {
  std::string out = "{\"state\":{\"reported\":{";
  write_system(out, state.system);
  out += ',';
  write_camera(out, state.camera);
  out += ',';
  write_proximity(out, state.proximity);
  out += ',';
  write_rfid(out, state.rfid);
  out += "}}}";
  return out;
}

std::string build_metrics(const ShadowState &state, const MetricsInfo &info) {
  const SystemShadow &s = state.system;
  std::string out = "{\"deviceID\":";
  append_json_string(out, info.device_id);
  // The hardware version is the character after "-H" in the thing name.
  const size_t pos = info.thing_name.find("-H");
  if (pos != std::string::npos && pos + 2 < info.thing_name.size() && std::isdigit(info.thing_name[pos + 2]))
    out += ",\"verHardware\":" + info.thing_name.substr(pos + 2, 1);
  out += ",\"verMajor\":" + std::to_string(s.ver_major);
  out += ",\"verMinor\":" + std::to_string(s.ver_minor);
  out += ",\"verRevision\":" + std::to_string(s.ver_revision);
  out += ",\"bootCount\":" + std::to_string(info.boot_count);
  out += ",\"eventCount\":" + std::to_string(info.event_count);
  out += ",\"ownerID\":";
  append_json_string(out, s.owner_id);
  out += ",\"identityID\":";
  append_json_string(out, s.identity_id);
  out += ",\"finaleIntervalMaxCount\":" + std::to_string(s.finale_interval_max_count);
  out += ",\"finaleIntervalDelta\":" + std::to_string(s.finale_interval_delta);
  out += '}';
  return out;
}

}  // namespace esphome::zeromouse_cloud
