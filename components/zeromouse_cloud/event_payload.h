#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>
#include <vector>

#include "esphome/core/helpers.h"

namespace esphome::zeromouse_cloud {

/// A sample of the IR scan.
struct AirPoint {
  /// Mean of the sample window.
  int32_t v;
  /// Milliseconds relative to time zero.
  int32_t t;
  /// Slope of the mean.
  int32_t s;
  /// Environment value.
  int32_t e;
  /// The motion and presence channels are only sent if the sensor has them.
  bool has_presence;
  int32_t motion;
  int32_t presence;
};

struct EventInfo {
  std::string device_id;
  std::string owner_id;
  std::string identity_id;
  std::string sw_version;
  uint32_t event_id;
  time_t event_time;
  int32_t pir_time_to_time0_ms;
  int32_t sensor_ref_brightness;
  int32_t sensor_ref_exposure_ctrl;
  std::vector<AirPoint> air;
};

struct EventImage {
  const uint8_t *jpeg;
  size_t length;
  /// Time of the frame relative to time zero. Negative for frames before time zero.
  int32_t milliseconds;
  /// Sensor gain (16 is 1x) and exposure rows of the frame.
  uint32_t gain;
  uint32_t exposure;
};

/// The JSON message of an event, with the JPEG images as base64. Lives in PSRAM.
class EventPayload {
 public:
  /// Largest message that the cloud accepts.
  static constexpr size_t MAX_SIZE = 131072;

  /// Drops the largest images until the message fits. Returns false if out of memory.
  bool build(const EventInfo &info, const std::vector<EventImage> &images);

  const char *data() const { return reinterpret_cast<const char *>(this->buffer_.get()); }
  size_t size() const { return this->size_; }
  /// How many images made it into the message.
  size_t image_count() const { return this->image_count_; }

 protected:
  RAMUniquePtr<uint8_t[]> buffer_;
  size_t size_{0};
  size_t image_count_{0};
};

}  // namespace esphome::zeromouse_cloud
