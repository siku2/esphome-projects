#pragma once

#include <cstdint>
#include <string>

#include "shadow_state.h"

namespace esphome::zeromouse_cloud {

/// The body of the shadow update that reports the whole state.
std::string build_reported_shadow(const ShadowState &state);

struct MetricsInfo {
  std::string device_id;
  std::string thing_name;
  uint32_t boot_count;
  uint32_t event_count;
};

/// The device metrics message.
std::string build_metrics(const ShadowState &state, const MetricsInfo &info);

}  // namespace esphome::zeromouse_cloud
