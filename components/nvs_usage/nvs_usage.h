#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome::nvs_usage {

/// Publishes the share of used entries in the default NVS partition.
class NVSUsageSensor : public sensor::Sensor, public PollingComponent {
 public:
  void update() override;
  void dump_config() override;
};

}  // namespace esphome::nvs_usage
