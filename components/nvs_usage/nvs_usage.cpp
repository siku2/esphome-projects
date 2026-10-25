#include "nvs_usage.h"

#include <nvs.h>

#include "esphome/core/log.h"

namespace esphome::nvs_usage {

static const char *const TAG = "nvs_usage";

void NVSUsageSensor::update() {
  nvs_stats_t stats;
  esp_err_t err = nvs_get_stats(nullptr, &stats);
  if (err != ESP_OK || stats.total_entries == 0) {
    ESP_LOGW(TAG, "Reading the NVS statistics failed: %s", esp_err_to_name(err));
    this->publish_state(NAN);
    return;
  }
  ESP_LOGD(TAG, "%u of %u entries used", static_cast<unsigned>(stats.used_entries),
           static_cast<unsigned>(stats.total_entries));
  this->publish_state(100.0f * stats.used_entries / stats.total_entries);
}

void NVSUsageSensor::dump_config() { LOG_SENSOR("", "NVS Usage", this); }

}  // namespace esphome::nvs_usage
