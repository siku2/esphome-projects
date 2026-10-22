#include "event_payload.h"

#include <mbedtls/base64.h>

#include <cstring>

#include "esphome/core/log.h"

namespace esphome::zeromouse_cloud {

static const char *const TAG = "zeromouse_cloud";

namespace {

constexpr const char *IMAGES_CLOSE = "]}";
constexpr size_t IMAGES_CLOSE_LEN = 2;
constexpr const char *IMAGE_CLOSE = "\"}";
constexpr size_t IMAGE_CLOSE_LEN = 2;

std::string format_utc(time_t time) {
  tm parts{};
  gmtime_r(&time, &parts);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%FT%TZ", &parts);
  return buffer;
}

std::string build_head(const EventInfo &info) {
  const std::string time = format_utc(info.event_time);
  std::string out = "{\"deviceID\":\"" + info.device_id + '"';
  out += ",\"eventID\":\"" + std::to_string(info.event_id) + '"';
  out += ",\"ownerID\":\"" + info.owner_id + '"';
  out += ",\"identityID\":\"" + info.identity_id + '"';
  out += ",\"eventTime\":" + std::to_string(static_cast<unsigned long long>(info.event_time));
  out += ",\"createdAt\":\"" + time + '"';
  out += ",\"updatedAt\":\"" + time + '"';
  out += ",\"pir_time_to_time0\":" + std::to_string(info.pir_time_to_time0_ms);
  out += ", \"swVersion\":\"" + info.sw_version + '"';
  out += ",\"air\": []";
  out += ",\"sensor_ref_brightness\":" + std::to_string(info.sensor_ref_brightness);
  out += ",\"sensor_ref_exposureCtrl\":" + std::to_string(info.sensor_ref_exposure_ctrl);
  if (info.has_previous) {
    out += ",\"prevEventLogs\":{\"eventID\":" + std::to_string(info.previous_event_id);
    out += ",\"eventTime\":" + std::to_string(static_cast<unsigned long long>(info.previous_event_time));
    out += ",\"messages\":[],\"air\":[]}";
  }
  out += ",\"images\":[";
  return out;
}

std::string build_image_prefix(size_t index, const EventImage &image) {
  return "{\"imageIndex\":" + std::to_string(index) + ",\"milliSeconds\":" + std::to_string(image.milliseconds) +
         ",\"gain\":0,\"exposure\":0,\"sensor_current_brightness\":0,\"imageData\":\"";
}

size_t base64_length(size_t length) { return (length + 2) / 3 * 4; }

}  // namespace

bool EventPayload::build(const EventInfo &info, const std::vector<EventImage> &images) {
  const std::string head = build_head(info);
  std::vector<std::string> prefixes;
  std::vector<size_t> entry_sizes;
  std::vector<bool> included(images.size(), true);
  for (size_t i = 0; i < images.size(); i++) {
    prefixes.push_back(build_image_prefix(i, images[i]));
    entry_sizes.push_back(prefixes[i].size() + base64_length(images[i].length) + IMAGE_CLOSE_LEN);
  }

  size_t count = images.size();
  auto total_size = [&]() {
    size_t total = head.size() + IMAGES_CLOSE_LEN;
    for (size_t i = 0; i < images.size(); i++) {
      if (included[i])
        total += entry_sizes[i];
    }
    return count > 1 ? total + count - 1 : total;
  };
  while (count > 0 && total_size() > MAX_SIZE) {
    size_t largest = 0;
    for (size_t i = 0; i < images.size(); i++) {
      if (included[i] && (!included[largest] || entry_sizes[i] > entry_sizes[largest]))
        largest = i;
    }
    ESP_LOGW(TAG, "Message too large, dropping image %u (%u bytes)", static_cast<unsigned>(largest),
             static_cast<unsigned>(entry_sizes[largest]));
    included[largest] = false;
    count--;
  }

  const size_t total = total_size();
  RAMAllocator<uint8_t> allocator;
  this->buffer_ = allocator.make_unique_array_for_overwrite(total + 1);
  if (this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "Out of memory for a %u byte message", static_cast<unsigned>(total));
    return false;
  }

  uint8_t *const begin = this->buffer_.get();
  uint8_t *const end = begin + total + 1;
  uint8_t *pos = begin;
  memcpy(pos, head.data(), head.size());
  pos += head.size();
  bool first = true;
  for (size_t i = 0; i < images.size(); i++) {
    if (!included[i])
      continue;
    if (!first)
      *pos++ = ',';
    first = false;
    memcpy(pos, prefixes[i].data(), prefixes[i].size());
    pos += prefixes[i].size();
    size_t written = 0;
    if (mbedtls_base64_encode(pos, end - pos, &written, images[i].jpeg, images[i].length) != 0) {
      ESP_LOGE(TAG, "Base64 encoding failed");
      this->buffer_ = nullptr;
      return false;
    }
    pos += written;
    memcpy(pos, IMAGE_CLOSE, IMAGE_CLOSE_LEN);
    pos += IMAGE_CLOSE_LEN;
  }
  memcpy(pos, IMAGES_CLOSE, IMAGES_CLOSE_LEN);
  pos += IMAGES_CLOSE_LEN;
  *pos = '\0';

  this->size_ = total;
  this->image_count_ = count;
  return true;
}

}  // namespace esphome::zeromouse_cloud
