#ifdef USE_ESP32

#include "zeromouse_recorder.h"

#include <algorithm>

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include "esphome/core/version.h"

namespace esphome::zeromouse_recorder {

static const char *const TAG = "zeromouse_recorder";

static constexpr uint32_t CLASSIFICATION_WAIT_MS = 10000;
static constexpr uint32_t FIRST_RETRY_MS = 30000;
static constexpr uint8_t MAX_RETRIES = 3;
static constexpr const char *INBOX = "inbox";

void ZeroMouseRecorder::setup() {
  this->counter_pref_ = global_preferences->make_preference<uint32_t>(fnv1_hash("zeromouse_recorder.counter"));
  if (!this->counter_pref_.load(&this->counter_)) {
    this->counter_ = 0;
  }

  this->update_quality_();
  if (this->enabled_ != nullptr) {
    this->enabled_->add_on_state_callback([this](bool) { this->update_quality_(); });
  }
  this->controller_->add_on_event_captured([this](const zeromouse::CapturedEvent &event) { this->on_captured_(event); });
  this->controller_->add_on_event_published(
      [this](bool published, uint32_t event_id) { this->on_published_(published, event_id); });
  this->controller_->add_on_classification([this](const std::string &class_name, int class_index, bool block) {
    this->on_classification_(class_name, class_index, block);
  });
  this->disable_loop();
}

void ZeroMouseRecorder::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ZeroMOUSE Recorder:\n"
                "  Board: %s\n"
                "  Secondary JPEG quality: %u\n"
                "  Max pending events: %u",
                this->board_.c_str(), this->secondary_quality_, this->max_pending_);
}

void ZeroMouseRecorder::update_quality_() {
  this->camera_->set_event_secondary_quality(this->is_enabled_() ? this->secondary_quality_ : 0);
}

void ZeroMouseRecorder::on_captured_(const zeromouse::CapturedEvent &event) {
  if (!this->is_enabled_()) {
    return;
  }
  auto record = std::make_shared<Record>();
  record->seq = this->next_seq_++;
  record->images = event.frames;
  for (const auto &image : event.frames) {
    const timeval time = image->get_timestamp();
    const int64_t us = static_cast<int64_t>(time.tv_sec) * 1000000 + time.tv_usec - event.time_zero_us;
    const size_t bytes_hq = image->get_secondary_length();
    record->frames.push_back({static_cast<int32_t>(us / 1000), static_cast<uint32_t>(image->get_data_length()),
                              static_cast<uint32_t>(image->get_secondary_data() != nullptr ? bytes_hq : 0)});
    if (image->get_secondary_data() != nullptr) {
      record->quality_hq = this->secondary_quality_;
    }
  }
  record->air.reserve(event.samples.size());
  for (const auto &sample : event.samples) {
    record->air.push_back({sample.mean, event.sample_time_ms(sample), sample.slope, sample.environment});
  }
  record->settings = event.settings;
  record->scan = event.scan;
  record->scan_result = event.result;
  record->ambient = event.ambient;
  record->free_value_raw = event.free_value_raw;
  record->time_zero_us = event.time_zero_us;
  record->pir_to_t0_ms = event.pir_to_t0_ms;
  record->epoch = event.time_zero_epoch;

  while (this->records_.size() >= this->max_pending_) {
    const RecordPtr oldest = this->records_.front();
    ESP_LOGW(TAG, "Too many pending events, dropping the oldest");
    oldest->dropped = true;
    this->cancel_timeout(oldest->seq);
    this->records_.pop_front();
  }
  this->records_.push_back(record);
  this->last_ = record;
}

void ZeroMouseRecorder::name_folder_(Record &record) {
  ESPTime local = ESPTime::from_epoch_local(record.epoch);
  std::string id;
  if (record.published) {
    id = std::to_string(record.cloud_event_id);
  } else {
    this->counter_++;
    this->counter_pref_.save(&this->counter_);
    id = "local" + std::to_string(this->counter_);
  }
  record.folder =
      std::string(INBOX) + "/" + local.strftime("%Y-%m-%d") + "/" + local.strftime("%Y%m%d-%H%M%S") + "_" + id;
}

void ZeroMouseRecorder::on_published_(bool published, uint32_t event_id) {
  const RecordPtr record = this->last_;
  if (!record || record->dropped || record->phase != Phase::AWAITING_PUBLISH) {
    return;
  }
  record->published = published;
  record->cloud_event_id = event_id;
  this->name_folder_(*record);
  if (!published) {
    this->queue_(record);
    return;
  }
  record->phase = Phase::AWAITING_CLASSIFICATION;
  this->set_timeout(record->seq, CLASSIFICATION_WAIT_MS, [this, record]() {
    ESP_LOGD(TAG, "No classification yet, writing %s without it", record->folder.c_str());
    this->queue_(record);
  });
}

void ZeroMouseRecorder::on_classification_(const std::string &class_name, int class_index, bool block) {
  const RecordPtr record = this->last_;
  if (!record || record->dropped || record->has_class || !record->published) {
    return;
  }
  record->has_class = true;
  record->class_name = class_name;
  record->class_index = class_index;
  record->block = block;
  switch (record->phase) {
    case Phase::AWAITING_CLASSIFICATION:
      this->cancel_timeout(record->seq);
      this->queue_(record);
      break;
    case Phase::UPLOADING:
      record->resend = true;
      break;
    case Phase::DONE:
      this->resend_json_(record);
      break;
    default:
      break;
  }
}

void ZeroMouseRecorder::queue_(const RecordPtr &record) {
  if (record->dropped) {
    return;
  }
  record->phase = Phase::QUEUED;
  this->pump_();
}

void ZeroMouseRecorder::pump_() {
  if (this->in_flight_) {
    return;
  }
  for (const auto &record : this->records_) {
    if (record->phase == Phase::QUEUED) {
      this->start_attempt_(record);
      return;
    }
  }
}

void ZeroMouseRecorder::start_attempt_(const RecordPtr &record) {
  record->body = this->build_json_(*record);
  if (!record->body) {
    ESP_LOGE(TAG, "Out of memory building event.json");
    this->in_flight_ = record;
    this->finish_attempt_(record, false);
    return;
  }
  record->steps.clear();
  for (uint8_t i = 0; i < 3; i++) {
    record->steps.push_back({Step::Kind::MKCOL, i});
  }
  for (size_t i = 0; i < record->images.size(); i++) {
    record->steps.push_back({Step::Kind::FRAME, static_cast<uint8_t>(i)});
    if (record->frames[i].bytes_hq != 0) {
      record->steps.push_back({Step::Kind::FRAME_HQ, static_cast<uint8_t>(i)});
    }
  }
  record->steps.push_back({Step::Kind::JSON, 0});
  record->phase = Phase::UPLOADING;
  this->in_flight_ = record;
  ESP_LOGD(TAG, "Uploading %s (attempt %u)", record->folder.c_str(), record->attempts + 1);
  this->run_step_(record, 0);
}

std::string ZeroMouseRecorder::step_path_(const Record &record, const Step &step) const {
  switch (step.kind) {
    case Step::Kind::MKCOL: {
      size_t end = record.folder.size();
      if (step.index == 0) {
        end = record.folder.find('/');
      } else if (step.index == 1) {
        end = record.folder.find('/', record.folder.find('/') + 1);
      }
      return record.folder.substr(0, end);
    }
    case Step::Kind::FRAME:
      return record.folder + "/frame_" + std::to_string(step.index) + ".jpg";
    case Step::Kind::FRAME_HQ:
      return record.folder + "/frame_" + std::to_string(step.index) + "_hq.jpg";
    case Step::Kind::JSON:
      break;
  }
  return record.folder + "/event.json";
}

void ZeroMouseRecorder::run_step_(const RecordPtr &record, size_t index) {
  if (record->dropped) {
    this->finish_attempt_(record, false);
    return;
  }
  if (index >= record->steps.size()) {
    this->finish_attempt_(record, true);
    return;
  }
  const Step step = record->steps[index];
  const std::string path = this->step_path_(*record, step);
  auto done = [this, record, index, path](bool ok, int status) {
    if (!ok) {
      ESP_LOGW(TAG, "Uploading %s failed with status %d", path.c_str(), status);
      this->finish_attempt_(record, false);
      return;
    }
    this->run_step_(record, index + 1);
  };
  switch (step.kind) {
    case Step::Kind::MKCOL:
      this->webdav_->mkcol(path, std::move(done));
      break;
    case Step::Kind::FRAME: {
      const auto &image = record->images[step.index];
      this->webdav_->put(path, image->get_data_buffer(), image->get_data_length(), image, "image/jpeg",
                         std::move(done));
      break;
    }
    case Step::Kind::FRAME_HQ: {
      const auto &image = record->images[step.index];
      this->webdav_->put(path, image->get_secondary_data(), image->get_secondary_length(), image, "image/jpeg",
                         std::move(done));
      break;
    }
    case Step::Kind::JSON:
      this->webdav_->put(path, record->body->data.get(), record->body->size, record->body, "application/json",
                         std::move(done));
      break;
  }
}

void ZeroMouseRecorder::finish_attempt_(const RecordPtr &record, bool ok) {
  if (this->in_flight_ == record) {
    this->in_flight_.reset();
  }
  if (record->dropped) {
    this->pump_();
    return;
  }
  if (ok) {
    ESP_LOGI(TAG, "Recorded %s", record->folder.c_str());
    record->phase = Phase::DONE;
    record->images.clear();
    record->steps.clear();
    record->body.reset();
    this->remove_(record);
    if (!record->recorded) {
      record->recorded = true;
      this->recorded_trigger_.trigger(record->folder, record->published ? record->cloud_event_id : 0,
                                      record->has_class ? record->class_name : std::string());
    }
    if (record->resend) {
      record->resend = false;
      this->resend_json_(record);
    }
  } else if (++record->attempts > MAX_RETRIES) {
    ESP_LOGW(TAG, "Giving up on %s", record->folder.c_str());
    record->dropped = true;
    this->remove_(record);
  } else {
    const uint32_t delay_ms = FIRST_RETRY_MS << (record->attempts - 1);
    ESP_LOGW(TAG, "Retrying %s in %u s", record->folder.c_str(), static_cast<unsigned>(delay_ms / 1000));
    record->phase = Phase::BACKOFF;
    this->set_timeout(record->seq, delay_ms, [this, record]() { this->queue_(record); });
  }
  this->pump_();
}

void ZeroMouseRecorder::resend_json_(const RecordPtr &record) {
  auto body = this->build_json_(*record);
  if (!body) {
    return;
  }
  const std::string path = record->folder + "/event.json";
  this->webdav_->put(path, body->data.get(), body->size, body, "application/json", [path](bool ok, int status) {
    if (!ok) {
      ESP_LOGW(TAG, "Updating %s failed with status %d", path.c_str(), status);
    }
  });
}

void ZeroMouseRecorder::remove_(const RecordPtr &record) {
  this->records_.erase(std::remove(this->records_.begin(), this->records_.end(), record), this->records_.end());
}

std::shared_ptr<ZeroMouseRecorder::Body> ZeroMouseRecorder::build_json_(const Record &record) const {
#ifdef USE_PSRAM
  json::SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
#else
  JsonDocument doc;
#endif
  doc["schema_version"] = 1;
  doc["device"] = App.get_name().c_str();
  doc["board"] = this->board_;
  JsonObject firmware = doc["firmware"].to<JsonObject>();
  firmware["esphome"] = ESPHOME_VERSION;
#ifdef ESPHOME_PROJECT_VERSION
  firmware["project"] = ESPHOME_PROJECT_VERSION;
#endif
  ESPTime utc = ESPTime::from_epoch_utc(record.epoch);
  doc["time"] = utc.strftime("%Y-%m-%dT%H:%M:%SZ");
  if (record.published) {
    doc["cloud_event_id"] = record.cloud_event_id;
  } else {
    doc["cloud_event_id"] = nullptr;
  }
  doc["published"] = record.published;
  if (record.has_class) {
    JsonObject classification = doc["classification"].to<JsonObject>();
    classification["class"] = record.class_name;
    classification["index"] = record.class_index;
    classification["block"] = record.block;
  } else {
    doc["classification"] = nullptr;
  }
  doc["time_zero_us"] = record.time_zero_us;
  doc["pir_to_t0_ms"] = record.pir_to_t0_ms;
  doc["jpeg_quality"] = this->camera_->get_jpeg_quality();
  doc["jpeg_quality_hq"] = record.quality_hq;

  JsonArray frames = doc["frames"].to<JsonArray>();
  for (size_t i = 0; i < record.frames.size(); i++) {
    const Frame &frame = record.frames[i];
    JsonObject item = frames.add<JsonObject>();
    item["index"] = i;
    item["ms"] = frame.ms;
    item["file"] = "frame_" + std::to_string(i) + ".jpg";
    if (frame.bytes_hq != 0) {
      item["file_hq"] = "frame_" + std::to_string(i) + "_hq.jpg";
    } else {
      item["file_hq"] = nullptr;
    }
    item["bytes"] = frame.bytes;
    item["bytes_hq"] = frame.bytes_hq;
  }

  JsonArray air = doc["air"].to<JsonArray>();
  for (const Air &point : record.air) {
    JsonObject item = air.add<JsonObject>();
    item["v"] = point.v;
    item["t"] = point.t;
    item["s"] = point.s;
    item["e"] = point.e;
  }

  JsonObject settings = doc["settings"].to<JsonObject>();
  settings["ir_slope_thresh_value"] = record.settings.slope_thresh;
  settings["ir_median_count"] = record.settings.mean_window;
  settings["ir_range_ignore_event"] = record.settings.range_ignore_event;
  settings["ir_ambient"] = record.ambient;
  settings["cam_shots_before_t0"] = record.settings.shots_before_t0;
  settings["ir_free_value"] = record.settings.ir_free_value;
  settings["ir_free_value_raw"] = record.free_value_raw;

  JsonObject scan = doc["scan"].to<JsonObject>();
  scan["sample_count"] = record.scan.sample_count;
  scan["min_mean"] = record.scan.min_mean;
  scan["max_mean"] = record.scan.max_mean;
  scan["result"] = record.scan_result;

  if (doc.overflowed()) {
    return nullptr;
  }
  const size_t size = measureJson(doc);
  auto body = std::make_shared<Body>();
  RAMAllocator<uint8_t> buffer_allocator;
  body->data = buffer_allocator.make_unique_array_for_overwrite(size + 1);
  if (!body->data) {
    return nullptr;
  }
  body->size = serializeJson(doc, reinterpret_cast<char *>(body->data.get()), size + 1);
  return body;
}

}  // namespace esphome::zeromouse_recorder

#endif
