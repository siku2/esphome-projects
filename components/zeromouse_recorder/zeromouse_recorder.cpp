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
// A pending secondary copy counts as this many times the primary size.
static constexpr size_t SECONDARY_SIZE_ESTIMATE = 3;

void ZeroMouseRecorder::setup() {
  this->counter_pref_ = global_preferences->make_preference<uint32_t>(fnv1_hash("zeromouse_recorder.counter"));
  if (!this->counter_pref_.load(&this->counter_)) {
    this->counter_ = 0;
  }

  this->update_quality_();
  this->s3_->add_on_config_change_callback([this]() { this->update_quality_(); });
  if (this->enabled_ != nullptr) {
    this->enabled_->add_on_state_callback([this](bool) { this->update_quality_(); });
  }
  this->camera_->add_on_event_secondary_ready([this]() { this->on_secondary_ready_(); });
  this->controller_->add_on_event_captured(
      [this](const zeromouse::CapturedEvent &event) { this->on_captured_(event); });
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
                "  Prefix: %s\n"
                "  Secondary JPEG quality: %u\n"
                "  Max pending events: %u\n"
                "  Max pending bytes: %u",
                this->board_.c_str(), this->prefix_.c_str(), this->secondary_quality_, this->max_pending_,
                static_cast<unsigned>(this->max_pending_bytes_));
}

void ZeroMouseRecorder::update_quality_() {
  this->camera_->set_event_secondary_quality(this->is_enabled_() ? this->secondary_quality_ : 0);
}

void ZeroMouseRecorder::warn_if_not_configured_() {
  if (this->warned_not_configured_ || this->s3_->is_configured())
    return;
  ESP_LOGW(TAG, "S3 is not configured, not recording");
  this->warned_not_configured_ = true;
}

void ZeroMouseRecorder::on_captured_(const zeromouse::CapturedEvent &event) {
  if (!this->is_enabled_()) {
    this->warn_if_not_configured_();
    return;
  }
  auto record = std::make_shared<Record>();
  record->seq = this->next_seq_++;
  record->images = event.frames;
  for (const auto &image : event.frames) {
    const timeval time = image->get_timestamp();
    const int64_t us = static_cast<int64_t>(time.tv_sec) * 1000000 + time.tv_usec - event.time_zero_us;
    record->frames.push_back({static_cast<int32_t>(us / 1000), static_cast<uint32_t>(image->get_data_length()), 0});
  }
  record->air.reserve(event.samples.size());
  for (const auto &sample : event.samples) {
    record->air.push_back({sample.mean, event.sample_time_ms(sample), sample.slope, sample.environment,
                           sample.has_presence, sample.motion, sample.presence});
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
    ESP_LOGW(TAG, "Too many pending events, dropping the oldest");
    this->drop_record_(this->records_.front());
  }
  this->enforce_budget_(this->record_bytes_(*record));
  this->records_.push_back(record);
  this->last_ = record;
}

void ZeroMouseRecorder::on_secondary_ready_() {
  this->enforce_budget_(0);
  this->pump_();
}

bool ZeroMouseRecorder::is_secondary_pending_(const Record &record) {
  return std::any_of(record.images.begin(), record.images.end(),
                     [](const auto &image) { return image->is_secondary_pending(); });
}

size_t ZeroMouseRecorder::record_bytes_(const Record &record) const {
  size_t bytes = 0;
  for (const auto &image : record.images) {
    bytes += image->get_data_length();
    bytes += image->is_secondary_pending() ? image->get_data_length() * SECONDARY_SIZE_ESTIMATE
                                           : image->get_secondary_length();
  }
  return bytes;
}

size_t ZeroMouseRecorder::drop_hq_(Record &record) {
  const size_t before = this->record_bytes_(record);
  for (const auto &image : record.images) {
    image->discard_secondary();
  }
  return before - this->record_bytes_(record);
}

void ZeroMouseRecorder::enforce_budget_(size_t reserve) {
  size_t total = reserve;
  for (const auto &record : this->records_) {
    total += this->record_bytes_(*record);
  }
  if (total <= this->max_pending_bytes_) {
    return;
  }
  ESP_LOGW(TAG, "Pending events hold %u bytes, over the limit of %u", static_cast<unsigned>(total),
           static_cast<unsigned>(this->max_pending_bytes_));
  for (const auto &record : this->records_) {
    if (total <= this->max_pending_bytes_) {
      return;
    }
    if (record != this->in_flight_) {
      total -= this->drop_hq_(*record);
    }
  }
  while (total > this->max_pending_bytes_ && (reserve != 0 ? !this->records_.empty() : this->records_.size() > 1)) {
    const RecordPtr oldest = this->records_.front();
    total -= this->record_bytes_(*oldest);
    ESP_LOGW(TAG, "Dropping the oldest event to stay within the limit");
    this->drop_record_(oldest);
  }
}

void ZeroMouseRecorder::update_hq_(Record &record) const {
  record.quality_hq = 0;
  for (size_t i = 0; i < record.images.size(); i++) {
    const size_t bytes_hq = record.images[i]->get_secondary_length();
    record.frames[i].bytes_hq = static_cast<uint32_t>(bytes_hq);
    if (bytes_hq != 0) {
      record.quality_hq = this->secondary_quality_;
    }
  }
}

void ZeroMouseRecorder::drop_record_(const RecordPtr &record) {
  record->dropped = true;
  this->cancel_timeout(record->seq);
  record->images.clear();
  record->steps.clear();
  record->body.reset();
  this->remove_(record);
  if (this->last_ == record) {
    this->last_.reset();
  }
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
  record.folder = this->prefix_ + "/" + local.strftime("%Y-%m-%d") + "/" + local.strftime("%Y%m%d-%H%M%S") + "_" + id;
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
    if (record->phase == Phase::QUEUED && !is_secondary_pending_(*record)) {
      this->start_attempt_(record);
      return;
    }
  }
}

void ZeroMouseRecorder::start_attempt_(const RecordPtr &record) {
  this->update_hq_(*record);
  record->body = this->build_json_(*record);
  if (!record->body) {
    ESP_LOGE(TAG, "Out of memory building event.json");
    this->in_flight_ = record;
    this->finish_attempt_(record, false);
    return;
  }
  record->steps.clear();
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
    case Step::Kind::FRAME: {
      const auto &image = record->images[step.index];
      this->s3_->put(path, image->get_data_buffer(), image->get_data_length(), image, "image/jpeg", std::move(done));
      break;
    }
    case Step::Kind::FRAME_HQ: {
      const auto &image = record->images[step.index];
      if (image->get_secondary_data() == nullptr) {
        this->run_step_(record, index + 1);
        break;
      }
      this->s3_->put(path, image->get_secondary_data(), image->get_secondary_length(), image, "image/jpeg",
                     std::move(done));
      break;
    }
    case Step::Kind::JSON:
      this->s3_->put(path, record->body->data.get(), record->body->size, record->body, "application/json",
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
    this->drop_record_(record);
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
  this->s3_->put(path, body->data.get(), body->size, body, "application/json", [path](bool ok, int status) {
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
    if (point.has_presence) {
      item["m"] = point.m;
      item["p"] = point.p;
    }
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
