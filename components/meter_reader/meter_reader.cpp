#include "meter_reader.h"

#include <cmath>

#include "esphome/core/log.h"

static const char *const TAG = "meter_reader";

// "METR" as a stable preference key for the single MeterReader per device.
static const uint32_t PREF_KEY = 0x4D455452U;

static const char *const STATUS_STRINGS[] = {"anchoring", "ok", "stale", "divergent"};

namespace esphome::meter_reader {

void MeterReader::setup() {
  if (this->camera_ == nullptr || !this->core_.has_wheels()) {
    this->mark_failed();
    return;
  }
  // The camera publishes every wheel sensor before it ends the cycle, so the
  // cycle-end callback sees one complete observation set.
  this->camera_->add_cycle_end_callback([this]() { this->on_cycle_end_(millis()); });

  this->rtc_ = global_preferences->make_preference<uint64_t>(PREF_KEY);
  uint64_t stored = 0;
  const bool has_saved = this->rtc_.load(&stored);
  this->core_.start(millis(), has_saved, stored);
  if (has_saved) {
    ESP_LOGI(TAG, "Restored reading: %.1f L", this->core_.reading_liters());
  } else {
    ESP_LOGI(TAG, "No stored reading: set rebase to start");
  }
  this->publish_reading_();
  this->publish_rejected_();
  this->publish_status_();
}

void MeterReader::loop() {
  if (this->core_.update_stale(millis()))
    this->publish_status_();
}

void MeterReader::on_observation(uint8_t level, float result, float fit, bool accepted) {
  if (this->is_failed() || !accepted)
    return;
  this->core_.observe(level, result, fit, millis());
}

void MeterReader::on_cycle_end_(uint32_t now) {
  char wheels_buf[96];
  wheels_buf[0] = '\0';
  size_t pos = 0;
  for (const auto &w : this->core_.wheels()) {
    if (w.resolution == 0.0f)
      continue;
    if (pos >= sizeof(wheels_buf))
      break;
    int n;
    if (w.present) {
      n = snprintf(wheels_buf + pos, sizeof(wheels_buf) - pos, "L%u=%.2f ", w.level, (double) w.result);
    } else {
      n = snprintf(wheels_buf + pos, sizeof(wheels_buf) - pos, "L%u:- ", w.level);
    }
    if (n < 0)
      break;
    pos += n;
  }
  ESP_LOGD(TAG, "Cycle wheels: %s", wheels_buf);

  const auto result = this->core_.end_cycle(now);
  if (!result.ran) {
    if (this->confidence_sensor_ != nullptr)
      this->confidence_sensor_->publish_state(NAN);
    return;
  }
  if (result.has_consistency && this->consistency_sensor_ != nullptr)
    this->consistency_sensor_->publish_state(result.consistency_error);
  if (result.has_fit && this->confidence_sensor_ != nullptr)
    this->confidence_sensor_->publish_state(100.0f * result.min_fit);
  this->log_events_(result);
  this->apply_(result);
}

void MeterReader::log_events_(const ::meter_reader_core::CycleResult &result) {
  using ::meter_reader_core::EventKind;
  for (size_t i = 0; i < result.event_count; ++i) {
    const auto &e = result.events[i];
    switch (e.kind) {
      case EventKind::REANCHOR:
        ESP_LOGW(TAG, "Reanchoring: adopting %llu (%+lld quanta)", (unsigned long long) e.a, (long long) e.b);
        break;
      case EventKind::DIVERGENT:
        ESP_LOGE(TAG, "Dials inconsistent with reading for %u s: marking divergent", (unsigned) e.a);
        break;
      case EventKind::WHEEL_EXCLUDED:
        ESP_LOGD(TAG, "Wheel L%d is outside tolerance for every candidate: ignored this cycle", (int) e.a);
        break;
      case EventKind::VETO:
        ESP_LOGD(TAG, "Veto: no consistent value in [U-%llu, U+%llu]", (unsigned long long) e.a,
                 (unsigned long long) e.b);
        break;
      case EventKind::PHANTOM:
        ESP_LOGD(TAG, "Phantom pending value %llu rejected", (unsigned long long) e.a);
        break;
    }
  }
}

void MeterReader::apply_(const ::meter_reader_core::CycleResult &result) {
  if (result.reading_changed)
    this->publish_reading_();
  if (result.committed && this->consumption_sensor_ != nullptr && result.commit_dt_ms != 0) {
    const double quantum = this->core_.quantum();
    const double delta = (static_cast<double>(this->core_.u()) - static_cast<double>(result.prev_u)) * quantum;
    this->consumption_sensor_->publish_state(static_cast<float>(delta / (result.commit_dt_ms / 1000.0)));
  }
  if (result.rejected_changed)
    this->publish_rejected_();
  if (result.status_changed)
    this->publish_status_();
  if (result.anchored)
    ESP_LOGI(TAG, "Anchored at %.1f L", this->core_.reading_liters());
  if (result.save) {
    const uint64_t u = this->core_.u();
    this->rtc_.save(&u);
    // Without flush the value waits for the preferences syncer
    // (flash_write_interval and shutdown).
    if (result.flush)
      global_preferences->sync();
  }
}

void MeterReader::apply_rebase(float liters) {
  const auto result = this->core_.rebase(liters, millis());
  if (!result.reading_changed)
    return;
  this->apply_(result);
  ESP_LOGI(TAG, "Rebased to %.1f L (%llu quanta)", static_cast<double>(liters), (unsigned long long) this->core_.u());
}

void MeterReader::publish_reading_() {
  if (this->reading_sensor_ != nullptr)
    this->reading_sensor_->publish_state(this->core_.reading_liters());
}

void MeterReader::publish_rejected_() {
  if (this->rejected_sensor_ != nullptr)
    this->rejected_sensor_->publish_state(static_cast<float>(this->core_.rejected()));
}

void MeterReader::publish_status_() {
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(STATUS_STRINGS[this->core_.status()]);
}

void MeterRebaseNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->apply_rebase(value);
}

void MeterReader::dump_config() {
  ESP_LOGCONFIG(TAG, "MeterReader:");
  ESP_LOGCONFIG(TAG, "  Max flow: %.3f L/s", this->core_.params.max_flow);
  ESP_LOGCONFIG(TAG, "  Corroborations: %u", this->core_.params.corroborations);
  ESP_LOGCONFIG(TAG, "  Pending window: %.1f s", this->core_.params.pending_window_ms / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Stale after: %.1f s", this->core_.params.stale_after_ms / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Reanchor tolerance: %.1f L", this->core_.params.reanchor_tolerance);
  ESP_LOGCONFIG(TAG, "  Back tolerance: %.1f L", this->core_.params.back_tolerance);
  ESP_LOGCONFIG(TAG, "  Quantum: %g L", (double) this->core_.quantum());
  for (const auto &w : this->core_.wheels()) {
    if (w.resolution == 0.0f)
      continue;  // unoccupied level slot
    ESP_LOGCONFIG(TAG, "  Wheel level %u: %.1f L/rev, tolerance %.2f", (unsigned) w.level, (double) w.resolution,
                  (double) w.tolerance);
  }
  LOG_SENSOR("  ", "Reading", this->reading_sensor_);
  LOG_SENSOR("  ", "Consumption", this->consumption_sensor_);
  LOG_SENSOR("  ", "Confidence", this->confidence_sensor_);
  LOG_SENSOR("  ", "Consistency", this->consistency_sensor_);
  LOG_SENSOR("  ", "Rejected", this->rejected_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
  LOG_NUMBER("  ", "Rebase", this->rebase_number_);
}

}  // namespace esphome::meter_reader
