#include "meter_reader.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

static const char *const TAG = "meter_reader";

// "METR" as a stable preference key for the single MeterReader per device.
static const uint32_t PREF_KEY = 0x4D455452U;
static const uint32_t SAVE_INTERVAL_MS = 60000;
static constexpr float CYCLE_EMA_ALPHA = 0.3f;
static constexpr float DIAL_PERIOD = 10.0f;

static const char *const STATUS_STRINGS[] = {"anchoring", "ok", "stale", "divergent"};

namespace esphome::meter_reader {

static float circdist(float a, float b) {
  float d = std::fmod(std::fabs(a - b), DIAL_PERIOD);
  return d > DIAL_PERIOD / 2.0f ? DIAL_PERIOD - d : d;
}

// A wheel at level k knows the integer digit (u / 10^k) mod 10. Mechanical
// wheels do not track the linear fraction of the finer digits: they sit at
// N.0 and creep only near a roll, so comparisons use the digit only and
// leave the tolerance for roll ambiguity and OCR noise.
static float digit_of(uint64_t u, uint8_t level) {
  uint64_t p = 1;
  for (uint8_t i = 0; i < level && i < 18; ++i)
    p *= 10U;
  return static_cast<float>((u / p) % 10ULL);
}

void MeterReader::setup() {
  if (this->snapshotter_ == nullptr || this->wheels_.empty()) {
    this->mark_failed();
    return;
  }
  // The snapshotter notifies one wheel sensor per loop pass, with
  // arbitrarily long stalls in between. Processing on the cycle-end
  // callback is the only way to see one complete observation set.
  this->snapshotter_->add_cycle_end_callback([this]() {
    this->process_cycle_(millis());
    for (auto &w : this->wheels_)
      w.present = false;
  });

  this->rtc_ = global_preferences->make_preference<uint64_t>(PREF_KEY);
  uint64_t stored = 0;
  this->has_saved_ = this->rtc_.load(&stored);
  if (this->has_saved_) {
    this->u_ = stored;
    ESP_LOGI(TAG, "Restored reading: %.1f L", this->reading_liters_());
  } else {
    ESP_LOGI(TAG, "No stored reading: set rebase to start");
  }
  const uint32_t now = millis();
  this->last_commit_ms_ = now;
  this->last_save_ms_ = now;
  this->publish_reading_();
  this->publish_rejected_();
  this->update_status_();
}

void MeterReader::loop() {
  const uint32_t now = millis();

  // Only the finest occupied wheels gate staleness: the reading machinery
  // cannot advance without them. Absent coarse wheels reduce cross-checking
  // but are common in practice (permanently mid-roll digits).
  bool stale = false;
  uint8_t checked = 0;
  for (const auto &w : this->wheels_) {
    if (w.resolution == 0.0f)
      continue;  // unoccupied level slot
    if (checked >= 2)
      break;
    checked++;
    if (now - w.last_accepted_ms > this->stale_after_ms_)
      stale = true;
  }
  if (stale != this->stale_) {
    this->stale_ = stale;
    this->update_status_();
  }
}

void MeterReader::on_observation(uint8_t level, float result, float fit, bool accepted) {
  if (this->is_failed() || level >= this->wheels_.size())
    return;
  if (!accepted)
    return;
  Wheel &w = this->wheels_[level];
  w.result = result;
  w.fit = fit;
  w.present = true;
  w.last_accepted_ms = millis();
}

void MeterReader::process_cycle_(uint32_t now) {
  char wheels_buf[96];
  size_t pos = 0;
  for (const auto &w : this->wheels_) {
    if (w.resolution == 0.0f)
      continue;
    if (w.present)
      pos += snprintf(wheels_buf + pos, sizeof(wheels_buf) - pos, "L%u=%.2f ", w.level,
                      (double) w.result);
    else
      pos += snprintf(wheels_buf + pos, sizeof(wheels_buf) - pos, "L%u:- ", w.level);
  }
  ESP_LOGD(TAG, "Cycle wheels: %s", wheels_buf);

  if (!this->wheels_[0].present) {
    if (this->confidence_sensor_ != nullptr)
      this->confidence_sensor_->publish_state(NAN);
    return;
  }

  const float err = this->consistency_error_(this->u_);
  this->last_consistency_error_ = err;
  // Pre-anchor the reading is meaningless, so only publish once anchored.
  if (this->anchored_ && this->consistency_sensor_ != nullptr)
    this->consistency_sensor_->publish_state(err);

  float min_fit = 1.0f;
  for (const auto &w : this->wheels_) {
    if (w.present && w.fit < min_fit)
      min_fit = w.fit;
  }
  if (this->confidence_sensor_ != nullptr)
    this->confidence_sensor_->publish_state(100.0f * min_fit);

  const bool u_consistent = this->consistent_(this->u_);
  if (u_consistent) {
    this->inconsistent_since_ms_ = 0;
  } else if (this->inconsistent_since_ms_ == 0) {
    this->inconsistent_since_ms_ = now;
  }
  if (!this->divergent_ && !u_consistent && (this->anchored_ || this->has_saved_) &&
      this->inconsistent_since_ms_ != 0 && now - this->inconsistent_since_ms_ > this->stale_after_ms_) {
    this->divergent_ = true;
    ESP_LOGE(TAG, "Dials inconsistent with reading for %u s: marking divergent",
             (unsigned) ((now - this->inconsistent_since_ms_) / 1000));
    this->update_status_();
  }

  if (!this->anchored_) {
    if (this->has_saved_) {
      if (u_consistent) {
        this->anchor_(false, now);
      } else {
        uint64_t candidate = 0;
        if (this->find_consistent_(this->u_ + 1, this->reanchor_span_u_(), &candidate)) {
          ESP_LOGW(TAG, "Reanchoring: adopting %llu (+%llu quanta)", (unsigned long long) candidate,
                   (unsigned long long) (candidate - this->u_));
          this->u_ = candidate;
          this->anchor_(true, now);
        }
      }
    }
    return;
  }

  const float quantum = this->quantum_();
  const uint64_t cap =
      static_cast<uint64_t>(std::ceil(this->max_flow_ * (now - this->last_commit_ms_) / 1000.0f / quantum)) + 1;

  // The finest wheel selects the candidate closest to its observed position.
  // Coarse wheels veto out-of-tolerance candidates and break ties on equal
  // finest distance, so the dial periodicity cannot leave the reading a full
  // revolution behind.
  const Wheel &finest = this->wheels_[0];
  uint64_t u_star = this->u_;
  bool found = false;
  float best_finest = 0.0f;
  float best_coarse = 0.0f;
  for (uint64_t c = this->u_; c <= this->u_ + cap; ++c) {
    const float d0 = circdist(digit_of(c, 0), finest.result);
    if (d0 > finest.tolerance)
      continue;
    if (!this->coarse_consistent_(c))
      continue;
    const float dc = this->coarse_error_(c);
    if (!found || d0 < best_finest || (d0 == best_finest && dc < best_coarse)) {
      found = true;
      best_finest = d0;
      best_coarse = dc;
      u_star = c;
    }
  }

  if (!found) {
    this->rejected_++;
    this->publish_rejected_();
    ESP_LOGD(TAG, "Veto: no consistent value in [U, U+%llu]", (unsigned long long) cap);
  } else {
    if (this->has_pending_) {
      if (u_star >= this->pending_u_) {
        this->pending_confirmed_++;
        if (this->pending_confirmed_ >= this->corroborations_)
          this->commit_(this->pending_u_, now);
      } else {
        this->has_pending_ = false;
        this->rejected_++;
        this->publish_rejected_();
        ESP_LOGD(TAG, "Phantom pending value %llu rejected", (unsigned long long) this->pending_u_);
      }
    }
    if (!this->has_pending_ && u_star > this->u_) {
      this->pending_u_ = u_star;
      this->pending_confirmed_ = 0;
      this->pending_ms_ = now;
      this->has_pending_ = true;
    }
  }

  if (this->has_pending_ && now - this->pending_ms_ > this->pending_window_ms_)
    this->has_pending_ = false;
}

void MeterReader::commit_(uint64_t u, uint32_t now) {
  const uint32_t dt_ms = now - this->last_commit_ms_;
  const float prev_liters = this->reading_liters_();
  this->u_ = u;
  this->has_pending_ = false;
  this->last_commit_ms_ = now;
  this->publish_reading_();
  if (this->consumption_sensor_ != nullptr && dt_ms != 0) {
    const float liters = this->reading_liters_();
    this->consumption_sensor_->publish_state((liters - prev_liters) / (dt_ms / 1000.0f));
  }
  if (now - this->last_save_ms_ >= SAVE_INTERVAL_MS)
    this->save_pref_(now);
}

void MeterReader::anchor_(bool jump, uint32_t now) {
  this->anchored_ = true;
  this->divergent_ = false;
  this->inconsistent_since_ms_ = 0;
  this->has_pending_ = false;
  this->last_commit_ms_ = now;
  this->publish_reading_();
  this->update_status_();
  ESP_LOGI(TAG, "Anchored at %.1f L", this->reading_liters_());
  if (jump)
    this->save_pref_(now);
}

void MeterReader::apply_rebase(float liters) {
  if (this->wheels_.empty() || std::isnan(liters) || liters < 0.0f)
    return;
  const uint64_t u =
      static_cast<uint64_t>(std::llround(static_cast<double>(liters) / static_cast<double>(this->quantum_())));
  const uint32_t now = millis();
  this->u_ = u;
  this->has_pending_ = false;
  this->divergent_ = false;
  this->stale_ = false;
  this->inconsistent_since_ms_ = 0;
  this->anchored_ = true;
  this->last_commit_ms_ = now;
  this->publish_reading_();
  this->update_status_();
  this->save_pref_(now);
  ESP_LOGI(TAG, "Rebased to %.1f L (%llu quanta)", static_cast<double>(liters), (unsigned long long) u);
}

void MeterReader::add_wheel(uint8_t level, float resolution, float tolerance) {
  if (level >= this->wheels_.size())
    this->wheels_.resize(level + 1);
  Wheel w{};
  w.level = level;
  w.resolution = resolution;
  w.tolerance = tolerance;
  this->wheels_[level] = w;
}

float MeterReader::quantum_() const {
  if (this->wheels_.empty())
    return 0.0f;
  return this->wheels_[0].resolution / 10.0f;
}

float MeterReader::reading_liters_() const {
  return static_cast<float>(static_cast<double>(this->u_) * static_cast<double>(this->quantum_()));
}

uint64_t MeterReader::reanchor_span_u_() const {
  const float quantum = this->quantum_();
  if (quantum <= 0.0f)
    return 0;
  return static_cast<uint64_t>(std::ceil(this->reanchor_tolerance_ / quantum));
}

float MeterReader::consistency_error_(uint64_t u) const {
  float err = 0.0f;
  for (const auto &w : this->wheels_) {
    if (!w.present)
      continue;
    const float d = circdist(digit_of(u, w.level), w.result);
    if (d > err)
      err = d;
  }
  return err;
}

float MeterReader::coarse_error_(uint64_t u) const {
  float err = 0.0f;
  for (const auto &w : this->wheels_) {
    if (w.level == 0 || !w.present)
      continue;
    const float d = circdist(digit_of(u, w.level), w.result);
    if (d > err)
      err = d;
  }
  return err;
}

bool MeterReader::coarse_consistent_(uint64_t u) const {
  for (const auto &w : this->wheels_) {
    if (w.level == 0 || !w.present)
      continue;
    if (circdist(digit_of(u, w.level), w.result) > w.tolerance)
      return false;
  }
  return true;
}

bool MeterReader::consistent_(uint64_t u) const {
  for (const auto &w : this->wheels_) {
    if (!w.present)
      continue;
    if (circdist(digit_of(u, w.level), w.result) > w.tolerance)
      return false;
  }
  return true;
}

bool MeterReader::find_consistent_(uint64_t start, uint64_t span, uint64_t *out) const {
  for (uint64_t c = start; c <= start + span; ++c) {
    if (this->consistent_(c)) {
      *out = c;
      return true;
    }
  }
  return false;
}

void MeterReader::publish_reading_() {
  if (this->reading_sensor_ != nullptr)
    this->reading_sensor_->publish_state(this->reading_liters_());
}

void MeterReader::publish_rejected_() {
  if (this->rejected_sensor_ != nullptr)
    this->rejected_sensor_->publish_state(static_cast<float>(this->rejected_));
}

void MeterReader::update_status_() {
  ReaderStatus status = STATUS_ANCHORING;
  if (this->anchored_) {
    if (this->divergent_)
      status = STATUS_DIVERGENT;
    else if (this->stale_)
      status = STATUS_STALE;
    else
      status = STATUS_OK;
  }
  if (status == this->status_ && this->status_sensor_ != nullptr && this->status_sensor_->has_state())
    return;
  this->status_ = status;
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(STATUS_STRINGS[status]);
}

void MeterReader::save_pref_(uint32_t now) {
  this->last_save_ms_ = now;
  this->rtc_.save(&this->u_);
  global_preferences->sync();
}

void MeterRebaseNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->apply_rebase(value);
}

void MeterReader::dump_config() {
  ESP_LOGCONFIG(TAG, "MeterReader:");
  ESP_LOGCONFIG(TAG, "  Max flow: %.3f L/s", this->max_flow_);
  ESP_LOGCONFIG(TAG, "  Corroborations: %u", this->corroborations_);
  ESP_LOGCONFIG(TAG, "  Pending window: %.1f s", this->pending_window_ms_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Stale after: %.1f s", this->stale_after_ms_ / 1000.0f);
  ESP_LOGCONFIG(TAG, "  Reanchor tolerance: %.1f L", this->reanchor_tolerance_);
  ESP_LOGCONFIG(TAG, "  Quantum: %g L", (double) this->quantum_());
  for (const auto &w : this->wheels_) {
    if (w.resolution == 0.0f)
      continue;  // unoccupied level slot
    ESP_LOGCONFIG(TAG, "  Wheel level %u: %.1f L/rev, tolerance %.2f", (unsigned) w.level,
                  (double) w.resolution, (double) w.tolerance);
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
