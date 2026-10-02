#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace grill_cook_model {

enum class Zone : uint8_t { WEST = 0, EAST = 1 };
enum class Phase : uint8_t { IDLE, GRILLING, COOKING, STALLED, DONE };

inline const char *phase_to_str(Phase phase) {
  switch (phase) {
    case Phase::IDLE:
      return "idle";
    case Phase::GRILLING:
      return "grilling";
    case Phase::COOKING:
      return "cooking";
    case Phase::STALLED:
      return "stalled";
    case Phase::DONE:
      return "done";
    default:
      return "idle";
  }
}

struct CookInputs {
  uint32_t t_s{0};
  bool zone_on[2]{false, false};
  float zone_target[2]{NAN, NAN};
  float zone_temp[2]{NAN, NAN};
  float meat_temp{NAN};  // NAN when the probe is absent or faulted
  float meat_target{NAN};
};

// cooking_, done_, armed_ and stalled_ are the latched values. Everything else
// is recomputed from the recent inputs and only turns samples into a slope, a
// sparkline and a duration.
class CookModel {
 public:
  static constexpr size_t HISTORY_LEN = 60;
  static constexpr uint32_t HISTORY_STEP_S = 30;

  CookModel() {
    this->reset_cook_state_();
    for (auto &history : this->zone_history_) {
      history.fill(NAN);
    }
    this->meat_history_.fill(NAN);
  }

  void start() {
    if (this->cooking_)
      return;
    this->cooking_ = true;
    this->reset_cook_state_();
  }

  void end() {
    if (!this->cooking_)
      return;
    this->cooking_ = false;
    this->reset_cook_state_();
  }

  void update(const CookInputs &in) {
    this->latest_ = in;

    const bool any_zone_on = in.zone_on[0] || in.zone_on[1];
    if (!this->cooking_ && any_zone_on && !this->prev_any_zone_on_) {
      this->start();
    }
    this->prev_any_zone_on_ = any_zone_on;

    this->push_history_sample_(in);

    if (!this->cooking_)
      return;

    const bool probe_present = std::isfinite(in.meat_temp);
    if (probe_present) {
      this->push_rate_sample_(in.t_s, in.meat_temp);
    } else {
      this->rate_head_ = 0;
      this->rate_count_ = 0;
    }
    this->compute_rate_();
    this->update_done_(in, probe_present);
    this->update_stalled_(any_zone_on, probe_present);

    const bool meat_cold_or_absent = !probe_present || in.meat_temp < MEAT_COLD_C;
    if (any_zone_on || !meat_cold_or_absent) {
      this->off_since_valid_ = false;
    } else if (!this->off_since_valid_) {
      this->off_since_valid_ = true;
      this->off_since_s_ = in.t_s;
    } else if ((in.t_s - this->off_since_s_) >= AUTO_END_S) {
      this->end();
    }
  }

  bool is_cooking() const { return this->cooking_; }

  Phase phase() const {
    if (!this->cooking_)
      return Phase::IDLE;
    if (!this->meat_probe_present())
      return Phase::GRILLING;
    if (this->done_)
      return Phase::DONE;
    if (this->stalled_)
      return Phase::STALLED;
    return Phase::COOKING;
  }

  const char *phase_str() const { return phase_to_str(this->phase()); }

  float meat_rate() const { return this->rate_c_per_min_; }

  int remaining_minutes() const {
    if (!this->cooking_ || !this->meat_probe_present())
      return -1;
    if (!std::isfinite(this->latest_.meat_target))
      return -1;
    if (std::isnan(this->rate_c_per_min_) || this->rate_c_per_min_ < RATE_ETA_MIN_C_PER_MIN)
      return -1;
    const float delta = this->latest_.meat_target - this->latest_.meat_temp;
    if (delta <= 0.0f)
      return 0;
    return static_cast<int>(std::lround(delta / this->rate_c_per_min_));
  }

  bool zone_at_setpoint(Zone zone) const {
    const size_t z = static_cast<size_t>(zone);
    if (!this->latest_.zone_on[z])
      return false;
    return std::fabs(this->latest_.zone_temp[z] - this->latest_.zone_target[z]) <= ZONE_SETPOINT_TOLERANCE_C;
  }

  bool meat_probe_present() const { return std::isfinite(this->latest_.meat_temp); }

  const std::array<float, HISTORY_LEN> &zone_history(Zone zone) const {
    return this->zone_history_[static_cast<size_t>(zone)];
  }
  const std::array<float, HISTORY_LEN> &meat_history() const { return this->meat_history_; }

 private:
  static constexpr float ZONE_SETPOINT_TOLERANCE_C = 5.0f;
  static constexpr float STALL_TEMP_C = 60.0f;
  static constexpr float STALL_ENTER_RATE_C_PER_MIN = 0.1f;
  static constexpr float STALL_EXIT_RATE_C_PER_MIN = 0.2f;
  static constexpr float MEAT_COLD_C = 30.0f;
  static constexpr uint32_t AUTO_END_S = 5 * 60;
  static constexpr uint32_t RATE_WINDOW_S = 15 * 60;
  static constexpr uint32_t MIN_RATE_SPAN_S = 5 * 60;
  static constexpr float RATE_ETA_MIN_C_PER_MIN = 0.05f;
  static constexpr size_t RATE_CAPACITY = 256;

  struct RateSample {
    uint32_t t_s;
    float temp;
  };

  void reset_cook_state_() {
    this->done_ = false;
    this->done_target_ = NAN;
    this->armed_ = false;
    this->stalled_ = false;
    this->rate_head_ = 0;
    this->rate_count_ = 0;
    this->rate_c_per_min_ = NAN;
    this->off_since_valid_ = false;
  }

  // The probe can read hot before it is inserted, so DONE only arms once the
  // meat has been seen below the target.
  void update_done_(const CookInputs &in, bool probe_present) {
    if (this->done_ && in.meat_target > this->done_target_) {
      this->done_ = false;
    }
    if (!probe_present)
      return;
    if (in.meat_temp < in.meat_target) {
      this->armed_ = true;
    } else if (this->armed_ && !this->done_ && in.meat_temp >= in.meat_target) {
      this->done_ = true;
      this->done_target_ = in.meat_target;
    }
  }

  void update_stalled_(bool any_zone_on, bool probe_present) {
    const bool eligible = probe_present && !this->done_ && any_zone_on && this->latest_.meat_temp >= STALL_TEMP_C &&
                          !std::isnan(this->rate_c_per_min_);
    if (!eligible) {
      this->stalled_ = false;
    } else if (this->rate_c_per_min_ < STALL_ENTER_RATE_C_PER_MIN) {
      this->stalled_ = true;
    } else if (this->rate_c_per_min_ >= STALL_EXIT_RATE_C_PER_MIN) {
      this->stalled_ = false;
    }
  }

  void push_rate_sample_(uint32_t t_s, float temp) {
    size_t idx;
    if (this->rate_count_ < RATE_CAPACITY) {
      idx = (this->rate_head_ + this->rate_count_) % RATE_CAPACITY;
      this->rate_count_++;
    } else {
      idx = this->rate_head_;
      this->rate_head_ = (this->rate_head_ + 1) % RATE_CAPACITY;
    }
    this->rate_samples_[idx] = {t_s, temp};

    while (this->rate_count_ > 0 && (t_s - this->rate_samples_[this->rate_head_].t_s) > RATE_WINDOW_S) {
      this->rate_head_ = (this->rate_head_ + 1) % RATE_CAPACITY;
      this->rate_count_--;
    }
  }

  void compute_rate_() {
    if (this->rate_count_ < 2) {
      this->rate_c_per_min_ = NAN;
      return;
    }

    const uint32_t t0 = this->rate_samples_[this->rate_head_].t_s;
    const uint32_t newest_t = this->rate_samples_[(this->rate_head_ + this->rate_count_ - 1) % RATE_CAPACITY].t_s;
    if ((newest_t - t0) < MIN_RATE_SPAN_S) {
      this->rate_c_per_min_ = NAN;
      return;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (size_t i = 0; i < this->rate_count_; i++) {
      const auto &s = this->rate_samples_[(this->rate_head_ + i) % RATE_CAPACITY];
      const double x = static_cast<double>(s.t_s - t0);
      const double y = s.temp;
      sum_x += x;
      sum_y += y;
      sum_xx += x * x;
      sum_xy += x * y;
    }

    const double n = static_cast<double>(this->rate_count_);
    const double denom = n * sum_xx - sum_x * sum_x;
    if (std::fabs(denom) < 1e-6) {
      this->rate_c_per_min_ = NAN;
      return;
    }

    const double slope_per_s = (n * sum_xy - sum_x * sum_y) / denom;
    this->rate_c_per_min_ = static_cast<float>(slope_per_s * 60.0);
  }

  void push_history_sample_(const CookInputs &in) {
    if (this->history_started_ && (in.t_s - this->last_history_t_s_) < HISTORY_STEP_S)
      return;

    for (size_t i = 0; i + 1 < HISTORY_LEN; i++) {
      this->zone_history_[0][i] = this->zone_history_[0][i + 1];
      this->zone_history_[1][i] = this->zone_history_[1][i + 1];
      this->meat_history_[i] = this->meat_history_[i + 1];
    }
    this->zone_history_[0][HISTORY_LEN - 1] = in.zone_temp[0];
    this->zone_history_[1][HISTORY_LEN - 1] = in.zone_temp[1];
    this->meat_history_[HISTORY_LEN - 1] = in.meat_temp;
    this->last_history_t_s_ = in.t_s;
    this->history_started_ = true;
  }

  bool cooking_{false};
  bool done_{false};
  float done_target_{NAN};
  bool armed_{false};
  bool stalled_{false};

  CookInputs latest_{};
  bool prev_any_zone_on_{false};

  std::array<RateSample, RATE_CAPACITY> rate_samples_{};
  size_t rate_head_{0};
  size_t rate_count_{0};
  float rate_c_per_min_{NAN};

  std::array<float, HISTORY_LEN> zone_history_[2]{};
  std::array<float, HISTORY_LEN> meat_history_{};
  uint32_t last_history_t_s_{0};
  bool history_started_{false};

  bool off_since_valid_{false};
  uint32_t off_since_s_{0};
};

}  // namespace grill_cook_model
