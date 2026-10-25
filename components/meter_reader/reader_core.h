#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace meter_reader_core {

enum ReaderStatus : uint8_t {
  STATUS_ANCHORING,
  STATUS_OK,
  STATUS_STALE,
  STATUS_DIVERGENT,
};

struct Params {
  float max_flow{1.5f};
  uint32_t corroborations{1};
  uint32_t pending_window_ms{6000};
  uint32_t stale_after_ms{60000};
  float reanchor_tolerance{100.0f};
  float back_tolerance{20.0f};
};

struct Wheel {
  uint8_t level;
  float resolution;
  float tolerance;
  bool linear;
  float result{0.0f};
  float fit{0.0f};
  bool present{false};
  uint32_t last_accepted_ms{0};
};

enum class EventKind : uint8_t {
  REANCHOR,
  DIVERGENT,
  WHEEL_EXCLUDED,
  VETO,
  PHANTOM,
};

struct Event {
  EventKind kind;
  int64_t a;
  int64_t b;
};

struct CycleResult {
  bool ran{false};
  bool has_consistency{false};
  float consistency_error{0.0f};
  bool has_fit{false};
  float min_fit{1.0f};
  bool committed{false};
  uint64_t prev_u{0};
  uint32_t commit_dt_ms{0};
  bool anchored{false};
  bool reading_changed{false};
  bool rejected_changed{false};
  bool status_changed{false};
  bool save{false};
  bool flush{false};
  std::array<Event, 16> events{};
  size_t event_count{0};

  void add_event(EventKind kind, int64_t a = 0, int64_t b = 0) {
    if (this->event_count < this->events.size())
      this->events[this->event_count++] = Event{kind, a, b};
  }
};

inline constexpr float DIAL_PERIOD = 10.0f;
inline constexpr uint64_t MAX_SEARCH_QUANTA = 100000;
inline constexpr uint32_t SAVE_INTERVAL_MS = 60000;

inline float circdist(float a, float b) {
  float d = std::fmod(std::fabs(a - b), DIAL_PERIOD);
  return d > DIAL_PERIOD / 2.0f ? DIAL_PERIOD - d : d;
}

inline uint64_t level_scale(uint8_t level) {
  uint64_t p = 1;
  for (uint8_t i = 0; i < level && i < 18; ++i)
    p *= 10U;
  return p;
}

// Wheel at level k shows (u / 10^k) mod 10. Digit drums lag the linear
// fraction (they creep only near a roll), so they compare by integer digit.
// Needle dials are geared 10:1 to the next finer dial and track the linear
// fraction exactly.
inline float digit_of(uint64_t u, uint8_t level) { return static_cast<float>((u / level_scale(level)) % 10ULL); }

inline float dial_of(uint64_t u, uint8_t level) {
  const uint64_t p = level_scale(level);
  const uint64_t digit = (u / p) % 10ULL;
  return static_cast<float>(static_cast<double>(digit) + static_cast<double>(u % p) / static_cast<double>(p));
}

inline float expected_of(uint64_t u, uint8_t level, bool linear) {
  return linear ? dial_of(u, level) : digit_of(u, level);
}

class ReaderCore {
 public:
  Params params;

  void add_wheel(uint8_t level, float resolution, float tolerance, bool linear) {
    if (level >= this->wheels_.size())
      this->wheels_.resize(level + 1);
    Wheel w{};
    w.level = level;
    w.resolution = resolution;
    w.tolerance = tolerance;
    w.linear = linear;
    this->wheels_[level] = w;
  }

  const std::vector<Wheel> &wheels() const { return this->wheels_; }
  bool has_wheels() const { return !this->wheels_.empty(); }
  uint64_t u() const { return this->u_; }
  bool anchored() const { return this->anchored_; }
  uint32_t rejected() const { return this->rejected_; }
  ReaderStatus status() const { return this->status_; }

  float quantum() const {
    if (this->wheels_.empty())
      return 0.0f;
    return this->wheels_[0].resolution / 10.0f;
  }

  float reading_liters() const {
    return static_cast<float>(static_cast<double>(this->u_) * static_cast<double>(this->quantum()));
  }

  void start(uint32_t now, bool has_saved, uint64_t stored) {
    this->has_saved_ = has_saved;
    if (has_saved)
      this->u_ = stored;
    this->last_commit_ms_ = now;
    this->last_confirmed_ms_ = now;
    this->last_save_ms_ = now;
  }

  void observe(uint8_t level, float result, float fit, uint32_t now) {
    if (level >= this->wheels_.size())
      return;
    Wheel &w = this->wheels_[level];
    w.result = result;
    w.fit = fit;
    w.present = true;
    w.last_accepted_ms = now;
  }

  // Only the two finest occupied wheels gate staleness.
  bool update_stale(uint32_t now) {
    bool stale = false;
    uint8_t checked = 0;
    for (const auto &w : this->wheels_) {
      if (w.resolution == 0.0f)
        continue;
      if (checked >= 2)
        break;
      checked++;
      if (now - w.last_accepted_ms > this->params.stale_after_ms)
        stale = true;
    }
    if (stale == this->stale_)
      return false;
    this->stale_ = stale;
    return this->refresh_status_();
  }

  CycleResult end_cycle(uint32_t now) {
    CycleResult r;
    this->process_(now, &r);
    for (auto &w : this->wheels_)
      w.present = false;
    return r;
  }

  CycleResult rebase(float liters, uint32_t now) {
    CycleResult r;
    if (this->wheels_.empty() || std::isnan(liters) || liters < 0.0f)
      return r;
    this->u_ = static_cast<uint64_t>(std::llround(static_cast<double>(liters) / static_cast<double>(this->quantum())));
    this->has_pending_ = false;
    this->divergent_ = false;
    this->stale_ = false;
    this->inconsistent_since_ms_ = 0;
    this->anchored_ = true;
    this->last_commit_ms_ = now;
    this->last_confirmed_ms_ = now;
    this->last_save_ms_ = now;
    r.reading_changed = true;
    r.save = true;
    r.flush = true;
    r.status_changed = this->refresh_status_();
    return r;
  }

  float consistency_error(uint64_t u) const {
    float err = 0.0f;
    for (const auto &w : this->wheels_) {
      if (!w.present)
        continue;
      err = std::max(err, circdist(expected_of(u, w.level, w.linear), w.result));
    }
    return err;
  }

  bool consistent(uint64_t u) const {
    for (const auto &w : this->wheels_) {
      if (w.present && circdist(expected_of(u, w.level, w.linear), w.result) > w.tolerance)
        return false;
    }
    return true;
  }

 protected:
  uint64_t span_u_(float liters) const {
    const float quantum = this->quantum();
    if (quantum <= 0.0f)
      return 0;
    return static_cast<uint64_t>(std::ceil(liters / quantum));
  }

  bool within_(const Wheel &w, uint64_t c) const {
    return circdist(expected_of(c, w.level, w.linear), w.result) <= w.tolerance;
  }

  bool refresh_status_() {
    ReaderStatus status = STATUS_ANCHORING;
    if (this->anchored_) {
      if (this->divergent_) {
        status = STATUS_DIVERGENT;
      } else if (this->stale_) {
        status = STATUS_STALE;
      } else {
        status = STATUS_OK;
      }
    }
    if (status == this->status_)
      return false;
    this->status_ = status;
    return true;
  }

  bool find_consistent_(uint64_t center, uint64_t span, uint64_t *out) const {
    for (uint64_t off = 0; off <= span; ++off) {
      const uint64_t forward = center + off;
      if (this->consistent(forward)) {
        *out = forward;
        return true;
      }
      if (off > 0 && center >= off) {
        const uint64_t backward = center - off;
        if (this->consistent(backward)) {
          *out = backward;
          return true;
        }
      }
    }
    return false;
  }

  // Picks the candidate with the smallest fine wheel error, closest to u on
  // ties. Coarse wheels only veto. A coarse wheel that is outside tolerance
  // for every candidate that passes the finest wheel is misread, so it is
  // left out of this cycle.
  // `strict` tells whether any candidate matches every present wheel.
  bool select_(uint64_t lo, uint64_t hi, uint64_t *out, bool *strict, CycleResult *r) const {
    const Wheel &finest = this->wheels_[0];
    std::vector<uint8_t> agrees(this->wheels_.size(), 0);
    bool any = false;
    *strict = false;
    for (uint64_t c = lo; c <= hi; ++c) {
      if (!this->within_(finest, c))
        continue;
      any = true;
      bool all = true;
      for (size_t i = 1; i < this->wheels_.size(); ++i) {
        const Wheel &w = this->wheels_[i];
        if (!w.present)
          continue;
        if (this->within_(w, c))
          agrees[i] = 1;
        else
          all = false;
      }
      if (all)
        *strict = true;
    }
    if (!any)
      return false;
    for (size_t i = 1; i < this->wheels_.size(); ++i) {
      if (this->wheels_[i].present && !agrees[i])
        r->add_event(EventKind::WHEEL_EXCLUDED, this->wheels_[i].level);
    }

    bool found = false;
    float best_err = 0.0f;
    uint64_t best_dist = 0;
    for (uint64_t c = lo; c <= hi; ++c) {
      if (!this->within_(finest, c))
        continue;
      bool vetoed = false;
      float err = circdist(expected_of(c, 0, false), finest.result);
      for (size_t i = 1; i < this->wheels_.size() && !vetoed; ++i) {
        const Wheel &w = this->wheels_[i];
        if (!w.present || !agrees[i])
          continue;
        if (!this->within_(w, c))
          vetoed = true;
        else if (w.linear)
          err += circdist(expected_of(c, w.level, true), w.result);
      }
      if (vetoed)
        continue;
      const uint64_t dist = c >= this->u_ ? c - this->u_ : this->u_ - c;
      if (!found || err < best_err || (err == best_err && dist < best_dist)) {
        found = true;
        best_err = err;
        best_dist = dist;
        *out = c;
      }
    }
    return found;
  }

  void commit_(uint64_t u, uint32_t now, CycleResult *r) {
    r->committed = true;
    r->prev_u = this->u_;
    r->commit_dt_ms = now - this->last_commit_ms_;
    r->reading_changed = true;
    this->u_ = u;
    this->has_pending_ = false;
    this->last_commit_ms_ = now;
    this->last_confirmed_ms_ = now;
    if (now - this->last_save_ms_ >= SAVE_INTERVAL_MS) {
      this->last_save_ms_ = now;
      r->save = true;
    }
  }

  void anchor_(bool jump, uint32_t now, CycleResult *r) {
    this->anchored_ = true;
    this->divergent_ = false;
    this->inconsistent_since_ms_ = 0;
    this->has_pending_ = false;
    this->last_commit_ms_ = now;
    this->last_confirmed_ms_ = now;
    r->anchored = true;
    r->reading_changed = true;
    if (this->refresh_status_())
      r->status_changed = true;
    if (jump) {
      this->last_save_ms_ = now;
      r->save = true;
      r->flush = true;
    }
  }

  // The reading only counts as inconsistent when no value in the search window
  // matches the wheels. A lagging reading during flow still has a match.
  void track_divergence_(bool matched, uint32_t now, CycleResult *r) {
    if (matched) {
      this->inconsistent_since_ms_ = 0;
      if (this->divergent_) {
        this->divergent_ = false;
        if (this->refresh_status_())
          r->status_changed = true;
      }
      return;
    }
    if (this->inconsistent_since_ms_ == 0)
      this->inconsistent_since_ms_ = now;
    if (!this->divergent_ && now - this->inconsistent_since_ms_ > this->params.stale_after_ms) {
      this->divergent_ = true;
      r->add_event(EventKind::DIVERGENT, (now - this->inconsistent_since_ms_) / 1000);
      if (this->refresh_status_())
        r->status_changed = true;
    }
  }

  void process_(uint32_t now, CycleResult *r) {
    if (this->wheels_.empty() || !this->wheels_[0].present)
      return;
    r->ran = true;

    const float err = this->consistency_error(this->u_);
    r->has_consistency = this->anchored_;
    r->consistency_error = err;

    for (const auto &w : this->wheels_) {
      if (w.present && w.fit < r->min_fit)
        r->min_fit = w.fit;
    }
    r->has_fit = true;

    if (!this->anchored_) {
      if (this->has_saved_) {
        uint64_t candidate = 0;
        if (this->find_consistent_(this->u_, this->span_u_(this->params.reanchor_tolerance), &candidate)) {
          if (candidate != this->u_) {
            r->add_event(EventKind::REANCHOR, static_cast<int64_t>(candidate),
                         static_cast<int64_t>(candidate) - static_cast<int64_t>(this->u_));
          }
          this->u_ = candidate;
          this->anchor_(true, now, r);
        }
      }
      return;
    }

    const float quantum = this->quantum();
    uint64_t cap =
        static_cast<uint64_t>(std::ceil(this->params.max_flow * (now - this->last_confirmed_ms_) / 1000.0f / quantum)) +
        1;
    cap = std::min(cap, MAX_SEARCH_QUANTA);
    const uint64_t back = this->span_u_(this->params.back_tolerance);
    const uint64_t lo = this->u_ > back ? this->u_ - back : 0;

    uint64_t u_star = this->u_;
    bool strict = false;
    const bool found = this->select_(lo, this->u_ + cap, &u_star, &strict, r);
    this->track_divergence_(strict, now, r);
    if (!found) {
      this->rejected_++;
      r->rejected_changed = true;
      r->add_event(EventKind::VETO, static_cast<int64_t>(this->u_ - lo), static_cast<int64_t>(cap));
    } else {
      if (this->has_pending_) {
        const bool confirms = this->pending_u_ > this->u_ ? u_star >= this->pending_u_ : u_star <= this->pending_u_;
        if (confirms) {
          this->pending_confirmed_++;
          if (this->pending_confirmed_ >= this->params.corroborations)
            this->commit_(this->pending_u_, now, r);
        } else {
          this->has_pending_ = false;
          this->rejected_++;
          r->rejected_changed = true;
          r->add_event(EventKind::PHANTOM, static_cast<int64_t>(this->pending_u_));
        }
      }
      if (u_star == this->u_) {
        this->last_confirmed_ms_ = now;
      } else if (!this->has_pending_) {
        this->pending_u_ = u_star;
        this->pending_confirmed_ = 0;
        this->pending_ms_ = now;
        this->has_pending_ = true;
      }
    }

    if (this->has_pending_ && now - this->pending_ms_ > this->params.pending_window_ms)
      this->has_pending_ = false;
  }

  std::vector<Wheel> wheels_{};
  uint64_t u_{0};
  bool has_saved_{false};
  bool anchored_{false};
  bool stale_{false};
  bool divergent_{false};
  bool has_pending_{false};
  uint64_t pending_u_{0};
  uint32_t pending_confirmed_{0};
  uint32_t pending_ms_{0};
  uint32_t last_commit_ms_{0};
  uint32_t last_confirmed_ms_{0};
  uint32_t last_save_ms_{0};
  uint32_t inconsistent_since_ms_{0};
  uint32_t rejected_{0};
  ReaderStatus status_{STATUS_ANCHORING};
};

}  // namespace meter_reader_core
