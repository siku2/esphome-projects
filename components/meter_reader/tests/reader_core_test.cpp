#include "../reader_core.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

using meter_reader_core::CycleResult;
using meter_reader_core::EventKind;
using meter_reader_core::ReaderCore;
using meter_reader_core::Wheel;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
      std::exit(1); \
    } \
  } while (0)

namespace {

constexpr double QUANTUM = 0.1;
constexpr uint32_t CYCLE_MS = 10000;
constexpr size_t LEVELS = 9;

struct Obs {
  bool present{false};
  float value{0.0f};
};
using Frame = std::array<Obs, LEVELS>;

float wrap(float v) { return std::fmod(v + 10.0f, 10.0f); }

// u is the true value in fractional quanta. Drums show the integer digit and
// creep towards the next one only near a roll. Linear dials show the exact fraction.
float render(double u, uint8_t level, bool linear) {
  const double x = u / static_cast<double>(meter_reader_core::level_scale(level));
  if (linear)
    return static_cast<float>(std::fmod(x, 10.0));
  const double digit = std::floor(x);
  const double roll = std::max(0.0, ((x - digit) - 0.8) / 0.2);
  return static_cast<float>(std::fmod(digit + 0.5 * roll, 10.0));
}

Frame render_frame(double u) {
  Frame f;
  for (uint8_t level = 0; level < LEVELS; ++level) {
    if (level == 2)
      continue;
    f[level] = Obs{true, render(u, level, level == 1)};
  }
  return f;
}

struct Rig {
  ReaderCore core;
  uint32_t now{1000};
  uint32_t last_flush_now{0};

  Rig() {
    core.add_wheel(0, 1.0f, 0.8f, false);
    core.add_wheel(1, 10.0f, 1.5f, true);
    core.add_wheel(3, 1000.0f, 1.5f, false);
    core.add_wheel(4, 10000.0f, 1.5f, false);
    core.add_wheel(5, 100000.0f, 1.5f, false);
    core.add_wheel(6, 1000000.0f, 1.5f, false);
    core.add_wheel(7, 10000000.0f, 1.5f, false);
    core.add_wheel(8, 100000000.0f, 1.5f, false);
    core.start(now, false, 0);
  }

  CycleResult feed(const Frame &f) {
    now += CYCLE_MS;
    for (uint8_t level = 0; level < LEVELS; ++level) {
      if (f[level].present)
        core.observe(level, f[level].value, 1.0f, now);
    }
    return core.end_cycle(now);
  }

  double liters() const { return core.reading_liters(); }
};

double liters_of(double quanta) { return quanta * QUANTUM; }

using Misread = std::function<void(int, Frame &)>;

Rig idle(int cycles, const Misread &misread) {
  Rig rig;
  rig.core.rebase(2414997.75f, rig.now);
  const uint64_t u0 = rig.core.u();
  const double truth = static_cast<double>(u0) + 0.2;
  for (int i = 0; i < cycles; ++i) {
    Frame f = render_frame(truth);
    misread(i, f);
    rig.feed(f);
    CHECK(rig.core.u() == u0);
  }
  CHECK(rig.core.rejected() == 0);
  return rig;
}

void idle_with_coarse_misreads_within_tolerance() {
  Rig rig = idle(1080, [](int i, Frame &f) {
    if (i % 3 == 0)
      f[4].value = wrap(f[4].value + 1.0f);
    if (i % 4 == 0)
      f[5].value = wrap(f[5].value - 1.0f);
  });
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

void idle_with_one_wheel_misread_in_some_cycles() {
  Rig rig = idle(1080, [](int i, Frame &f) {
    if (i % 5 < 2)
      f[6].value = 7.0f;
  });
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

void idle_with_one_wheel_misread_in_every_cycle() {
  idle(1080, [](int, Frame &f) { f[6].value = 7.0f; });
}

void idle_with_wheels_missing() {
  idle(1080, [](int i, Frame &f) {
    if (i % 2 == 0)
      f[4].present = false;
    if (i % 3 == 0)
      f[5].present = false;
    if (i % 7 == 0)
      f[6].present = false;
  });
}

void exclusion_is_reported() {
  Rig rig;
  rig.core.rebase(2414997.75f, rig.now);
  Frame f = render_frame(static_cast<double>(rig.core.u()) + 0.2);
  f[6].value = 7.0f;
  const CycleResult r = rig.feed(f);
  bool excluded = false;
  for (size_t i = 0; i < r.event_count; ++i) {
    if (r.events[i].kind == EventKind::WHEEL_EXCLUDED && r.events[i].a == 6)
      excluded = true;
  }
  CHECK(excluded);
  CHECK(!r.rejected_changed);
}

void steady_flow() {
  Rig rig;
  rig.core.rebase(2414000.0f, rig.now);
  double truth = static_cast<double>(rig.core.u());
  const double per_cycle = 0.15 * (CYCLE_MS / 1000.0) / QUANTUM;
  double last = rig.liters();
  for (int i = 0; i < 120; ++i) {
    truth += per_cycle;
    rig.feed(render_frame(truth));
    CHECK(rig.liters() >= last);
    last = rig.liters();
  }
  for (int i = 0; i < 3; ++i)
    rig.feed(render_frame(truth));
  CHECK(std::fabs(rig.liters() - liters_of(truth)) < 1.0);
  CHECK(rig.core.rejected() == 0);
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

Rig blind_flow(double flow_liters) {
  Rig rig;
  rig.core.rebase(2414000.0f, rig.now);
  double truth = static_cast<double>(rig.core.u());
  const int blind_cycles = 180;
  for (int i = 0; i < blind_cycles; ++i) {
    truth += flow_liters / QUANTUM / blind_cycles;
    Frame f = render_frame(truth);
    f[0].present = false;
    f[1].present = false;
    rig.feed(f);
  }
  for (int i = 0; i < 3; ++i)
    rig.feed(render_frame(truth));
  const double err = rig.liters() - liters_of(truth);
  if (flow_liters < 5.0) {
    CHECK(std::fabs(err) < 1.0);
  } else {
    // The 10 L disc has no sensor, so only the remainder of the flow is visible.
    const double rem = std::fmod(std::fabs(err), 10.0);
    CHECK(rem < 1.0 || rem > 9.0);
  }
  return rig;
}

void fine_wheels_missing_then_visible() {
  Rig small = blind_flow(4.3);
  CHECK(small.core.rejected() == 0);
  Rig large = blind_flow(40.0);
  CHECK(large.core.rejected() == 0);
}

void rebase_then_tracking() {
  Rig rig;
  rig.core.rebase(100.0f, rig.now);
  for (int i = 0; i < 10; ++i)
    rig.feed(render_frame(static_cast<double>(rig.core.u())));
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);

  const CycleResult r = rig.core.rebase(5000.0f, rig.now);
  CHECK(r.reading_changed && r.save && r.flush);
  CHECK(rig.core.u() == 50000);
  double truth = 50000.0;
  for (int i = 0; i < 30; ++i) {
    truth += 0.1 * (CYCLE_MS / 1000.0) / QUANTUM;
    rig.feed(render_frame(truth));
  }
  for (int i = 0; i < 3; ++i)
    rig.feed(render_frame(truth));
  CHECK(std::fabs(rig.liters() - liters_of(truth)) < 1.0);
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);

  const CycleResult bad = rig.core.rebase(-1.0f, rig.now);
  CHECK(!bad.reading_changed);
}

void boot_reanchors_a_saved_value_that_is_off() {
  Rig rig;
  const double truth = 24149978.2;
  const uint64_t saved = 24149978 + 303;
  rig.core.start(rig.now, true, saved);
  CHECK(rig.core.status() == meter_reader_core::STATUS_ANCHORING);

  const CycleResult first = rig.feed(render_frame(truth));
  CHECK(first.anchored && first.flush && first.save);
  CHECK(first.event_count == 1 && first.events[0].kind == EventKind::REANCHOR);
  CHECK(rig.core.anchored());
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
  CHECK(!first.has_consistency);
  for (int i = 0; i < 5; ++i)
    rig.feed(render_frame(truth));
  // Whole 10 L steps are invisible, so the value stays at the saved 30 L offset.
  CHECK(std::fabs(rig.liters() - liters_of(truth + 300.0)) < 0.3);
}

void boot_waits_while_no_candidate_is_consistent() {
  Rig rig;
  rig.core.start(rig.now, true, 24149978 + 303);
  Frame f = render_frame(24149978.2);
  f[6].value = 7.0f;
  rig.feed(f);
  CHECK(!rig.core.anchored());
  rig.feed(render_frame(24149978.2));
  CHECK(rig.core.anchored());
}

void wheels_far_from_reading_mark_it_divergent() {
  Rig rig;
  rig.core.rebase(2414997.75f, rig.now);
  const uint64_t u0 = rig.core.u();
  const double truth = static_cast<double>(u0) + 0.2 + 3000.0;
  CycleResult last;
  bool divergent_event = false;
  for (int i = 0; i < 10; ++i) {
    last = rig.feed(render_frame(truth));
    for (size_t e = 0; e < last.event_count; ++e)
      divergent_event |= last.events[e].kind == EventKind::DIVERGENT;
    CHECK(rig.core.u() == u0);
  }
  CHECK(divergent_event);
  CHECK(rig.core.status() == meter_reader_core::STATUS_DIVERGENT);

  last = rig.feed(render_frame(static_cast<double>(u0) + 0.2));
  CHECK(last.status_changed);
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

void silent_wheels_make_the_reading_stale() {
  Rig rig;
  rig.core.rebase(100.0f, rig.now);
  rig.core.observe(0, 0.0f, 1.0f, rig.now);
  rig.core.observe(1, 0.0f, 1.0f, rig.now);
  CHECK(!rig.core.update_stale(rig.now + 30000));
  CHECK(rig.core.update_stale(rig.now + 61000));
  CHECK(rig.core.status() == meter_reader_core::STATUS_STALE);
  rig.core.observe(0, 0.0f, 1.0f, rig.now + 62000);
  rig.core.observe(1, 0.0f, 1.0f, rig.now + 62000);
  CHECK(rig.core.update_stale(rig.now + 62000));
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

const char *const KONA_CYCLES[] = {"L0=8.15 L1=7.96 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.93 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.13 L1=7.94 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.96 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.97 L3=9.80 L4=4.50 L5=1.00 L6=7.00 L7=2.00 L8=0.10",
                                   "L0=8.16 L1=7.95 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.12 L1=7.96 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.93 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.96 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.13 L1=7.97 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.96 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.97 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.13 L1=7.96 L3=9.60 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.92 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.92 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.97 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.97 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.96 L3=9.80 L4=4.60 L5:- L6:- L7=2.00 L8=0.20",
                                   "L0=8.12 L1=7.94 L3=9.60 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.97 L3=9.60 L4=4.60 L5:- L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.97 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.96 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.16 L1=7.94 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.16 L1=7.97 L3=9.60 L4=4.60 L5=0.90 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.94 L3=9.60 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.12 L1=7.95 L3=9.60 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.96 L3=9.60 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.13 L1=7.93 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.12 L1=7.95 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.10",
                                   "L0=8.15 L1=7.95 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.95 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.93 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.93 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.94 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.93 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.95 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.94 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.96 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.95 L3=9.60 L4=4.50 L5:- L6:- L7=2.00 L8=0.40",
                                   "L0=8.13 L1=7.96 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.12 L1=7.96 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.96 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.16 L1=7.96 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.96 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.97 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.94 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.14 L1=7.97 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.13 L1=7.98 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=8.15 L1=7.93 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=8.15 L1=7.94 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=8.14 L1=7.96 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=8.13 L1=7.95 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=1.72 L1=8.12 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.10",
                                   "L0=3.26 L1=8.26 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.24 L1=8.26 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.25 L1=8.26 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.24 L1=8.24 L3=9.60 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=3.36 L1=8.29 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.30 L3=9.60 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.31 L3=9.80 L4=4.50 L5:- L6:- L7=2.00 L8=0.20",
                                   "L0=3.43 L1=8.29 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.41 L1=8.32 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=3.39 L1=8.27 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.32 L3=9.60 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.10",
                                   "L0=3.43 L1=8.33 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.31 L3=9.80 L4=4.50 L5:- L6=7.00 L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.31 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.10",
                                   "L0=3.40 L1=8.29 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.31 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=3.41 L1=8.29 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.41 L1=8.32 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.30 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.39 L1=8.29 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.40 L1=8.28 L3=9.60 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.29 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.42 L1=8.31 L3=9.60 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.32 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=3.42 L1=8.28 L3=9.80 L4=4.60 L5=1.00 L6=7.00 L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.28 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=3.40 L1=8.28 L3=9.80 L4=4.50 L5:- L6:- L7=2.00 L8=0.20",
                                   "L0=3.43 L1=8.28 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.42 L1=8.29 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=3.38 L1=8.29 L3=9.60 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.20",
                                   "L0=3.40 L1=8.29 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=3.42 L1=8.31 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.43 L1=8.29 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.43 L1=8.31 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.41 L1=8.28 L3=9.80 L4=4.50 L5=0.90 L6:- L7=2.00 L8=0.40",
                                   "L0=3.40 L1=8.32 L3=9.80 L4=4.50 L5=1.00 L6:- L7=2.00 L8=0.20",
                                   "L0=3.41 L1=8.31 L3=9.80 L4=4.50 L5:- L6:- L7=2.00 L8=0.40",
                                   "L0=3.41 L1=8.29 L3=9.80 L4=4.60 L5=1.00 L6:- L7=2.00 L8=0.40",
                                   "L0=3.37 L1=8.29 L3=9.80 L4=4.60 L5=0.90 L6:- L7=2.00 L8=0.40"};

Frame parse_cycle(const std::string &line) {
  Frame f;
  std::istringstream in(line);
  std::string tok;
  while (in >> tok) {
    const unsigned level = static_cast<unsigned>(tok[1] - '0');
    CHECK(level < LEVELS);
    if (tok[2] == '=')
      f[level] = Obs{true, std::stof(tok.substr(3))};
  }
  return f;
}

// The decision of the previous logic for a fixed reading: the candidate with
// the smallest error summed over all wheels, vetoed by every coarse wheel.
// The window is measured from the last change, which is the start here.
bool legacy_pick(const Frame &f, uint64_t u, uint64_t cap, uint64_t *out) {
  const uint64_t back = 200;
  bool found = false;
  float best = 0.0f;
  for (uint64_t c = u - back; c <= u + cap; ++c) {
    float sum = 0.0f;
    bool ok = true;
    for (uint8_t level = 0; level < LEVELS && ok; ++level) {
      if (!f[level].present)
        continue;
      const float tol = level == 0 ? 0.8f : 1.5f;
      const float d = meter_reader_core::circdist(meter_reader_core::expected_of(c, level, level == 1), f[level].value);
      ok = d <= tol;
      sum += d;
    }
    const uint64_t dist = c >= u ? c - u : u - c;
    const uint64_t best_dist = *out >= u ? *out - u : u - *out;
    if (ok && (!found || sum < best || (sum == best && dist < best_dist))) {
      found = true;
      best = sum;
      *out = c;
    }
  }
  return found;
}

void replay_of_recorded_kona_cycles() {
  Rig rig;
  rig.core.rebase(2415017.75f, rig.now);
  const uint64_t u0 = rig.core.u();
  const double start = rig.liters();
  double last = start;
  double lo = start;
  double hi = start;
  double max_step = 0.0;
  int legacy_vetoed = 0;
  int legacy_moved = 0;
  uint64_t legacy_far = 0;
  uint64_t legacy_cap = 1;
  for (const char *line : KONA_CYCLES) {
    const Frame f = parse_cycle(line);
    rig.feed(f);
    const double now = rig.liters();
    max_step = std::max(max_step, std::fabs(now - last));
    lo = std::min(lo, now);
    hi = std::max(hi, now);
    last = now;

    uint64_t pick = u0;
    legacy_cap += 150;
    if (!legacy_pick(f, u0, legacy_cap, &pick)) {
      legacy_vetoed++;
    } else if (pick != u0) {
      legacy_moved++;
      legacy_far = std::max(legacy_far, pick > u0 ? pick - u0 : u0 - pick);
    }
  }
  std::printf("replay: moved %.2f L, max step %.2f L, rejected %u, status %d\n", last - start, max_step,
              static_cast<unsigned>(rig.core.rejected()), static_cast<int>(rig.core.status()));
  std::printf("previous logic on the same cycles: %d vetoed, %d moved by up to %.1f L\n", legacy_vetoed, legacy_moved,
              liters_of(static_cast<double>(legacy_far)));
  CHECK(std::fabs(last - start) < 5.0);
  CHECK(hi - lo < 5.0);
  CHECK(max_step < 2.0);
  CHECK(rig.core.rejected() == 0);
  CHECK(rig.core.status() == meter_reader_core::STATUS_OK);
}

}  // namespace

int main() {
  idle_with_coarse_misreads_within_tolerance();
  idle_with_one_wheel_misread_in_some_cycles();
  idle_with_one_wheel_misread_in_every_cycle();
  idle_with_wheels_missing();
  exclusion_is_reported();
  steady_flow();
  fine_wheels_missing_then_visible();
  rebase_then_tracking();
  boot_reanchors_a_saved_value_that_is_off();
  boot_waits_while_no_candidate_is_consistent();
  wheels_far_from_reading_mark_it_divergent();
  silent_wheels_make_the_reading_stale();
  replay_of_recorded_kona_cycles();
  std::puts("meter_reader tests passed");
  return 0;
}
