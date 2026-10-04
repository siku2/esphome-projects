#include "../cook_model.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

using grill_cook_model::CookInputs;
using grill_cook_model::CookModel;
using grill_cook_model::Phase;
using grill_cook_model::Zone;

namespace {

constexpr uint32_t DT_S = 5;
constexpr size_t HISTORY_LAST = CookModel::HISTORY_LEN - 1;

CookInputs make_inputs(uint32_t t_s, bool west_on, bool east_on, float zone_temp, float meat_temp, float meat_target) {
  CookInputs in{};
  in.t_s = t_s;
  in.zone_on[0] = west_on;
  in.zone_on[1] = east_on;
  in.zone_target[0] = 120.0f;
  in.zone_target[1] = 120.0f;
  in.zone_temp[0] = zone_temp;
  in.zone_temp[1] = zone_temp;
  in.meat_temp = meat_temp;
  in.meat_target = meat_target;
  in.meat_fresh = true;
  return in;
}

// Runs a fixed cook profile against `model`, appending the phase observed at
// every tick to `trace`. Returns the elapsed simulated seconds.
uint32_t run_cook_profile(CookModel &model, uint32_t t0, std::vector<Phase> &trace) {
  const float meat_target = 90.0f;
  const float rise_c_per_min = 3.0f;
  uint32_t t = t0;

  // 2 minutes idle before the zone turns on.
  for (int i = 0; i < 24; i++) {
    model.update(make_inputs(t, false, false, 20.0f, NAN, meat_target));
    trace.push_back(model.phase());
    t += DT_S;
  }

  // Zone on, no probe yet: grilling. The cook starts by hand.
  model.start();
  for (int i = 0; i < 24; i++) {
    model.update(make_inputs(t, true, false, 25.0f, NAN, meat_target));
    trace.push_back(model.phase());
    t += DT_S;
  }

  // Probe inserted, linear rise up to the target.
  const uint32_t rise_start = t;
  const float start_temp = 20.0f;
  while (true) {
    const float elapsed_min = (t - rise_start) / 60.0f;
    const float meat_temp = start_temp + rise_c_per_min * elapsed_min;
    if (meat_temp >= meat_target)
      break;
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, meat_target));
    trace.push_back(model.phase());
    t += DT_S;
  }

  // Hold at the target for a minute, then zones off and the meat cools.
  for (int i = 0; i < 12; i++) {
    model.update(make_inputs(t, true, false, 130.0f, meat_target, meat_target));
    trace.push_back(model.phase());
    t += DT_S;
  }
  for (int i = 0; i < 65; i++) {  // 65 * 5s > 300s of continuous cold-and-off
    model.update(make_inputs(t, false, false, 25.0f, 20.0f, meat_target));
    trace.push_back(model.phase());
    t += DT_S;
  }

  return t;
}

void test_idle_without_input() {
  CookModel model;
  assert(!model.is_cooking());
  assert(model.phase() == Phase::IDLE);
  assert(std::strcmp(model.phase_str(), "idle") == 0);
  assert(std::isnan(model.meat_rate()));
  assert(model.remaining_minutes() == -1);
  assert(!model.meat_probe_present());
}

void test_zone_on_does_not_start_cook() {
  CookModel model;
  model.update(make_inputs(0, false, false, 20.0f, NAN, 90.0f));
  assert(!model.is_cooking());
  assert(model.phase() == Phase::IDLE);

  model.update(make_inputs(5, true, false, 25.0f, NAN, 90.0f));
  assert(!model.is_cooking());
  assert(model.phase() == Phase::GRILLING);

  model.update(make_inputs(10, false, true, 25.0f, 20.0f, 90.0f));
  assert(!model.is_cooking());
  assert(model.phase() == Phase::GRILLING);

  model.update(make_inputs(15, false, false, 25.0f, 20.0f, 90.0f));
  assert(model.phase() == Phase::IDLE);
}

void test_start_gives_grilling_without_probe() {
  CookModel model;
  model.update(make_inputs(0, true, false, 25.0f, NAN, 90.0f));
  model.start();
  assert(model.is_cooking());
  assert(model.phase() == Phase::GRILLING);
  assert(!model.meat_probe_present());
}

void test_rate_and_remaining_minutes() {
  CookModel model;
  model.start();

  const float meat_target = 90.0f;
  const float rise_c_per_min = 3.0f;
  const float start_temp = 20.0f;
  uint32_t t = 0;
  float meat_temp = start_temp;

  // 20 minutes of linear rise: more than the 15 minute rate window and the
  // 5 minute minimum span.
  while (t <= 20 * 60) {
    meat_temp = start_temp + rise_c_per_min * (t / 60.0f);
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, meat_target));
    t += DT_S;
  }

  const float measured_rate = model.meat_rate();
  assert(!std::isnan(measured_rate));
  assert(std::fabs(measured_rate - rise_c_per_min) <= 0.05f * rise_c_per_min);

  const float remaining_expected = (meat_target - meat_temp) / measured_rate;
  const int expected_minutes = static_cast<int>(std::lround(remaining_expected));
  assert(model.remaining_minutes() == expected_minutes);
}

void test_stall_above_60() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  // Ramp from 20 to 65 over 10 minutes.
  for (; t <= 10 * 60; t += DT_S) {
    const float meat_temp = 20.0f + 4.5f * (t / 60.0f);
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, 90.0f));
  }
  // Hold flat at 65 for 20 more minutes.
  Phase last_phase = Phase::IDLE;
  for (uint32_t held = 0; held <= 20 * 60; held += DT_S, t += DT_S) {
    model.update(make_inputs(t, true, false, 130.0f, 65.0f, 90.0f));
    last_phase = model.phase();
  }
  assert(last_phase == Phase::STALLED);
}

void test_done_stays_after_dip() {
  CookModel model;
  model.start();

  const float meat_target = 90.0f;
  uint32_t t = 0;
  for (; t <= 20 * 60; t += DT_S) {
    const float meat_temp = 20.0f + 4.0f * (t / 60.0f);
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, meat_target));
  }
  model.update(make_inputs(t, true, false, 130.0f, meat_target, meat_target));
  t += DT_S;
  assert(model.phase() == Phase::DONE);

  model.update(make_inputs(t, true, false, 130.0f, meat_target - 5.0f, meat_target));
  assert(model.phase() == Phase::DONE);
}

void test_auto_end_after_cold_and_off() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  model.update(make_inputs(t, true, false, 130.0f, 80.0f, 90.0f));
  t += DT_S;

  // Zones off, meat cold, starting now.
  const uint32_t off_start = t;
  while (t - off_start < 295) {
    model.update(make_inputs(t, false, false, 25.0f, 20.0f, 90.0f));
    assert(model.is_cooking());
    t += DT_S;
  }
  while (model.is_cooking() && t - off_start < 400) {
    model.update(make_inputs(t, false, false, 25.0f, 20.0f, 90.0f));
    t += DT_S;
  }
  assert(!model.is_cooking());
}

void test_second_cook_matches_first() {
  CookModel model;
  std::vector<Phase> trace1;
  std::vector<Phase> trace2;

  uint32_t t = run_cook_profile(model, 0, trace1);
  assert(!model.is_cooking());
  run_cook_profile(model, t, trace2);
  assert(!model.is_cooking());

  assert(trace1.size() == trace2.size());
  for (size_t i = 0; i < trace1.size(); i++) {
    assert(trace1[i] == trace2[i]);
  }
}

void test_start_end_start_resets_state() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  for (; t <= 10 * 60; t += DT_S) {
    const float meat_temp = 20.0f + 7.0f * (t / 60.0f);
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, 90.0f));
  }
  assert(!std::isnan(model.meat_rate()));

  model.end();
  model.start();

  assert(std::isnan(model.meat_rate()));
  assert(model.remaining_minutes() == -1);
  assert(model.phase() != Phase::DONE);
}

// Feeds `ticks` samples with the zone on at 130 C and returns the next time.
uint32_t feed(CookModel &model, uint32_t t, int ticks, bool zone_on, float meat_temp, float meat_target) {
  for (int i = 0; i < ticks; i++) {
    model.update(make_inputs(t, zone_on, false, 130.0f, meat_temp, meat_target));
    t += DT_S;
  }
  return t;
}

// Raises the meat from `start` to just below `target` at 3 C/min.
uint32_t ramp_below(CookModel &model, uint32_t t, float start, float target) {
  const uint32_t t0 = t;
  while (true) {
    const float meat_temp = start + 3.0f * (t - t0) / 60.0f;
    if (meat_temp >= target - 1.0f)
      return t;
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, target));
    t += DT_S;
  }
}

void test_hot_probe_before_insertion_is_not_done() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  for (int i = 0; i < 12; i++) {
    model.update(make_inputs(t, true, false, 130.0f, 120.0f, 90.0f));
    assert(model.phase() != Phase::DONE);
    t += DT_S;
  }
  for (int i = 0; i < 120; i++) {
    model.update(make_inputs(t, true, false, 130.0f, 8.0f, 90.0f));
    assert(model.phase() != Phase::DONE);
    t += DT_S;
  }
}

void test_done_latch_and_release_on_higher_target() {
  CookModel model;
  model.start();

  uint32_t t = ramp_below(model, 0, 20.0f, 90.0f);
  t = feed(model, t, 1, true, 90.0f, 90.0f);
  assert(model.phase() == Phase::DONE);

  t = feed(model, t, 6, true, 88.0f, 90.0f);
  assert(model.phase() == Phase::DONE);

  t = feed(model, t, 1, true, 88.0f, 95.0f);
  assert(model.phase() == Phase::COOKING);

  t = feed(model, t, 1, true, 95.0f, 95.0f);
  assert(model.phase() == Phase::DONE);
}

void test_lowered_target_gives_done_once() {
  CookModel model;
  model.start();

  uint32_t t = ramp_below(model, 0, 20.0f, 90.0f);
  t = feed(model, t, 3, true, 70.0f, 90.0f);
  assert(model.phase() == Phase::COOKING);

  int done_entries = 0;
  Phase prev = model.phase();
  for (float target : {65.0f, 65.0f, 60.0f, 50.0f, 40.0f}) {
    for (int i = 0; i < 6; i++) {
      model.update(make_inputs(t, true, false, 130.0f, 70.0f, target));
      t += DT_S;
      assert(model.phase() == Phase::DONE);
      if (prev != Phase::DONE)
        done_entries++;
      prev = model.phase();
    }
  }
  assert(done_entries == 1);
}

void test_cooling_with_zones_off_is_not_stalled() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  for (; t <= 15 * 60; t += DT_S) {
    const float meat_temp = 66.0f - 0.3f * (t / 60.0f);
    model.update(make_inputs(t, false, false, 25.0f, meat_temp, 90.0f));
    assert(model.phase() == Phase::COOKING);
  }
  assert(model.meat_rate() < -0.25f);
}

void test_stall_hysteresis() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  float meat_temp = 20.0f;
  int stalled_entries = 0;
  bool was_stalled = false;
  bool stall_reached = false;
  bool stall_left_early = false;

  auto tick = [&](float c_per_min) {
    meat_temp += c_per_min * DT_S / 60.0f;
    model.update(make_inputs(t, true, false, 130.0f, meat_temp, 90.0f));
    t += DT_S;
    const bool stalled = model.phase() == Phase::STALLED;
    if (stalled && !was_stalled)
      stalled_entries++;
    if (!stalled && was_stalled && stall_reached && !std::isnan(model.meat_rate()) && model.meat_rate() < 0.2f)
      stall_left_early = true;
    was_stalled = stalled;
  };

  for (int i = 0; i < 120; i++)
    tick(4.5f);
  for (int i = 0; i < 15 * 12 + 24; i++)
    tick(0.0f);
  assert(model.phase() == Phase::STALLED);
  stall_reached = true;

  // The rate wanders between 0.08 and 0.15 C/min.
  for (int round = 0; round < 3; round++) {
    for (int i = 0; i < 20 * 12; i++)
      tick(0.08f);
    for (int i = 0; i < 20 * 12; i++)
      tick(0.15f);
    assert(model.phase() == Phase::STALLED);
  }
  assert(stalled_entries == 1);
  assert(!stall_left_early);

  for (int i = 0; i < 20 * 12; i++)
    tick(0.5f);
  assert(model.meat_rate() >= 0.2f);
  assert(model.phase() == Phase::COOKING);
  assert(stalled_entries == 1);
}

void test_end_cook_with_zones_on_is_not_restarted() {
  CookModel model;
  model.start();
  uint32_t t = feed(model, 0, 10, true, 40.0f, 90.0f);
  assert(model.is_cooking());

  model.end();
  assert(model.phase() == Phase::GRILLING);
  feed(model, t, 40, true, 40.0f, 90.0f);
  assert(!model.is_cooking());
  assert(model.phase() == Phase::GRILLING);
}

void test_probe_dropout_and_return() {
  CookModel model;
  model.start();

  uint32_t t = ramp_below(model, 0, 20.0f, 90.0f);
  t = feed(model, t, 6, true, 50.0f, 90.0f);
  assert(model.phase() == Phase::COOKING);
  assert(model.meat_probe_present());

  for (int i = 0; i < 24; i++) {
    model.update(make_inputs(t, true, false, 130.0f, NAN, 90.0f));
    t += DT_S;
    assert(model.phase() == Phase::GRILLING);
    assert(!model.meat_probe_present());
  }

  t = feed(model, t, 6, true, 52.0f, 90.0f);
  assert(model.phase() == Phase::COOKING);
  assert(model.meat_probe_present());
}

// The latch is kept while the probe is absent, so DONE comes back with it.
void test_done_latch_survives_probe_dropout() {
  CookModel model;
  model.start();

  uint32_t t = ramp_below(model, 0, 20.0f, 90.0f);
  t = feed(model, t, 2, true, 90.0f, 90.0f);
  assert(model.phase() == Phase::DONE);

  t = feed(model, t, 24, true, NAN, 90.0f);
  assert(model.phase() == Phase::GRILLING);

  t = feed(model, t, 2, true, 85.0f, 90.0f);
  assert(model.phase() == Phase::DONE);
}

void test_timestamps_near_counter_limits() {
  CookModel model;
  std::vector<Phase> reference;
  run_cook_profile(model, 0, reference);

  // 4294967 s is where a 32 bit millisecond counter wraps. 2^32 s is where
  // the model's own seconds counter wraps.
  for (uint32_t t0 : {4294967u - 400u, UINT32_MAX - 400u}) {
    CookModel other;
    std::vector<Phase> trace;
    run_cook_profile(other, t0, trace);
    assert(!other.is_cooking());
    assert(trace == reference);
  }
}

void test_auto_end_restarts_when_zone_turns_on() {
  CookModel model;
  model.start();

  uint32_t t = feed(model, 0, 1, true, 40.0f, 90.0f);
  t = feed(model, t, 50, false, 20.0f, 90.0f);  // 250 s
  assert(model.is_cooking());

  t = feed(model, t, 1, true, 20.0f, 90.0f);
  t = feed(model, t, 50, false, 20.0f, 90.0f);  // another 250 s
  assert(model.is_cooking());

  t = feed(model, t, 15, false, 20.0f, 90.0f);
  assert(!model.is_cooking());
}

struct Tick {
  Phase phase;
  float rate;
  int remaining;
};

bool same_float(float a, float b) {
  if (std::isnan(a) || std::isnan(b))
    return std::isnan(a) && std::isnan(b);
  return std::fabs(a - b) <= 1e-4f;
}

uint32_t run_traced_cook(CookModel &model, uint32_t t, std::vector<Tick> &trace) {
  const float meat_target = 80.0f;
  auto tick = [&](bool zone_on, float meat_temp) {
    model.update(make_inputs(t, zone_on, zone_on, 130.0f, meat_temp, meat_target));
    trace.push_back({model.phase(), model.meat_rate(), model.remaining_minutes()});
    t += DT_S;
  };

  for (int i = 0; i < 12; i++)
    tick(true, NAN);
  for (int i = 0; i < 12 * 20; i++)
    tick(true, 20.0f + 3.0f * i * DT_S / 60.0f);
  for (int i = 0; i < 12 * 5; i++)
    tick(true, meat_target);
  for (int i = 0; i < 12 * 3; i++)
    tick(true, 75.0f);
  return t;
}

void test_second_traced_cook_matches_first() {
  CookModel model;
  std::vector<Tick> first;
  std::vector<Tick> second;

  model.start();
  uint32_t t = run_traced_cook(model, 1000, first);
  assert(model.phase() == Phase::DONE);
  model.end();
  assert(model.phase() == Phase::GRILLING);

  model.start();
  run_traced_cook(model, t, second);

  assert(first.size() == second.size());
  for (size_t i = 0; i < first.size(); i++) {
    assert(first[i].phase == second[i].phase);
    assert(same_float(first[i].rate, second[i].rate));
    assert(first[i].remaining == second[i].remaining);
  }
}

size_t valid_count(const std::array<float, CookModel::HISTORY_LEN> &history) {
  size_t count = 0;
  for (float v : history) {
    if (!std::isnan(v))
      count++;
  }
  return count;
}

void test_history_fills_while_idle() {
  CookModel model;
  assert(valid_count(model.meat_history()) == 0);

  uint32_t t = 0;
  for (; t < 10 * 60; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 25.0f, 90.0f));
  }
  assert(!model.is_cooking());
  assert(valid_count(model.meat_history()) == 20);
  assert(valid_count(model.zone_history(Zone::WEST)) == 20);
  assert(valid_count(model.zone_history(Zone::EAST)) == 20);
  assert(model.meat_history().back() == 25.0f);
}

void test_history_survives_cook_start_and_end() {
  CookModel model;
  uint32_t t = 0;
  for (; t < 5 * 60; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 25.0f, 90.0f));
  }
  const size_t idle_count = valid_count(model.meat_history());
  assert(idle_count == 10);

  model.start();
  model.update(make_inputs(t, true, false, 130.0f, 25.0f, 90.0f));
  assert(model.is_cooking());
  assert(valid_count(model.meat_history()) == idle_count + 1);
  assert(model.meat_history()[HISTORY_LAST - idle_count] == 25.0f);

  model.end();
  assert(valid_count(model.meat_history()) == idle_count + 1);

  t += DT_S;
  for (; t < 40 * 60; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 25.0f, 90.0f));
  }
  assert(valid_count(model.meat_history()) == CookModel::HISTORY_LEN);
}

CookInputs zone_inputs(uint32_t t_s, float west_temp, float east_temp) {
  CookInputs in = make_inputs(t_s, false, false, 20.0f, NAN, 90.0f);
  in.zone_temp[0] = west_temp;
  in.zone_temp[1] = east_temp;
  return in;
}

void test_zone_presence_takes_first_sample() {
  CookModel model;
  assert(!model.zone_present(Zone::WEST));
  model.update(zone_inputs(0, 20.0f, NAN));
  assert(model.zone_present(Zone::WEST));
  assert(!model.zone_present(Zone::EAST));
}

void test_zone_presence_debounce() {
  CookModel model;
  model.update(zone_inputs(0, 20.0f, 20.0f));
  assert(model.zone_present(Zone::WEST));

  model.update(zone_inputs(5, NAN, 20.0f));
  model.update(zone_inputs(10, NAN, 20.0f));
  assert(model.zone_present(Zone::WEST));
  model.update(zone_inputs(15, NAN, 20.0f));
  assert(!model.zone_present(Zone::WEST));
  assert(model.zone_present(Zone::EAST));

  model.update(zone_inputs(20, 20.0f, 20.0f));
  model.update(zone_inputs(29, 20.0f, 20.0f));
  assert(!model.zone_present(Zone::WEST));
  model.update(zone_inputs(30, 20.0f, 20.0f));
  assert(model.zone_present(Zone::WEST));
}

void test_zone_presence_glitch_is_ignored() {
  CookModel model;
  model.update(zone_inputs(0, 20.0f, 20.0f));
  model.update(zone_inputs(5, NAN, 20.0f));
  model.update(zone_inputs(10, NAN, 20.0f));
  model.update(zone_inputs(14, 20.0f, 20.0f));
  model.update(zone_inputs(19, NAN, 20.0f));
  model.update(zone_inputs(25, NAN, 20.0f));
  assert(model.zone_present(Zone::WEST));
  model.update(zone_inputs(29, NAN, 20.0f));
  assert(!model.zone_present(Zone::WEST));
}

void test_absent_zone_is_not_grilling() {
  CookModel model;
  CookInputs in = zone_inputs(0, NAN, 20.0f);
  in.zone_on[0] = true;
  model.update(in);
  assert(!model.is_cooking());
  assert(model.phase() == Phase::IDLE);
}

void test_cold_meat_that_never_warmed_survives() {
  CookModel model;
  model.update(make_inputs(0, false, false, 20.0f, 10.0f, 90.0f));
  model.start();
  uint32_t t = 5;
  for (; t < 60 * 60; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 10.0f, 90.0f));
  }
  assert(model.is_cooking());
}

void test_warmed_then_cold_ends_after_five_minutes() {
  CookModel model;
  model.start();
  uint32_t t = feed(model, 0, 12, true, 50.0f, 90.0f);
  assert(model.is_cooking());

  const uint32_t off_start = t;
  for (; t - off_start < 295; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 20.0f, 90.0f));
    assert(model.is_cooking());
  }
  for (; t - off_start < 310; t += DT_S) {
    model.update(make_inputs(t, false, false, 20.0f, 20.0f, 90.0f));
  }
  assert(!model.is_cooking());
}

void test_zone_on_blocks_auto_end() {
  CookModel model;
  model.start();
  uint32_t t = feed(model, 0, 12, true, 50.0f, 90.0f);
  t = feed(model, t, 12 * 20, true, 20.0f, 90.0f);
  assert(model.is_cooking());
}

void test_probe_absent_with_zones_off_ends() {
  CookModel model;
  model.start();
  uint32_t t = feed(model, 0, 12, false, NAN, 90.0f);
  assert(model.is_cooking());
  t = feed(model, t, 12 * 4 + 2, false, NAN, 90.0f);
  assert(!model.is_cooking());
}

void test_warmed_flag_clears_on_end() {
  CookModel model;
  model.start();
  uint32_t t = feed(model, 0, 12, true, 50.0f, 90.0f);
  model.end();
  model.start();
  t = feed(model, t, 12 * 20, false, 10.0f, 90.0f);
  assert(model.is_cooking());
}

void test_remaining_minutes_zero_after_done() {
  CookModel model;
  model.start();

  uint32_t t = ramp_below(model, 0, 20.0f, 90.0f);
  t = feed(model, t, 1, true, 90.0f, 90.0f);
  assert(model.phase() == Phase::DONE);
  assert(model.remaining_minutes() == 0);

  t = feed(model, t, 6, true, 85.0f, 90.0f);
  assert(model.phase() == Phase::DONE);
  assert(model.remaining_minutes() == 0);
}

void test_rate_needs_one_minute_of_samples() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  for (; t < 60; t += DT_S) {
    model.update(make_inputs(t, true, false, 130.0f, 20.0f + 3.0f * t / 60.0f, 90.0f));
    assert(std::isnan(model.meat_rate()));
    assert(model.remaining_minutes() == -1);
  }
  model.update(make_inputs(t, true, false, 130.0f, 20.0f + 3.0f * t / 60.0f, 90.0f));
  assert(!std::isnan(model.meat_rate()));
  assert(std::fabs(model.meat_rate() - 3.0f) < 0.1f);
  assert(model.remaining_minutes() > 0);
}

CookInputs other_event(CookInputs in) {
  in.meat_fresh = false;
  return in;
}

void test_non_meat_events_add_no_rate_samples() {
  CookModel model;
  model.start();

  uint32_t t = 0;
  for (; t <= 5 * 60; t += DT_S) {
    model.update(make_inputs(t, true, false, 130.0f, 20.0f + 3.0f * t / 60.0f, 90.0f));
  }
  const float rate = model.meat_rate();
  assert(std::fabs(rate - 3.0f) < 0.1f);

  // A flat meat temperature reported without a new reading must not drag the rate down.
  for (int i = 0; i < 600; i++, t++) {
    model.update(other_event(make_inputs(t, true, false, 130.0f, 35.0f, 90.0f)));
    assert(model.meat_rate() == rate);
  }
}

void test_first_rate_sample_needs_fresh_reading() {
  CookModel model;
  model.start();

  for (uint32_t t = 0; t <= 10 * 60; t += DT_S) {
    model.update(other_event(make_inputs(t, true, false, 130.0f, 20.0f + 3.0f * t / 60.0f, 90.0f)));
    assert(std::isnan(model.meat_rate()));
  }
}

void test_rate_window_spans_15_minutes_at_2s_readings() {
  CookModel model;
  model.start();

  // Flat for 10 minutes, then 3 C/min for 10 more. The window is 15 minutes, so the
  // slope is that of a fit over the last 5 flat minutes and the full ramp.
  const uint32_t read_step_s = 2;
  const uint32_t end_s = 20 * 60;
  auto meat_at = [](uint32_t t) { return t < 600 ? 20.0f : 20.0f + 3.0f * (t - 600) / 60.0f; };

  for (uint32_t t = 0; t <= end_s; t += read_step_s) {
    model.update(make_inputs(t, true, false, 130.0f, meat_at(t), 200.0f));
  }

  double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (uint32_t t = end_s - 15 * 60; t <= end_s; t += read_step_s) {
    const double x = t - (end_s - 15 * 60);
    const double y = meat_at(t);
    n++;
    sx += x;
    sy += y;
    sxx += x * x;
    sxy += x * y;
  }
  const double expected = (n * sxy - sx * sy) / (n * sxx - sx * sx) * 60.0;
  assert(std::fabs(model.meat_rate() - expected) < 0.05 * expected);
}

}  // namespace

int main() {
  test_idle_without_input();
  test_zone_on_does_not_start_cook();
  test_start_gives_grilling_without_probe();
  test_rate_and_remaining_minutes();
  test_stall_above_60();
  test_done_stays_after_dip();
  test_auto_end_after_cold_and_off();
  test_second_cook_matches_first();
  test_start_end_start_resets_state();
  test_hot_probe_before_insertion_is_not_done();
  test_done_latch_and_release_on_higher_target();
  test_lowered_target_gives_done_once();
  test_cooling_with_zones_off_is_not_stalled();
  test_stall_hysteresis();
  test_end_cook_with_zones_on_is_not_restarted();
  test_probe_dropout_and_return();
  test_done_latch_survives_probe_dropout();
  test_timestamps_near_counter_limits();
  test_auto_end_restarts_when_zone_turns_on();
  test_second_traced_cook_matches_first();
  test_history_fills_while_idle();
  test_history_survives_cook_start_and_end();
  test_zone_presence_takes_first_sample();
  test_zone_presence_debounce();
  test_zone_presence_glitch_is_ignored();
  test_absent_zone_is_not_grilling();
  test_cold_meat_that_never_warmed_survives();
  test_warmed_then_cold_ends_after_five_minutes();
  test_zone_on_blocks_auto_end();
  test_probe_absent_with_zones_off_ends();
  test_warmed_flag_clears_on_end();
  test_remaining_minutes_zero_after_done();
  test_rate_needs_one_minute_of_samples();
  test_non_meat_events_add_no_rate_samples();
  test_first_rate_sample_needs_fresh_reading();
  test_rate_window_spans_15_minutes_at_2s_readings();

  std::printf("all cook_model tests passed\n");
  return 0;
}
