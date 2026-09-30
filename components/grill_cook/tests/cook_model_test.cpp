#include "../cook_model.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using grill_cook_model::CookInputs;
using grill_cook_model::CookModel;
using grill_cook_model::Phase;
using grill_cook_model::Zone;

namespace {

constexpr uint32_t DT_S = 5;

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

  // Zone on, no probe yet: grilling.
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

void test_auto_start_and_grilling() {
  CookModel model;
  model.update(make_inputs(0, false, false, 20.0f, NAN, 90.0f));
  assert(!model.is_cooking());

  model.update(make_inputs(5, true, false, 25.0f, NAN, 90.0f));
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
  for (float v : model.meat_history()) {
    assert(std::isnan(v));
  }
  for (int z = 0; z < 2; z++) {
    for (float v : model.zone_history(static_cast<Zone>(z))) {
      assert(std::isnan(v));
    }
  }
}

}  // namespace

int main() {
  test_idle_without_input();
  test_auto_start_and_grilling();
  test_rate_and_remaining_minutes();
  test_stall_above_60();
  test_done_stays_after_dip();
  test_auto_end_after_cold_and_off();
  test_second_cook_matches_first();
  test_start_end_start_resets_state();

  std::printf("all cook_model tests passed\n");
  return 0;
}
