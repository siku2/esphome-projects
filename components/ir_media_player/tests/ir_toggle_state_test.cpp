#include "../ir_toggle_state.h"

#include <cstdio>
#include <cstdlib>

using ir_toggle::Code;
using ir_toggle::Input;
using ir_toggle::Outcome;
using ir_toggle::ToggleState;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
      std::exit(1); \
    } \
  } while (0)

namespace {

const Code POWER{0x817E, 0xD52A};
const Code MUTE{0x857A, 0xE31C};
const Code SCENE{0x857A, 0x7F00};

void turn_on_when_already_on_sends_nothing() {
  ToggleState s;
  s.restore(true, false);
  CHECK(!s.request_power(true));
  CHECK(s.on());
}

void turn_on_when_off_sends_toggle_and_updates() {
  ToggleState s;
  CHECK(s.request_power(true));
  CHECK(s.on());
  CHECK(s.request_power(false));
  CHECK(!s.on());
}

void mute_set_semantics() {
  ToggleState s;
  CHECK(!s.request_mute(false));
  CHECK(s.request_mute(true));
  CHECK(s.muted());
  CHECK(!s.request_mute(true));
}

void echo_inside_window_is_ignored() {
  ToggleState s;
  s.request_power(true);
  s.note_transmitted(POWER, 1000);
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 1100) == Outcome::IGNORED);
  CHECK(s.on());
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 1999) == Outcome::IGNORED);
  CHECK(s.on());
}

void same_code_after_window_toggles() {
  ToggleState s;
  s.request_power(true);
  s.note_transmitted(POWER, 1000);
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 2000) == Outcome::POWER_CHANGED);
  CHECK(!s.on());
}

void echo_window_is_configurable() {
  ToggleState s;
  s.set_echo_window(100);
  s.request_power(true);
  s.note_transmitted(POWER, 1000);
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 1100) == Outcome::POWER_CHANGED);
}

void echo_survives_clock_wraparound() {
  ToggleState s;
  s.note_transmitted(POWER, 0xFFFFFF00u);
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 0x100u) == Outcome::IGNORED);
}

void other_codes_are_not_echoes() {
  ToggleState s;
  s.note_transmitted(MUTE, 1000);
  CHECK(s.receive(Input::POWER_TOGGLE, POWER, 1100) == Outcome::POWER_CHANGED);
  CHECK(s.on());
}

void remote_mute_while_off_is_ignored() {
  ToggleState s;
  CHECK(s.receive(Input::MUTE_TOGGLE, MUTE, 5000) == Outcome::IGNORED);
  CHECK(!s.muted());
}

void remote_mute_while_on_toggles() {
  ToggleState s;
  s.restore(true, false);
  CHECK(s.receive(Input::MUTE_TOGGLE, MUTE, 5000) == Outcome::MUTE_CHANGED);
  CHECK(s.muted());
  CHECK(s.receive(Input::MUTE_TOGGLE, MUTE, 6000) == Outcome::MUTE_CHANGED);
  CHECK(!s.muted());
}

void remote_scene_turns_on() {
  ToggleState s;
  CHECK(s.receive(Input::SCENE, SCENE, 5000) == Outcome::SCENE_SELECTED);
  CHECK(s.on());
}

void scene_request_turns_on_without_toggle() {
  ToggleState s;
  s.request_scene();
  CHECK(s.on());
  CHECK(!s.request_power(true));
}

void echo_of_scene_is_ignored() {
  ToggleState s;
  s.request_scene();
  s.note_transmitted(SCENE, 1000);
  CHECK(s.receive(Input::SCENE, SCENE, 1200) == Outcome::IGNORED);
}

void force_changes_state_only() {
  ToggleState s;
  s.force_power(true);
  s.force_mute(true);
  CHECK(s.on());
  CHECK(s.muted());
}

}  // namespace

int main() {
  turn_on_when_already_on_sends_nothing();
  turn_on_when_off_sends_toggle_and_updates();
  mute_set_semantics();
  echo_inside_window_is_ignored();
  same_code_after_window_toggles();
  echo_window_is_configurable();
  echo_survives_clock_wraparound();
  other_codes_are_not_echoes();
  remote_mute_while_off_is_ignored();
  remote_mute_while_on_toggles();
  remote_scene_turns_on();
  scene_request_turns_on_without_toggle();
  echo_of_scene_is_ignored();
  force_changes_state_only();
  std::puts("ok");
  return 0;
}
