#include "watchdog.h"

#ifdef USE_ESP32

#include <cinttypes>

#include "esp_timer.h"

#include "esphome/core/log.h"

#include "zeromouse.h"

namespace esphome::zeromouse {

static const char *const TAG = "zeromouse.watchdog";

static constexpr uint32_t CHECK_INTERVAL_MS = 500;
static constexpr int64_t IDLE_ON_LIMIT_US = 2000000;

void ZeroMouseWatchdog::setup() {
  this->set_interval("check", CHECK_INTERVAL_MS, [this]() { this->check_(); });
}

void ZeroMouseWatchdog::dump_config() { ESP_LOGCONFIG(TAG, "Watchdog:\n  Interval: %" PRIu32 " ms", CHECK_INTERVAL_MS); }

void ZeroMouseWatchdog::check_() {
  const int64_t now = esp_timer_get_time();
  const bool running = this->controller_->is_event_running() && !this->controller_->is_suspended();

  const int64_t deadline_us = static_cast<int64_t>(this->controller_->event_deadline_ms()) * 1000;

  if (running && now - this->controller_->event_start_us() > deadline_us) {
    this->release_("event deadline", true);
    return;
  }

  if (this->blocker_ == nullptr || !this->blocker_->state) {
    this->on_since_us_ = 0;
    this->idle_on_since_us_ = 0;
    return;
  }
  if (this->on_since_us_ == 0) {
    this->on_since_us_ = now;
  }
  if (running) {
    this->idle_on_since_us_ = 0;
  } else if (this->idle_on_since_us_ == 0) {
    this->idle_on_since_us_ = now;
  }

  if (this->idle_on_since_us_ != 0 && now - this->idle_on_since_us_ > IDLE_ON_LIMIT_US) {
    this->release_("blocker on without an event", false);
  } else if (now - this->on_since_us_ > deadline_us && !this->controller_->prey_hold_allowed()) {
    this->release_("blocker on for longer than one event", false);
  }
}

void ZeroMouseWatchdog::release_(const char *rule, bool event_stuck) {
  ESP_LOGW(TAG, "Releasing the blocker: %s", rule);
  if (this->blocker_ != nullptr && this->blocker_->state) {
    this->blocker_->turn_off();
  }
  this->on_since_us_ = 0;
  this->idle_on_since_us_ = 0;
  this->controller_->on_watchdog_release(event_stuck);
  this->trigger_.trigger(rule);
}

}  // namespace esphome::zeromouse

#endif
