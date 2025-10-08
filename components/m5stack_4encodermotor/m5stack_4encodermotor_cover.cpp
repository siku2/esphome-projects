#include "m5stack_4encodermotor_cover.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#define COVER_ALMOST_OPEN 0.99f
#define COVER_ALMOST_CLOSED 0.01f

namespace esphome {
namespace m5stack_4encodermotor {
static const char *const TAG = "m5stack_4encodermotor.cover";

using namespace esphome::cover;

void M5Stack4EncoderMotorCover::setup() {
  auto restore = this->restore_state_();

  if (restore.has_value()) {
    restore->apply(this);
  } else {
    // if no other information, assume half open
    this->position = 0.5f;
  }
  this->current_operation = COVER_OPERATION_IDLE;
  this->last_recompute_time_ = this->start_dir_time_ = millis();

  this->parent_->set_motor_mode(this->motor_, MODE_NORMAL);
  this->parent_->set_motor_soft_start_stop(this->motor_, this->soft_start_stop_);
  this->parent_->set_motor_pwm_duty(this->motor_, 0);
}

void M5Stack4EncoderMotorCover::dump_config() {
  LOG_COVER("", "M5Stack 4 Encoder Motor Cover", this);
  ESP_LOGCONFIG(TAG, "  Motor: %d", this->motor_ + 1);
  ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Min Current: %.3fA", this->min_current_);
  if (this->soft_start_stop_) {
    ESP_LOGCONFIG(TAG, "  Soft Start Stop: YES");
  }
}

void M5Stack4EncoderMotorCover::loop() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  const uint32_t now = App.get_loop_component_start_time();
  this->recompute_position_(now);

  if (this->is_at_target_()) {
    this->position = this->target_position_;
    this->start_direction_(COVER_OPERATION_IDLE);
  } else if (now - this->last_publish_time_ > this->update_interval_) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

void M5Stack4EncoderMotorCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(COVER_OPERATION_IDLE);
  } else if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos != this->position) {
      this->target_position_ = pos;
      this->start_direction_(pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING);
    }
  }
}

bool M5Stack4EncoderMotorCover::is_at_target_() {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      if (this->target_position_ == COVER_OPEN) {
        return this->position >= COVER_ALMOST_OPEN && this->is_moving_ == false;
      } else {
        return this->position >= this->target_position_;
      }
    case COVER_OPERATION_CLOSING:
      if (this->target_position_ == COVER_CLOSED) {
        return this->position <= COVER_ALMOST_CLOSED && this->is_moving_ == false;
      } else {
        return this->position <= this->target_position_;
      }
    case COVER_OPERATION_IDLE:
      return this->current_operation == COVER_OPERATION_IDLE;
    default:
      return true;
  }
}

void M5Stack4EncoderMotorCover::start_direction_(CoverOperation dir) {
  int8_t mul = this->positive_is_up_ ? 1 : -1;
  switch (dir) {
    case COVER_OPERATION_IDLE:
      this->parent_->set_motor_pwm_duty(this->motor_, 0);
      break;
    case COVER_OPERATION_OPENING:
      this->parent_->set_motor_pwm_duty(this->motor_, mul * 127);
      break;
    case COVER_OPERATION_CLOSING:
      this->parent_->set_motor_pwm_duty(this->motor_, mul * -127);
      break;
    default:
      return;
  }
  this->start_dir_time_ = millis();
  this->current_operation = dir;
  this->publish_state(true);
}

void M5Stack4EncoderMotorCover::recompute_position_(uint32_t now) {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  float dir;
  float action_dur;
  float min_pos;
  float max_pos;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      min_pos = COVER_CLOSED;
      max_pos = this->position < COVER_OPEN ? COVER_ALMOST_OPEN : COVER_OPEN;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      min_pos = this->position > COVER_CLOSED ? COVER_ALMOST_CLOSED : COVER_CLOSED;
      max_pos = COVER_OPEN;
      break;
    default:
      return;
  }

  // check if we have an acceleration_wait_time, and remove from position computation
  if (now > (this->start_dir_time_ + this->acceleration_wait_time_)) {
    this->position +=
        dir * (now - std::max(this->start_dir_time_ + this->acceleration_wait_time_, this->last_recompute_time_)) /
        (action_dur - this->acceleration_wait_time_);
    this->position = clamp(this->position, min_pos, max_pos);
  }
  this->last_recompute_time_ = now;

  this->is_moving_ = this->parent_->get_current() >= this->min_current_;
}
}  // namespace m5stack_4encodermotor
}  // namespace esphome