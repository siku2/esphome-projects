#include "measured_endstop_cover.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace measured_endstop_cover
    {
        static const char *const TAG = "measured_endstop_cover";

        using namespace esphome::cover;

        void MeasuredEndstopCover::setup()
        {
            auto restore = this->restore_state_();
            if (restore.has_value())
            {
                restore->apply(this);
            }
            else
            {
                this->position = 0.5f;
            }

            this->current_operation = COVER_OPERATION_IDLE;
        }

        void MeasuredEndstopCover::dump_config()
        {
            LOG_COVER("", "Measured Endstop Cover", this);
            ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
            ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
            LOG_SENSOR("  ", "Moving Covers", this->moving_covers_);
        }

        CoverTraits MeasuredEndstopCover::get_traits()
        {
            auto traits = CoverTraits();
            traits.set_supports_stop(true);
            traits.set_supports_position(true);
            traits.set_is_assumed_state(true);
            return traits;
        }

        void MeasuredEndstopCover::loop()
        {
            const uint32_t now = millis();

            switch (this->state_)
            {
            // Nothing is currently happening.
            case STATE_IDLE:
                switch (this->pending_command_)
                {
                case PENDING_COMMAND_MOVE:
                    this->enter_state(STATE_STARTING);
                    break;
                case PENDING_COMMAND_STOP:
                    // We are already stopped, but let's trigger the stop event anyway
                    this->enter_state(STATE_IDLE);
                    break;
                }
                // We accepted the command.
                this->pending_command_ = PENDING_COMMAND_NONE;
                break;

            // We are starting to move, wait for the moving_covers sensor to change.
            case STATE_STARTING:
                if (this->moving_covers_->state > this->moving_covers_snapshot_)
                {
                    // We are now moving.
                    this->enter_state(STATE_MOVING);
                }
                else if ((now - this->movement_start_time_) > this->moving_change_timeout_)
                {
                    ESP_LOGW(TAG, "Timeout waiting for moving_covers sensor to change.");
                    this->enter_state(STATE_IDLE);
                }
                break;

            case STATE_MOVING:
                // Update position.
                if (this->movement_start_time_ > this->last_position_compute_time_)
                {
                    this->last_position_compute_time_ = this->movement_start_time_;
                }
                switch (this->current_operation)
                {
                case COVER_OPERATION_OPENING:
                    this->position += (now - this->last_position_compute_time_) / this->open_duration_;
                    break;
                case COVER_OPERATION_CLOSING:
                    this->position -= (now - this->last_position_compute_time_) / this->close_duration_;
                    break;
                }
                this->position = std::clamp(this->position, 0.0f, 1.0f);
                this->last_position_compute_time_ = now;

                if (now - this->last_publish_time_ > this->update_interval_)
                {
                    this->publish_state(false);
                    this->last_publish_time_ = now;
                }

                // Check if we have a new command.
                switch (this->pending_command_)
                {
                // We want to stop
                case PENDING_COMMAND_STOP:
                    this->enter_state(STATE_STOPPING);
                    this->pending_command_ = PENDING_COMMAND_NONE;
                    break;
                // We want to move somewhere else
                case PENDING_COMMAND_MOVE:
                    if (
                        // We are moving up, but want to go down now.
                        (this->current_operation == COVER_OPERATION_OPENING && this->target_position_ < this->position)
                        // We are moving down, but want to go up now.
                        || (this->current_operation == COVER_OPERATION_CLOSING && this->target_position_ > this->position))
                    {
                        this->enter_state(STATE_STOPPING);
                        // Don't clear the pending command, we will pick it up when we are stopped.
                    }
                    break;
                }
                // State changed, don't continue processing.
                if (this->state_ != STATE_MOVING)
                    break;

                if (this->moving_covers_snapshot_ > this->moving_covers_->state)
                {
                    // Another cover started moving, update the snapshot.
                    this->moving_covers_snapshot_ = this->moving_covers_->state;
                }
                else if (
                    // Number of covers decreased.
                    (this->moving_covers_snapshot_ < this->moving_covers_->state)
                    // Or no cover is moving (sensor reset to 0).
                    || (this->moving_covers_->state == 0))
                {
                    this->moving_covers_snapshot_ = this->moving_covers_->state;
                    // A cover stopped moving. Check if it's reasonable to assume it's us.
                    if (
                        // No cover is moving now, so it must be us.
                        (this->moving_covers_snapshot_ == 0)
                        // We are close to the target position, assume we have reached it.
                        || (std::abs(this->target_position_ - this->position) < 0.05f))
                    {
                        ESP_LOGI(TAG, "Cover stopped moving, assuming we have reached the target position.");
                        this->position = this->target_position_;
                        // Go directly to idle since we're not moving anymore.
                        this->enter_state(STATE_IDLE);
                    }
                }

                // Check if we have reached the target position.
                if (
                    // Only check if the target position is not fully open or fully closed.
                    !(this->target_position_ == 1.0f || this->target_position_ == 0.0f)
                    //
                    && (
                           // We are opening and have reached or passed the target position.
                           (this->current_operation == COVER_OPERATION_OPENING && this->position >= this->target_position_)
                           // We are closing and have reached or passed the target position.
                           || (this->current_operation == COVER_OPERATION_CLOSING && this->position <= this->target_position_)))
                {
                    ESP_LOGI(TAG, "Reached target position.");
                    this->enter_state(STATE_STOPPING);
                }

                break;

            case STATE_STOPPING:
                if (this->moving_covers_->state < this->moving_covers_snapshot_)
                {
                    // We have stopped moving.
                    this->enter_state(STATE_IDLE);
                }
                else if ((now - this->movement_start_time_) > this->moving_change_timeout_)
                {
                    ESP_LOGW(TAG, "Timeout waiting for moving_covers sensor to change.");
                    this->enter_state(STATE_IDLE);
                }

                break;
            }
        }

        void MeasuredEndstopCover::control(const cover::CoverCall &call)
        {
            if (call.get_stop())
            {
                pending_command_ = PENDING_COMMAND_STOP;
            }
            else if (call.get_position().has_value())
            {
                auto pos = *call.get_position();
                this->target_position_ = pos;
                pending_command_ = PENDING_COMMAND_MOVE;
            }
        }

        void MeasuredEndstopCover::enter_state(State new_state)
        {
            switch (new_state)
            {
            // The following variables should be set:
            // - target_position_
            case STATE_STARTING:
                if (this->target_position_ == 1.0 || this->target_position_ > this->position)
                {
                    // Move up
                    this->open_trigger_->trigger();
                    this->current_operation = COVER_OPERATION_OPENING;
                }
                else if (this->target_position_ == 0.0 || this->target_position_ < this->position)
                {
                    // Move down
                    this->close_trigger_->trigger();
                    this->current_operation = COVER_OPERATION_CLOSING;
                }

                this->movement_start_time_ = millis();
                this->last_position_compute_time_ = 0;
                this->publish_state();
                this->last_publish_time_ = this->movement_start_time_;
                break;

            case STATE_MOVING:
                this->movement_start_time_ = millis();
                break;

            case STATE_STOPPING:
                this->stop_trigger_->trigger();
                break;

            case STATE_IDLE:
                this->stop_trigger_->trigger();
                this->current_operation = COVER_OPERATION_IDLE;
                this->publish_state();
                this->last_publish_time_ = millis();
                break;
            }

            ESP_LOGI(TAG, "State change: %d -> %d", this->state_, new_state);
            this->state_ = new_state;
            this->moving_covers_snapshot_ = this->moving_covers_->state;
        }
    }
}
