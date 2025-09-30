#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"

#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome
{
    namespace measured_endstop_cover
    {
        enum State
        {
            STATE_IDLE,
            STATE_STARTING,
            STATE_MOVING,
            STATE_STOPPING,
        };

        enum PendingCommand
        {
            PENDING_COMMAND_NONE,
            PENDING_COMMAND_MOVE,
            PENDING_COMMAND_STOP,
        };

        class MeasuredEndstopCover : public cover::Cover, public Component
        {
        public:
            void setup() override;
            void dump_config() override;
            cover::CoverTraits get_traits() override;
            void loop() override;

            Trigger<> *get_open_trigger() const { return this->open_trigger_; }
            Trigger<> *get_close_trigger() const { return this->close_trigger_; }
            Trigger<> *get_stop_trigger() const { return this->stop_trigger_; }

            void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
            void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }
            void set_moving_change_timeout(uint32_t moving_change_timeout) { this->moving_change_timeout_ = moving_change_timeout; }
            void set_moving_covers_sensor(sensor::Sensor *moving_covers) { this->moving_covers_ = moving_covers; }

        protected:
            void control(const cover::CoverCall &call) override;
            void enter_state(State new_state);

            Trigger<> *open_trigger_{new Trigger<>()};
            Trigger<> *close_trigger_{new Trigger<>()};
            Trigger<> *stop_trigger_{new Trigger<>()};
            uint32_t open_duration_{0};
            uint32_t close_duration_{0};
            uint32_t moving_change_timeout_{0};
            sensor::Sensor *moving_covers_{nullptr};

            State state_{STATE_IDLE};
            PendingCommand pending_command_{PENDING_COMMAND_NONE};
            float target_position_{0.0f};
            float moving_covers_snapshot_{0.0f};
            uint32_t last_publish_time_{0};
            uint32_t last_position_compute_time_{0};
            uint32_t movement_start_time_{0};
            uint32_t update_interval_{1000};
        };
    }
}
