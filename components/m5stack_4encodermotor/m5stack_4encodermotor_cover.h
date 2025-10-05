#pragma once

#include "m5stack_4encodermotor.h"

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"

namespace esphome
{
    namespace m5stack_4encodermotor
    {
        class M5Stack4EncoderMotorCover : public cover::Cover, public Component
        {
        public:
            void setup() override;
            void dump_config() override;
            void loop() override;

            cover::CoverTraits get_traits() override
            {
                auto traits = cover::CoverTraits();
                traits.set_is_assumed_state(true);
                traits.set_supports_position(true);
                traits.set_supports_stop(true);
                return traits;
            }

            void set_parent(M5Stack4EncoderMotor *parent) { this->parent_ = parent; }
            void set_motor(Motor motor) { this->motor_ = motor; }
            void set_min_current(float min_current) { this->min_current_ = min_current; }
            void set_soft_start_stop(bool enable) { this->soft_start_stop_ = enable; }
            void set_positive_is_up(bool positive_is_up) { this->positive_is_up_ = positive_is_up; }

            void set_open_duration(uint32_t duration) { this->open_duration_ = duration; }
            void set_close_duration(uint32_t duration) { this->close_duration_ = duration; }
            void set_acceleration_wait_time(uint32_t duration) { this->acceleration_wait_time_ = duration; }

        protected:
            M5Stack4EncoderMotor *parent_;
            Motor motor_;
            float min_current_;
            bool soft_start_stop_;
            bool positive_is_up_;
            uint32_t open_duration_;
            uint32_t close_duration_;
            uint32_t acceleration_wait_time_;

            void control(const cover::CoverCall &call) override;

            uint32_t last_recompute_time_{0};
            bool is_moving_{false};
            uint32_t start_dir_time_{0};
            uint32_t last_publish_time_{0};
            float target_position_{0};
            uint32_t update_interval_{1000};

            bool is_at_target_();
            void start_direction_(cover::CoverOperation dir);
            void recompute_position_(uint32_t now);
        };
    }
}
