#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/elechouse_cc1101/elechouse_cc1101.h"

namespace esphome
{
  namespace somfy_cover
  {
    struct SomfyCoverPrivate;

    class SomfyCover : public cover::Cover, public Component
    {
    public:
      void setup() override;
      void loop() override;
      cover::CoverTraits get_traits() override;
      cover::CoverOperation get_last_operation() const { return this->last_operation_; }

      void set_cc1101(elechouse_cc1101::ElechouseCc1101 *cc1101) { this->cc1101_ = cc1101; }
      void set_cover_id(const char *cover_id) { this->cover_id_ = cover_id; }
      void set_remote_code(uint32_t remote_code) { this->remote_code_ = remote_code; }
      void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
      void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }

    protected:
      void control(const cover::CoverCall &call) override;

      bool is_at_target_() const;

      void start_direction_(cover::CoverOperation dir);
      void recompute_position_();

      SomfyCoverPrivate *priv_;

      elechouse_cc1101::ElechouseCc1101 *cc1101_;
      const char *cover_id_;
      uint32_t remote_code_;
      uint32_t open_duration_;
      uint32_t close_duration_;

      uint32_t last_recompute_time_{0};
      uint32_t start_dir_time_{0};
      uint32_t last_publish_time_{0};
      float target_position_{0};
      cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
    };
  }
}
