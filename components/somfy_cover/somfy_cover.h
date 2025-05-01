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
      void set_cc1101(elechouse_cc1101::ElechouseCc1101 *cc1101) { this->cc1101_ = cc1101; }
      void set_cover_id(const char *cover_id) { this->cover_id_ = cover_id; }
      void set_remote_code(uint32_t remote_code) { this->remote_code_ = remote_code; }

    protected:
      SomfyCoverPrivate *somfy_cover_private_;

      elechouse_cc1101::ElechouseCc1101 *cc1101_;
      const char *cover_id_;
      uint32_t remote_code_;
    };
  }
}
