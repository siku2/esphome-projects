#include "somfy_cover.h"
#include "esphome/core/log.h"
#include <NVSRollingCodeStorage.h>
#include <SomfyRemote.h>

namespace esphome
{
  namespace somfy_cover
  {
    static const char *NVS_NAMESPACE = "somfy";
    static const char *const TAG = "somfy_cover";

    using namespace esphome::cover;
    using namespace esphome::elechouse_cc1101;

    struct SomfyCoverPrivate
    {
      NVSRollingCodeStorage *rolling_code_storage_{};
      SomfyRemote *somfy_remote_{};

      SomfyCoverPrivate(const char *cover_id, uint8_t emitter_pin, uint32_t remote_code)
      {
        this->rolling_code_storage_ = new NVSRollingCodeStorage(NVS_NAMESPACE, cover_id);
        this->somfy_remote_ = new SomfyRemote(emitter_pin, remote_code, this->rolling_code_storage_);
      }

      ~SomfyCoverPrivate()
      {
        if (this->rolling_code_storage_)
          delete this->rolling_code_storage_;
        if (this->somfy_remote_)
          delete this->somfy_remote_;
      }

      void send_command(ElechouseCc1101 *cc1101, Command command)
      {
        cc1101->enable_tx();
        this->somfy_remote_->sendCommand(command);
        cc1101->enable_sidle();
      }
    };

    void SomfyCover::dump_config()
    {
      LOG_COVER("", "Somfy Cover", this);
      ESP_LOGCONFIG(TAG, "  CC1101: %p", static_cast<void *>(this->cc1101_));
      ESP_LOGCONFIG(TAG, "  Cover ID: %s", this->cover_id_);
      ESP_LOGCONFIG(TAG, "  Remote Code: 0x%08X", this->remote_code_);
      ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
      ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
    }

    void
    SomfyCover::setup()
    {
      if (!this->cc1101_ || !this->cover_id_)
      {
        ESP_LOGE(TAG, "cc1101_ or cover_id_ not set");
        this->mark_failed();
        return;
      }

      this->priv_ = new SomfyCoverPrivate(this->cover_id_, this->cc1101_->get_emitter_pin(), this->remote_code_);
      if (!this->priv_)
      {
        ESP_LOGE(TAG, "Failed to create SomfyCoverPrivate");
        this->mark_failed();
        return;
      }

      auto restore = this->restore_state_();
      if (restore.has_value())
      {
        restore->apply(this);
      }
      else
      {
        this->position = 0.5f;
      }
    }

    void SomfyCover::loop()
    {
      if (this->current_operation == COVER_OPERATION_IDLE)
        return;

      const uint32_t now = millis();

      // Recompute position every loop cycle
      this->recompute_position_();

      if (this->is_at_target_())
      {
        if (this->target_position_ == COVER_OPEN || this->target_position_ == COVER_CLOSED)
        {
          // Don't trigger stop, let the cover stop by itself.
          this->current_operation = COVER_OPERATION_IDLE;
        }
        else
        {
          this->start_direction_(COVER_OPERATION_IDLE);
        }
        this->publish_state();
      }

      // Send current position every second
      if (now - this->last_publish_time_ > 1000)
      {
        this->publish_state(false);
        this->last_publish_time_ = now;
      }
    }

    CoverTraits SomfyCover::get_traits()
    {
      auto traits = CoverTraits();
      traits.set_supports_stop(true);
      traits.set_supports_position(true);
      traits.set_supports_toggle(true);
      traits.set_is_assumed_state(true);
      return traits;
    }

    void SomfyCover::control(const CoverCall &call)
    {
      if (call.get_stop())
      {
        this->start_direction_(COVER_OPERATION_IDLE);
        this->publish_state();
      }
      if (call.get_toggle().has_value())
      {
        if (this->current_operation != COVER_OPERATION_IDLE)
        {
          this->start_direction_(COVER_OPERATION_IDLE);
          this->publish_state();
        }
        else
        {
          if (this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING)
          {
            this->target_position_ = COVER_OPEN;
            this->start_direction_(COVER_OPERATION_OPENING);
          }
          else
          {
            this->target_position_ = COVER_CLOSED;
            this->start_direction_(COVER_OPERATION_CLOSING);
          }
        }
      }
      if (call.get_position().has_value())
      {
        auto pos = *call.get_position();
        if (pos == this->position)
        {
          // already at target
          // for covers with built in end stop, we should send the command again
          if (pos == COVER_OPEN || pos == COVER_CLOSED)
          {
            auto op = pos == COVER_CLOSED ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
            this->target_position_ = pos;
            this->start_direction_(op);
          }
        }
        else
        {
          auto op = pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
          this->target_position_ = pos;
          this->start_direction_(op);
        }
      }
    }

    bool SomfyCover::is_at_target_() const
    {
      switch (this->current_operation)
      {
      case COVER_OPERATION_OPENING:
        return this->position >= this->target_position_;
      case COVER_OPERATION_CLOSING:
        return this->position <= this->target_position_;
      case COVER_OPERATION_IDLE:
      default:
        return true;
      }
    }

    void SomfyCover::start_direction_(CoverOperation dir)
    {
      if (dir == this->current_operation && dir != COVER_OPERATION_IDLE)
        return;

      this->recompute_position_();
      switch (dir)
      {
      case COVER_OPERATION_IDLE:
        this->priv_->send_command(this->cc1101_, Command::My);
        break;
      case COVER_OPERATION_OPENING:
        this->last_operation_ = dir;
        this->priv_->send_command(this->cc1101_, Command::Up);
        break;
      case COVER_OPERATION_CLOSING:
        this->last_operation_ = dir;
        this->priv_->send_command(this->cc1101_, Command::Down);
        break;
      default:
        return;
      }

      this->current_operation = dir;

      const uint32_t now = millis();
      this->start_dir_time_ = now;
      this->last_recompute_time_ = now;
    }

    void SomfyCover::recompute_position_()
    {
      if (this->current_operation == COVER_OPERATION_IDLE)
        return;

      float dir;
      float action_dur;
      switch (this->current_operation)
      {
      case COVER_OPERATION_OPENING:
        dir = 1.0f;
        action_dur = this->open_duration_;
        break;
      case COVER_OPERATION_CLOSING:
        dir = -1.0f;
        action_dur = this->close_duration_;
        break;
      default:
        return;
      }

      const uint32_t now = millis();
      this->position += dir * (now - this->last_recompute_time_) / action_dur;
      this->position = clamp(this->position, 0.0f, 1.0f);

      this->last_recompute_time_ = now;
    }

    void SomfyCover::program()
    {
      this->priv_->send_command(this->cc1101_, Command::Prog);
    }
  }
}
