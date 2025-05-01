#include "somfy_cover.h"
#include <NVSRollingCodeStorage.h>
#include <SomfyRemote.h>

namespace esphome
{
  namespace somfy_cover
  {
    const char *NVS_NAMESPACE = "somfy";

    struct SomfyCoverPrivate
    {
      NVSRollingCodeStorage *rolling_code_storage_;
      SomfyRemote *somfy_remote_;

      void send_command(elechouse_cc1101::ElechouseCc1101 *cc1101, Command command)
      {
        cc1101->set_tx();
        this->somfy_remote_->sendCommand(command);
        cc1101->set_sidle();
      }
    };

    void
    SomfyCover::setup()
    {
      SomfyCoverPrivate *priv = new SomfyCoverPrivate();
      priv->rolling_code_storage_ = new NVSRollingCodeStorage(NVS_NAMESPACE, this->cover_id_);
      priv->somfy_remote_ = new SomfyRemote(this->cc1101_->get_emitter_pin(), this->remote_code_, priv->rolling_code_storage_);
      this->somfy_cover_private_ = priv;
    }
  }
}
