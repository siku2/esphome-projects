#include "elechouse_cc1101.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>

namespace siku2
{
    namespace elechouse_cc1101
    {
        void ElechouseCc1101::setup()
        {
            ELECHOUSE_cc1101.setGDO(this->tx_pin_->get_pin(), this->rx_pin_->get_pin());
            ELECHOUSE_cc1101.Init();
            if (this->frequency_ > 0)
            {
                // Frequency is in Hz
                ELECHOUSE_cc1101.setMHZ(this->frequency_ / 1000000);
            }
        }
    }
}
