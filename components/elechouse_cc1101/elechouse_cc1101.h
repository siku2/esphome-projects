#pragma once

#include "esphome/core/component.h"

namespace siku2
{
    namespace elechouse_cc1101
    {
        class ElechouseCc1101 : public Component
        {
        public:
            void set_tx_pin(InternalGPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }
            void set_rx_pin(InternalGPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }
            void set_frequency(float frequency) { this->frequency_ = frequency; }

            void setup() override;

        protected:
            InternalGPIOPin *tx_pin_;
            InternalGPIOPin *rx_pin_;
            float frequency_{};
        };
    }
}
