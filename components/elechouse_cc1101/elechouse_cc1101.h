#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace siku2
{
    namespace elechouse_cc1101
    {
        class ElechouseCc1101 : public esphome::Component
        {
        public:
            void set_tx_pin(esphome::InternalGPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }
            void set_rx_pin(esphome::InternalGPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }
            void set_frequency(float frequency) { this->frequency_ = frequency; }

            void setup() override;

        protected:
            esphome::InternalGPIOPin *tx_pin_;
            esphome::InternalGPIOPin *rx_pin_;
            float frequency_{};
        };
    }
}
