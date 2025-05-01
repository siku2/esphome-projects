#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome
{
  namespace elechouse_cc1101
  {
    class ElechouseCc1101 : public Component
    {
    public:
      void set_tx();
      void set_sidle();

      uint8_t get_emitter_pin() const { return this->tx_pin_->get_pin(); }

      void setup() override;
      void set_tx_pin(InternalGPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }
      void set_rx_pin(InternalGPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }
      void set_frequency(float frequency) { this->frequency_ = frequency; }

    protected:
      InternalGPIOPin *tx_pin_;
      InternalGPIOPin *rx_pin_;
      float frequency_{};
    };
  }
}
