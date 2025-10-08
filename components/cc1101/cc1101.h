#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace cc1101 {
class Cc1101 : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_5MHZ> {
 public:
  void setup() override;
  void dump_config() override;

  void enable_and_wait();

  void enable_tx();
  void enable_sidle();

  GPIOPin *get_emitter_pin() const { return this->tx_pin_; }

  void set_miso_pin(GPIOPin *miso_pin) { this->miso_pin_ = miso_pin; }
  void set_tx_pin(GPIOPin *tx_pin) { this->tx_pin_ = tx_pin; }
  void set_rx_pin(GPIOPin *rx_pin) { this->rx_pin_ = rx_pin; }
  void set_channel(uint8_t chan) { this->chan_ = chan; }
  void set_cc_mode(bool cc_mode) { this->cc_mode_ = cc_mode; }
  void set_frequency(float frequency) { this->frequency_ = frequency; }

 protected:
  GPIOPin *miso_pin_{};
  GPIOPin *tx_pin_{};
  GPIOPin *rx_pin_{};
  uint8_t chan_{};
  bool cc_mode_{};
  float frequency_{};

  uint8_t modulation_{2};
  uint8_t m4_rx_bw_{0};
  uint8_t clb1_[2] = {24, 28};
  uint8_t clb2_[2] = {31, 38};
  uint8_t clb3_[2] = {65, 76};
  uint8_t clb4_[2] = {77, 79};
  uint8_t last_pa_{0};
  uint8_t pa_{12};
  uint8_t m2_dc_off_{0};
  uint8_t m2_mod_fm_{0};
  uint8_t m2_man_ch_{0};
  uint8_t m2_sync_m_{0};

  void reset_();
  void calibrate_();
  void command_strobe_(uint8_t strobe);

  uint8_t read_reg_(uint8_t addr);
  void write_reg_(uint8_t addr, uint8_t value);
  void write_burst_reg_(uint8_t addr, uint8_t *data, size_t length);

  void read_mdmcfg2_();

  void write_config_();
  void write_frequency_();
  void write_cc_mode_();
  void write_modulation_();
  void write_pa_();
};
}  // namespace cc1101
}  // namespace esphome
