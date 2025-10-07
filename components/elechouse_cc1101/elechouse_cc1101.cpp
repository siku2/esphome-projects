#include "elechouse_cc1101.h"

#include "esphome/core/log.h"

//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGSITER
#define CC1101_IOCFG2 0x00   // GDO2 output pin configuration
#define CC1101_IOCFG1 0x01   // GDO1 output pin configuration
#define CC1101_IOCFG0 0x02   // GDO0 output pin configuration
#define CC1101_FIFOTHR 0x03  // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1 0x04    // Sync word, high INT8U
#define CC1101_SYNC0 0x05    // Sync word, low INT8U
#define CC1101_PKTLEN 0x06   // Packet length
#define CC1101_PKTCTRL1 0x07 // Packet automation control
#define CC1101_PKTCTRL0 0x08 // Packet automation control
#define CC1101_ADDR 0x09     // Device address
#define CC1101_CHANNR 0x0A   // Channel number
#define CC1101_FSCTRL1 0x0B  // Frequency synthesizer control
#define CC1101_FSCTRL0 0x0C  // Frequency synthesizer control
#define CC1101_FREQ2 0x0D    // Frequency control word, high INT8U
#define CC1101_FREQ1 0x0E    // Frequency control word, middle INT8U
#define CC1101_FREQ0 0x0F    // Frequency control word, low INT8U
#define CC1101_MDMCFG4 0x10  // Modem configuration
#define CC1101_MDMCFG3 0x11  // Modem configuration
#define CC1101_MDMCFG2 0x12  // Modem configuration
#define CC1101_MDMCFG1 0x13  // Modem configuration
#define CC1101_MDMCFG0 0x14  // Modem configuration
#define CC1101_DEVIATN 0x15  // Modem deviation setting
#define CC1101_MCSM2 0x16    // Main Radio Control State Machine configuration
#define CC1101_MCSM1 0x17    // Main Radio Control State Machine configuration
#define CC1101_MCSM0 0x18    // Main Radio Control State Machine configuration
#define CC1101_FOCCFG 0x19   // Frequency Offset Compensation configuration
#define CC1101_BSCFG 0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2 0x1B // AGC control
#define CC1101_AGCCTRL1 0x1C // AGC control
#define CC1101_AGCCTRL0 0x1D // AGC control
#define CC1101_WOREVT1 0x1E  // High INT8U Event 0 timeout
#define CC1101_WOREVT0 0x1F  // Low INT8U Event 0 timeout
#define CC1101_WORCTRL 0x20  // Wake On Radio control
#define CC1101_FREND1 0x21   // Front end RX configuration
#define CC1101_FREND0 0x22   // Front end TX configuration
#define CC1101_FSCAL3 0x23   // Frequency synthesizer calibration
#define CC1101_FSCAL2 0x24   // Frequency synthesizer calibration
#define CC1101_FSCAL1 0x25   // Frequency synthesizer calibration
#define CC1101_FSCAL0 0x26   // Frequency synthesizer calibration
#define CC1101_RCCTRL1 0x27  // RC oscillator configuration
#define CC1101_RCCTRL0 0x28  // RC oscillator configuration
#define CC1101_FSTEST 0x29   // Frequency synthesizer calibration control
#define CC1101_PTEST 0x2A    // Production test
#define CC1101_AGCTEST 0x2B  // AGC test
#define CC1101_TEST2 0x2C    // Various test settings
#define CC1101_TEST1 0x2D    // Various test settings
#define CC1101_TEST0 0x2E    // Various test settings

// CC1101 Strobe commands
#define CC1101_SRES 0x30    // Reset chip.
#define CC1101_SFSTXON 0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                            // If in RX/TX: Go to a wait state where only the synthesizer is
                            // running (for quick RX / TX turnaround).
#define CC1101_SXOFF 0x32   // Turn off crystal oscillator.
#define CC1101_SCAL 0x33    // Calibrate frequency synthesizer and turn it off
                            // (enables quick start).
#define CC1101_SRX 0x34     // Enable RX. Perform calibration first if coming from IDLE and
                            // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX 0x35     // In IDLE state: Enable TX. Perform calibration first if
                            // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                            // Only go to TX if channel is clear.
#define CC1101_SIDLE 0x36   // Exit RX / TX, turn off frequency synthesizer and exit
                            // Wake-On-Radio mode if applicable.
#define CC1101_SAFC 0x37    // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR 0x38    // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD 0x39    // Enter power down mode when CSn goes high.
#define CC1101_SFRX 0x3A    // Flush the RX FIFO buffer.
#define CC1101_SFTX 0x3B    // Flush the TX FIFO buffer.
#define CC1101_SWORRST 0x3C // Reset real time clock.
#define CC1101_SNOP 0x3D    // No operation. May be used to pad strobe commands to two
                            // INT8Us for simpler software.
// CC1101 STATUS REGSITER
#define CC1101_PARTNUM 0x30
#define CC1101_VERSION 0x31
#define CC1101_FREQEST 0x32
#define CC1101_LQI 0x33
#define CC1101_RSSI 0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_WORTIME1 0x36
#define CC1101_WORTIME0 0x37
#define CC1101_PKTSTATUS 0x38
#define CC1101_VCO_VC_DAC 0x39
#define CC1101_TXBYTES 0x3A
#define CC1101_RXBYTES 0x3B

// CC1101 PATABLE,TXFIFO,RXFIFO
#define CC1101_PATABLE 0x3E
#define CC1101_TXFIFO 0x3F
#define CC1101_RXFIFO 0x3F

#define WRITE_BURST 0x40     // write burst
#define READ_SINGLE 0x80     // read single
#define READ_BURST 0xC0      // read burst
#define BYTES_IN_RXFIFO 0x7F // byte number in RXfifo

namespace esphome
{
  namespace elechouse_cc1101
  {
    static const char *const TAG = "elechouse_cc1101";

    static uint8_t PA_TABLE[8]{0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //                       -30  -20  -15  -10   0    5    7    10
    static uint8_t PA_TABLE_315[8]{
        0x12,
        0x0D,
        0x1C,
        0x34,
        0x51,
        0x85,
        0xCB,
        0xC2,
    }; // 300 - 348
    static uint8_t PA_TABLE_433[8]{
        0x12,
        0x0E,
        0x1D,
        0x34,
        0x60,
        0x84,
        0xC8,
        0xC0,
    }; // 387 - 464
    //                        -30  -20  -15  -10  -6    0    5    7    10   12
    static uint8_t PA_TABLE_868[10]{
        0x03,
        0x17,
        0x1D,
        0x26,
        0x37,
        0x50,
        0x86,
        0xCD,
        0xC5,
        0xC0,
    }; // 779 - 899.99
    //                        -30  -20  -15  -10  -6    0    5    7    10   11
    static uint8_t PA_TABLE_915[10]{
        0x03,
        0x0E,
        0x1E,
        0x27,
        0x38,
        0x8E,
        0x84,
        0xCC,
        0xC3,
        0xC0,
    }; // 900 - 928

    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void ElechouseCc1101::setup()
    {
      this->spi_setup();

      // Perform SW reset.
      this->enable();
      delay(1);
      this->cs_->digital_write(true);
      delay(1);
      this->cs_->digital_write(false);
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);
      this->transfer_byte(CC1101_SRES);
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);
      this->cs_->digital_write(true);
      this->disable();

      this->write_config_();
    }

    void ElechouseCc1101::dump_config()
    {
      ESP_LOGCONFIG(TAG, "ELECHOUSE CC1101:");
      ESP_LOGCONFIG(TAG, "  TX Pin: %d", this->tx_pin_->get_pin());
      ESP_LOGCONFIG(TAG, "  RX Pin: %d", this->rx_pin_->get_pin());
      ESP_LOGCONFIG(TAG, "  Frequency: %.1f MHz", this->frequency_ / 1000000);
      LOG_PIN("  CS Pin:", this->cs_);
    }

    void ElechouseCc1101::enable_tx()
    {
      this->command_strobe_(CC1101_SIDLE);
      this->command_strobe_(CC1101_STX); // start send
    }

    void ElechouseCc1101::enable_sidle()
    {
      this->command_strobe_(CC1101_SIDLE);
    }

    void ElechouseCc1101::command_strobe_(uint8_t strobe)
    {
      this->enable();
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);
      this->transfer_byte(strobe);
      this->disable();
    }

    void ElechouseCc1101::write_reg_(uint8_t addr, uint8_t value)
    {
      this->enable();
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);
      this->transfer_byte(addr);
      this->transfer_byte(value);
      this->disable();
    }

    void ElechouseCc1101::write_burst_reg_(uint8_t addr, uint8_t *data, size_t length)
    {
      this->enable();
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);
      this->transfer_byte(addr | WRITE_BURST);
      this->transfer_array(data, length);
      this->disable();
    }

    uint8_t ElechouseCc1101::read_reg_(uint8_t addr)
    {
      this->enable();
      // TODO: We should wait for MISO to go low, but we have no access to it.
      //       Instead, we sleep.
      delay(1);

      this->transfer_byte(addr | READ_BURST);
      uint8_t value = this->read_byte();
      this->disable();
      return value;
    }

    void ElechouseCc1101::read_mdmcfg2_()
    {
      int calc = this->read_reg_(CC1101_MDMCFG2);
      this->m2_dc_off_ = 0;
      this->m2_mod_fm_ = 0;
      this->m2_man_ch_ = 0;
      this->m2_sync_m_ = 0;
      for (bool i = 0; i == 0;)
      {
        if (calc >= 128)
        {
          calc -= 128;
          this->m2_dc_off_ += 128;
        }
        else if (calc >= 16)
        {
          calc -= 16;
          this->m2_mod_fm_ += 16;
        }
        else if (calc >= 8)
        {
          calc -= 8;
          this->m2_man_ch_ += 8;
        }
        else
        {
          this->m2_sync_m_ = calc;
          i = 1;
        }
      }
    }

    void ElechouseCc1101::write_config_()
    {
      this->write_reg_(CC1101_FSCTRL1, 0x06);

      this->write_cc_mode_();
      this->write_frequency_mhz_();

      this->write_reg_(CC1101_MDMCFG1, 0x02);
      this->write_reg_(CC1101_MDMCFG0, 0xF8);
      this->write_reg_(CC1101_CHANNR, this->chan_);
      this->write_reg_(CC1101_DEVIATN, 0x47);
      this->write_reg_(CC1101_FREND1, 0x56);
      this->write_reg_(CC1101_MCSM0, 0x18);
      this->write_reg_(CC1101_FOCCFG, 0x16);
      this->write_reg_(CC1101_BSCFG, 0x1C);
      this->write_reg_(CC1101_AGCCTRL2, 0xC7);
      this->write_reg_(CC1101_AGCCTRL1, 0x00);
      this->write_reg_(CC1101_AGCCTRL0, 0xB2);
      this->write_reg_(CC1101_FSCAL3, 0xE9);
      this->write_reg_(CC1101_FSCAL2, 0x2A);
      this->write_reg_(CC1101_FSCAL1, 0x00);
      this->write_reg_(CC1101_FSCAL0, 0x1F);
      this->write_reg_(CC1101_FSTEST, 0x59);
      this->write_reg_(CC1101_TEST2, 0x81);
      this->write_reg_(CC1101_TEST1, 0x35);
      this->write_reg_(CC1101_TEST0, 0x09);
      this->write_reg_(CC1101_PKTCTRL1, 0x04);
      this->write_reg_(CC1101_ADDR, 0x00);
      this->write_reg_(CC1101_PKTLEN, 0x00);
    }

    void ElechouseCc1101::write_frequency_mhz_()
    {
      float mhz = this->frequency_ / 1000000;
      uint8_t freq2 = 0;
      uint8_t freq1 = 0;
      uint8_t freq0 = 0;
      for (bool i = 0; i == 0;)
      {
        if (mhz >= 26)
        {
          mhz -= 26;
          freq2 += 1;
        }
        else if (mhz >= 0.1015625)
        {
          mhz -= 0.1015625;
          freq1 += 1;
        }
        else if (mhz >= 0.00039675)
        {
          mhz -= 0.00039675;
          freq0 += 1;
        }
        else
        {
          i = 1;
        }
      }
      if (freq0 > 255)
      {
        freq1 += 1;
        freq0 -= 256;
      }

      this->write_reg_(CC1101_FREQ2, freq2);
      this->write_reg_(CC1101_FREQ1, freq1);
      this->write_reg_(CC1101_FREQ0, freq0);

      this->calibrate_();
    }

    void ElechouseCc1101::write_cc_mode_()
    {
      if (this->cc_mode_)
      {
        this->write_reg_(CC1101_IOCFG2, 0x0B);
        this->write_reg_(CC1101_IOCFG0, 0x06);
        this->write_reg_(CC1101_PKTCTRL0, 0x05);
        this->write_reg_(CC1101_MDMCFG3, 0xF8);
        this->write_reg_(CC1101_MDMCFG4, 11 + this->m4_rx_bw_);
      }
      else
      {
        this->write_reg_(CC1101_IOCFG2, 0x0D);
        this->write_reg_(CC1101_IOCFG0, 0x0D);
        this->write_reg_(CC1101_PKTCTRL0, 0x32);
        this->write_reg_(CC1101_MDMCFG3, 0x93);
        this->write_reg_(CC1101_MDMCFG4, 7 + this->m4_rx_bw_);
      }
      this->write_modulation_();
    }

    void ElechouseCc1101::calibrate_()
    {
      float mhz = this->frequency_ / 1000000;
      if (mhz >= 300 && mhz <= 348)
      {
        this->write_reg_(CC1101_FSCTRL0, map(mhz, 300, 348, this->clb1_[0], this->clb1_[1]));
        if (mhz < 322.88)
        {
          this->write_reg_(CC1101_TEST0, 0x0B);
        }
        else
        {
          this->write_reg_(CC1101_TEST0, 0x09);
          int s = this->read_reg_(CC1101_FSCAL2);
          if (s < 32)
          {
            this->write_reg_(CC1101_FSCAL2, s + 32);
          }
          if (this->last_pa_ != 1)
          {
            this->write_pa_();
          }
        }
      }
      else if (mhz >= 378 && mhz <= 464)
      {
        this->write_reg_(CC1101_FSCTRL0, map(mhz, 378, 464, this->clb2_[0], this->clb2_[1]));
        if (mhz < 430.5)
        {
          this->write_reg_(CC1101_TEST0, 0x0B);
        }
        else
        {
          this->write_reg_(CC1101_TEST0, 0x09);
          int s = this->read_reg_(CC1101_FSCAL2);
          if (s < 32)
          {
            this->write_reg_(CC1101_FSCAL2, s + 32);
          }
          if (this->last_pa_ != 2)
          {
            this->write_pa_();
          }
        }
      }
      else if (mhz >= 779 && mhz <= 899.99)
      {
        this->write_reg_(CC1101_FSCTRL0, map(mhz, 779, 899, this->clb3_[0], this->clb3_[1]));
        if (mhz < 861)
        {
          this->write_reg_(CC1101_TEST0, 0x0B);
        }
        else
        {
          this->write_reg_(CC1101_TEST0, 0x09);
          int s = this->read_reg_(CC1101_FSCAL2);
          if (s < 32)
          {
            this->write_reg_(CC1101_FSCAL2, s + 32);
          }
          if (this->last_pa_ != 3)
          {
            this->write_pa_();
          }
        }
      }
      else if (mhz >= 900 && mhz <= 928)
      {
        this->write_reg_(CC1101_FSCTRL0, map(mhz, 900, 928, this->clb4_[0], this->clb4_[1]));
        this->write_reg_(CC1101_TEST0, 0x09);
        int s = this->read_reg_(CC1101_FSCAL2);
        if (s < 32)
        {
          this->write_reg_(CC1101_FSCAL2, s + 32);
        }
        if (this->last_pa_ != 4)
        {
          this->write_pa_();
        }
      }
    }

    void ElechouseCc1101::write_modulation_()
    {
      this->read_mdmcfg2_();
      uint8_t frend0;
      switch (this->modulation_)
      {
      case 0:
        this->m2_mod_fm_ = 0x00;
        frend0 = 0x10;
        break; // 2-FSK
      case 1:
        this->m2_mod_fm_ = 0x10;
        frend0 = 0x10;
        break; // GFSK
      case 2:
        this->m2_mod_fm_ = 0x30;
        frend0 = 0x11;
        break; // ASK
      case 3:
        this->m2_mod_fm_ = 0x40;
        frend0 = 0x10;
        break; // 4-FSK
      case 4:
        this->m2_mod_fm_ = 0x70;
        frend0 = 0x10;
        break; // MSK
      default:
        ESP_LOGE(TAG, "Modulation %d not supported.", this->modulation_);
        return;
      }
      this->write_reg_(CC1101_MDMCFG2, this->m2_dc_off_ + this->m2_mod_fm_ + this->m2_man_ch_ + this->m2_sync_m_);
      this->write_reg_(CC1101_FREND0, frend0);
      this->write_pa_();
    }

    void ElechouseCc1101::write_pa_()
    {
      int a;
      float mhz = this->frequency_ / 1000000;

      if (mhz >= 300 && mhz <= 348)
      {
        if (this->pa_ <= -30)
        {
          a = PA_TABLE_315[0];
        }
        else if (this->pa_ > -30 && this->pa_ <= -20)
        {
          a = PA_TABLE_315[1];
        }
        else if (this->pa_ > -20 && this->pa_ <= -15)
        {
          a = PA_TABLE_315[2];
        }
        else if (this->pa_ > -15 && this->pa_ <= -10)
        {
          a = PA_TABLE_315[3];
        }
        else if (this->pa_ > -10 && this->pa_ <= 0)
        {
          a = PA_TABLE_315[4];
        }
        else if (this->pa_ > 0 && this->pa_ <= 5)
        {
          a = PA_TABLE_315[5];
        }
        else if (this->pa_ > 5 && this->pa_ <= 7)
        {
          a = PA_TABLE_315[6];
        }
        else if (this->pa_ > 7)
        {
          a = PA_TABLE_315[7];
        }
        this->last_pa_ = 1;
      }
      else if (mhz >= 378 && mhz <= 464)
      {
        if (this->pa_ <= -30)
        {
          a = PA_TABLE_433[0];
        }
        else if (this->pa_ > -30 && this->pa_ <= -20)
        {
          a = PA_TABLE_433[1];
        }
        else if (this->pa_ > -20 && this->pa_ <= -15)
        {
          a = PA_TABLE_433[2];
        }
        else if (this->pa_ > -15 && this->pa_ <= -10)
        {
          a = PA_TABLE_433[3];
        }
        else if (this->pa_ > -10 && this->pa_ <= 0)
        {
          a = PA_TABLE_433[4];
        }
        else if (this->pa_ > 0 && this->pa_ <= 5)
        {
          a = PA_TABLE_433[5];
        }
        else if (this->pa_ > 5 && this->pa_ <= 7)
        {
          a = PA_TABLE_433[6];
        }
        else if (this->pa_ > 7)
        {
          a = PA_TABLE_433[7];
        }
        this->last_pa_ = 2;
      }
      else if (mhz >= 779 && mhz <= 899.99)
      {
        if (this->pa_ <= -30)
        {
          a = PA_TABLE_868[0];
        }
        else if (this->pa_ > -30 && this->pa_ <= -20)
        {
          a = PA_TABLE_868[1];
        }
        else if (this->pa_ > -20 && this->pa_ <= -15)
        {
          a = PA_TABLE_868[2];
        }
        else if (this->pa_ > -15 && this->pa_ <= -10)
        {
          a = PA_TABLE_868[3];
        }
        else if (this->pa_ > -10 && this->pa_ <= -6)
        {
          a = PA_TABLE_868[4];
        }
        else if (this->pa_ > -6 && this->pa_ <= 0)
        {
          a = PA_TABLE_868[5];
        }
        else if (this->pa_ > 0 && this->pa_ <= 5)
        {
          a = PA_TABLE_868[6];
        }
        else if (this->pa_ > 5 && this->pa_ <= 7)
        {
          a = PA_TABLE_868[7];
        }
        else if (this->pa_ > 7 && this->pa_ <= 10)
        {
          a = PA_TABLE_868[8];
        }
        else if (this->pa_ > 10)
        {
          a = PA_TABLE_868[9];
        }
        this->last_pa_ = 3;
      }
      else if (mhz >= 900 && mhz <= 928)
      {
        if (this->pa_ <= -30)
        {
          a = PA_TABLE_915[0];
        }
        else if (this->pa_ > -30 && this->pa_ <= -20)
        {
          a = PA_TABLE_915[1];
        }
        else if (this->pa_ > -20 && this->pa_ <= -15)
        {
          a = PA_TABLE_915[2];
        }
        else if (this->pa_ > -15 && this->pa_ <= -10)
        {
          a = PA_TABLE_915[3];
        }
        else if (this->pa_ > -10 && this->pa_ <= -6)
        {
          a = PA_TABLE_915[4];
        }
        else if (this->pa_ > -6 && this->pa_ <= 0)
        {
          a = PA_TABLE_915[5];
        }
        else if (this->pa_ > 0 && this->pa_ <= 5)
        {
          a = PA_TABLE_915[6];
        }
        else if (this->pa_ > 5 && this->pa_ <= 7)
        {
          a = PA_TABLE_915[7];
        }
        else if (this->pa_ > 7 && this->pa_ <= 10)
        {
          a = PA_TABLE_915[8];
        }
        else if (this->pa_ > 10)
        {
          a = PA_TABLE_915[9];
        }
        this->last_pa_ = 4;
      }
      if (this->modulation_ == 2)
      {
        PA_TABLE[0] = 0;
        PA_TABLE[1] = a;
      }
      else
      {
        PA_TABLE[0] = a;
        PA_TABLE[1] = 0;
      }
      this->write_burst_reg_(CC1101_PATABLE, PA_TABLE, 8);
    }
  }
}
