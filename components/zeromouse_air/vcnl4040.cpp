#include "esphome/core/defines.h"

#ifdef USE_ZEROMOUSE_AIR_VCNL4040

#include "vcnl4040.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esphome/core/log.h"

namespace esphome::zeromouse_air {

static const char *const TAG = "zeromouse_air.vcnl4040";

static constexpr uint8_t ALS_CONF = 0x00;
static constexpr uint8_t PS_CONF1 = 0x03;
static constexpr uint8_t PS_CONF3 = 0x04;
static constexpr uint8_t PS_DATA = 0x08;
static constexpr uint8_t ALS_DATA = 0x09;
static constexpr uint8_t ID = 0x0c;

static constexpr uint16_t DEVICE_ID = 0x0186;
static constexpr int ID_ATTEMPTS = 5;
static constexpr uint32_t SETTLE_DELAY_MS = 10;

static constexpr uint8_t PS_SD = 0x01;
static constexpr uint8_t PS_IT_4T = 0x0c;
static constexpr uint8_t PS_HD_16_BIT = 0x08;
static constexpr uint8_t PS_AF = 0x08;
static constexpr uint8_t PS_TRIG = 0x04;
static constexpr uint8_t PS_SMART_PERS = 0x10;
static constexpr uint8_t LED_I_200_MA = 0x07;

void Vcnl4040::setup() {
  uint16_t id = 0;
  for (int attempt = 1; attempt <= ID_ATTEMPTS; attempt++) {
    if (this->read_word_(ID, id) && id == DEVICE_ID) {
      return;
    }
    ESP_LOGW(TAG, "ID attempt %d/%d: 0x%04X (want 0x%04X)", attempt, ID_ATTEMPTS, id, DEVICE_ID);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
  }
  ESP_LOGE(TAG, "ID invalid: 0x%04X, should be 0x%04X", id, DEVICE_ID);
  this->mark_failed();
}

void Vcnl4040::dump_config() {
  ESP_LOGCONFIG(TAG,
                "VCNL4040:\n"
                "  Calibration tolerance: %u\n"
                "  Calibration max: %u",
                this->calibration_tolerance_, this->calibration_max_);
  LOG_I2C_DEVICE(this);
}

bool Vcnl4040::power_on() {
  if (this->is_failed()) {
    return false;
  }
  if (this->powered_on_) {
    return true;
  }

  bool ok = this->update_(PS_CONF3, Byte::HIGH_BYTE, 0xf8, LED_I_200_MA);
  ok &= this->update_(PS_CONF1, Byte::LOW_BYTE, 0x3f, 0x00);
  ok &= this->update_(PS_CONF3, Byte::LOW_BYTE, 0xf7, 0x00);
  this->one_shot_ = false;
  ok &= this->update_(PS_CONF1, Byte::LOW_BYTE, 0xf1, PS_IT_4T);
  ok &= this->update_(PS_CONF1, Byte::HIGH_BYTE, 0xf7, PS_HD_16_BIT);
  ok &= this->update_(PS_CONF3, Byte::LOW_BYTE, 0xef, PS_SMART_PERS);
  ok &= this->update_(ALS_CONF, Byte::LOW_BYTE, 0xfe, 0x00);
  ok &= this->update_(ALS_CONF, Byte::LOW_BYTE, 0x3f, 0x00);
  ok &= this->update_(PS_CONF3, Byte::HIGH_BYTE, 0x7f, 0x00);
  ok &= this->update_(PS_CONF1, Byte::LOW_BYTE, 0xfe, 0x00);
  if (!ok) {
    ESP_LOGE(TAG, "Powering on failed");
    return false;
  }
  this->powered_on_ = true;

  AirSample discarded;
  this->read(discarded);
  vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
  return true;
}

void Vcnl4040::power_off() {
  this->powered_on_ = false;
  this->update_(PS_CONF1, Byte::LOW_BYTE, 0xfe, PS_SD);
}

void Vcnl4040::set_one_shot(bool one_shot) {
  this->one_shot_ = one_shot;
  this->update_(PS_CONF3, Byte::LOW_BYTE, 0xf7, one_shot ? PS_AF : 0x00);
}

bool Vcnl4040::read(AirSample &sample) {
  if (this->is_failed()) {
    return false;
  }
  if (this->one_shot_) {
    if (!this->update_(PS_CONF3, Byte::LOW_BYTE, 0xfb, PS_TRIG)) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
  }

  uint16_t value = 0;
  uint16_t environment = 0;
  if (!this->read_word_(PS_DATA, value) || !this->read_word_(ALS_DATA, environment)) {
    return false;
  }
  sample.value = value;
  sample.environment = environment;
  sample.mean = 0;
  sample.slope = 0;
  sample.time_us = esp_timer_get_time();
  return true;
}

bool Vcnl4040::read_word_(uint8_t reg, uint16_t &value) {
  uint8_t data[2];
  if (this->read_register(reg, data, sizeof(data)) != i2c::ERROR_OK) {
    return false;
  }
  value = static_cast<uint16_t>(data[0] | (data[1] << 8));
  return true;
}

bool Vcnl4040::update_(uint8_t reg, Byte byte, uint8_t mask, uint8_t value) {
  uint16_t word;
  if (!this->read_word_(reg, word)) {
    return false;
  }
  if (byte == Byte::LOW_BYTE) {
    word = (word & 0xff00) | static_cast<uint8_t>((word & mask) | value);
  } else {
    word = (word & 0x00ff) | (static_cast<uint16_t>(static_cast<uint8_t>(((word >> 8) & mask) | value)) << 8);
  }
  const uint8_t data[2] = {static_cast<uint8_t>(word & 0xff), static_cast<uint8_t>(word >> 8)};
  return this->write_register(reg, data, sizeof(data)) == i2c::ERROR_OK;
}

}  // namespace esphome::zeromouse_air

#endif
