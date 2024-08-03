#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include <esp_adc_cal.h>
#include "driver/adc.h"

namespace esphome {
namespace adc {

static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_12;

class ADCSensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler {
 public:
  /// Set the attenuation for this pin. Only available on the ESP32.
  void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
  void set_channel1(adc1_channel_t channel) {
    this->channel1_ = channel;
  }
  void set_autorange(bool autorange) { this->autorange_ = autorange; }

  /// Update ADC values
  void update() override;
  /// Setup ADC
  void setup() override;
  void dump_config() override;
  /// `HARDWARE_LATE` setup priority
  float get_setup_priority() const override;
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
  void set_sample_count(uint8_t sample_count);
  float sample() override;

 protected:
  InternalGPIOPin *pin_;
  bool output_raw_{false};
  uint8_t sample_count_{1};

  adc_atten_t attenuation_{ADC_ATTEN_DB_0};
  adc1_channel_t channel1_{ADC1_CHANNEL_MAX};
  bool autorange_{false};
  esp_adc_cal_characteristics_t cal_characteristics_[SOC_ADC_ATTEN_NUM] = {};
};

}  // namespace adc
}  // namespace esphome
