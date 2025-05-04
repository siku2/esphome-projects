#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace esphome {
namespace adc {

class ADCSensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler {
 public:
  /// Set the attenuation for this pin. Only available on the ESP32.
  void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }
  void set_channel1(adc_channel_t channel) {
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
  adc_channel_t channel1_;

  adc_atten_t attenuation_{ADC_ATTEN_DB_0};
  bool autorange_{false};

  adc_cali_handle_t cali_chan0_handle_;
};

}  // namespace adc
}  // namespace esphome
