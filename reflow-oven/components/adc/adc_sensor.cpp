#include "adc_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace adc {

static const char *const TAG = "adc";

#ifndef SOC_ADC_RTC_MAX_BITWIDTH
static const int32_t SOC_ADC_RTC_MAX_BITWIDTH = 12;
#endif

static const int ADC_MAX = (1 << SOC_ADC_RTC_MAX_BITWIDTH) - 1;    // 4095 (12 bit) or 8191 (13 bit)
static const int ADC_HALF = (1 << SOC_ADC_RTC_MAX_BITWIDTH) >> 1;  // 2048 (12 bit) or 4096 (13 bit)

static adc_oneshot_unit_handle_t g_adc_handle;
static bool g_initialized;

void ADCSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADC '%s'...", this->get_name().c_str());

  esp_err_t ret = ESP_FAIL;

  if (!g_initialized) {
    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ret = adc_oneshot_new_unit(&cfg, &g_adc_handle);
    g_initialized = true;
    ESP_LOGCONFIG(TAG, "'%s': adc_oneshot_new_unit()=%d", this->get_name().c_str(), ret);
  }

  adc_oneshot_chan_cfg_t config = {
    .atten = this->attenuation_,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_oneshot_config_channel(g_adc_handle, this->channel1_, &config);
  ESP_LOGCONFIG(TAG, "'%s': adc_oneshot_config_channel()=%d", this->get_name().c_str(), ret);

  ESP_LOGCONFIG(TAG, "ADC '%s' calibration scheme version is Curve Fitting", this->get_name().c_str());
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = this->channel1_,
      .atten = this->attenuation_,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &this->cali_chan0_handle_);
  ESP_LOGCONFIG(TAG, "'%s': adc_cali_create_scheme_curve_fitting()=%d", this->get_name().c_str(), ret);

  ESP_LOGCONFIG(TAG, "ADC '%s' setup finished!", this->get_name().c_str());
}

void ADCSensor::dump_config() {
  LOG_SENSOR("", "ADC Sensor", this);

  LOG_PIN("  Pin: ", this->pin_);
  // if (this->autorange_) {
  //   ESP_LOGCONFIG(TAG, "  Attenuation: auto");
  // } else {
  //   switch (this->attenuation_) {
  //     case ADC_ATTEN_DB_0:
  //       ESP_LOGCONFIG(TAG, "  Attenuation: 0db");
  //       break;
  //     case ADC_ATTEN_DB_2_5:
  //       ESP_LOGCONFIG(TAG, "  Attenuation: 2.5db");
  //       break;
  //     case ADC_ATTEN_DB_6:
  //       ESP_LOGCONFIG(TAG, "  Attenuation: 6db");
  //       break;
  //     case ADC_ATTEN_DB_12_COMPAT:
  //       ESP_LOGCONFIG(TAG, "  Attenuation: 12db");
  //       break;
  //     default:  // This is to satisfy the unused ADC_ATTEN_MAX
  //       break;
  //   }
  // }

  ESP_LOGCONFIG(TAG, "  Samples: %i", this->sample_count_);
  LOG_UPDATE_INTERVAL(this);
}

float ADCSensor::get_setup_priority() const { return setup_priority::DATA; }
void ADCSensor::update() {
  float value_v = this->sample();
  ESP_LOGV(TAG, "'%s': Got voltage=%.4fV", this->get_name().c_str(), value_v);
  this->publish_state(value_v);
}

void ADCSensor::set_sample_count(uint8_t sample_count) {
  if (sample_count != 0) {
    this->sample_count_ = sample_count;
  }
}

float ADCSensor::sample() {
  int adc_raw = 0;
  int voltage = 0;

  adc_oneshot_read(g_adc_handle, this->channel1_, &adc_raw);
  ESP_LOGV(TAG, "'%s': Got raw=%d", this->get_name().c_str(), adc_raw);
  adc_cali_raw_to_voltage(this->cali_chan0_handle_, adc_raw, &voltage);

  return voltage / 1000.0;
}
}  // namespace adc
}  // namespace esphome
