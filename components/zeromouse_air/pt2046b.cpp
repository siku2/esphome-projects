#include "pt2046b.h"

#include <algorithm>

#include "driver/gpio.h"
#include "esp_private/adc_private.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include "esphome/core/log.h"

namespace esphome::zeromouse_air {

static const char *const TAG = "zeromouse_air.pt2046b";

static constexpr int ADC_MAX = 4096;
static constexpr uint32_t ONE_SHOT_DELAY_MS = 10;

void Pt2046b::setup() {
  adc_unit_t unit;
  esp_err_t err = adc_oneshot_io_to_channel(this->adc_pin_, &unit, &this->channel_);
  if (err != ESP_OK || unit != ADC_UNIT_1) {
    ESP_LOGE(TAG, "GPIO%u is not an ADC1 pin", this->adc_pin_);
    this->mark_failed();
    return;
  }

  adc_oneshot_unit_init_cfg_t unit_config = {};
  unit_config.unit_id = ADC_UNIT_1;
  err = adc_oneshot_new_unit(&unit_config, &this->adc_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Creating the ADC unit failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  adc_oneshot_chan_cfg_t channel_config = {};
  channel_config.atten = ADC_ATTEN_DB_12;
  channel_config.bitwidth = ADC_BITWIDTH_12;
  err = adc_oneshot_config_channel(this->adc_, this->channel_, &channel_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Configuring the ADC channel failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  gpio_reset_pin(static_cast<gpio_num_t>(this->led_pin_));
  gpio_set_direction(static_cast<gpio_num_t>(this->led_pin_), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(this->led_pin_), 0);
}

void Pt2046b::dump_config() {
  ESP_LOGCONFIG(TAG,
                "PT2046B:\n"
                "  ADC pin: GPIO%u\n"
                "  LED pin: GPIO%u\n"
                "  Calibration tolerance: %u\n"
                "  Calibration max: %u",
                this->adc_pin_, this->led_pin_, this->calibration_tolerance_, this->calibration_max_);
}

bool Pt2046b::power_on() {
  this->one_shot_ = false;
  return !this->is_failed();
}

void Pt2046b::power_off() { this->one_shot_ = true; }

bool Pt2046b::read(AirSample &sample) {
  if (this->is_failed()) {
    return false;
  }
  if (this->one_shot_) {
    vTaskDelay(pdMS_TO_TICKS(ONE_SHOT_DELAY_MS));
  }

  const auto led = static_cast<gpio_num_t>(this->led_pin_);
  int environment = 0;
  int lit = 0;
  if (adc_oneshot_read(this->adc_, this->channel_, &environment) != ESP_OK) {
    return false;
  }

  portENTER_CRITICAL(&this->lock_);
  gpio_set_level(led, 1);
  sample.time_us = esp_timer_get_time();
  esp_err_t err = adc_oneshot_read_isr(this->adc_, this->channel_, &lit);
  gpio_set_level(led, 0);
  portEXIT_CRITICAL(&this->lock_);
  if (err != ESP_OK) {
    return false;
  }

  environment = std::clamp(environment, 0, ADC_MAX);
  lit = std::clamp(lit, 0, ADC_MAX);
  sample.value = std::max(0, lit - environment);
  sample.environment = environment;
  sample.mean = 0;
  sample.slope = 0;
  return true;
}

}  // namespace esphome::zeromouse_air
