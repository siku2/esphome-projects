#include "fdxb_emulator.h"

#include <cstring>

#include "esp_random.h"
#include "esp_rom_crc.h"

#include "esphome/core/log.h"

namespace esphome::fdxb_emulator {

static const char *const TAG = "fdxb_emulator";

static constexpr uint32_t RESOLUTION_HZ = 1000000;
static constexpr size_t MEM_BLOCK_SYMBOLS = 192;
static constexpr size_t TRANS_QUEUE_DEPTH = 4;
static constexpr size_t TRANSMIT_SYMBOLS = 128;
// FDX-B runs at 134.2 kHz / 32 = 4194 bit/s, so 238 us per bit.
static constexpr uint16_t HALF_BIT_TICKS = 119;
static constexpr uint16_t COUNTRY_CODE = 999;
static constexpr uint32_t LEAD_IN_ITEM = 0x80778077;
static constexpr uint32_t END_MARKER = 0x00008000;

void FdxbEmulator::setup() {
  rmt_tx_channel_config_t channel_config = {};
  channel_config.gpio_num = static_cast<gpio_num_t>(this->pin_);
  channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
  channel_config.resolution_hz = RESOLUTION_HZ;
  channel_config.mem_block_symbols = MEM_BLOCK_SYMBOLS;
  channel_config.trans_queue_depth = TRANS_QUEUE_DEPTH;
  esp_err_t err = rmt_new_tx_channel(&channel_config, &this->channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Creating the RMT channel failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  rmt_copy_encoder_config_t encoder_config = {};
  err = rmt_new_copy_encoder(&encoder_config, &this->encoder_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Creating the RMT encoder failed: %s", esp_err_to_name(err));
    rmt_del_channel(this->channel_);
    this->channel_ = nullptr;
    this->mark_failed();
  }
}

void FdxbEmulator::dump_config() {
  LOG_SWITCH("", "FDX-B Emulator", this);
  ESP_LOGCONFIG(TAG, "  Pin: GPIO%u", this->pin_);
}

void FdxbEmulator::write_state(bool state) {
  if (state) {
    this->start();
  } else {
    this->stop();
  }
  this->publish_state(this->running_);
}

// Differential biphase: every bit starts with a level flip, a zero bit flips
// again halfway through the bit.
void FdxbEmulator::encode_bit_(rmt_symbol_word_t *item, bool one, bool *level) {
  bool old_level = *level;
  bool next = !old_level;
  item->level0 = next;
  item->duration0 = HALF_BIT_TICKS;
  item->duration1 = HALF_BIT_TICKS;
  if (one) {
    item->level1 = next;
    *level = next;
  } else {
    item->level1 = old_level;
  }
}

void FdxbEmulator::build_frame_(uint16_t country_code, uint64_t national_id) {
  memset(this->items_, 0, sizeof(this->items_));

  uint8_t data[8] = {};
  memcpy(data, &national_id, 5);
  data[4] = (data[4] & 0x3f) | (country_code << 6);
  data[5] = 0x00;
  for (int i = 0; i < 8; i++)
    data[5] |= (country_code & (1 << (i + 2))) >> 2;
  data[7] = 0x80;
  uint16_t crc = ~esp_rom_crc16_le(0xffff, data, sizeof(data));

  bool level = true;
  rmt_symbol_word_t *item = this->items_;
  (item++)->val = LEAD_IN_ITEM;
  for (int i = 0; i < 10; i++) {
    this->encode_bit_(item + i, false, &level);
  }
  item += 10;
  for (int i = 0; i < 8; i++) {
    this->encode_bit_(item, true, &level);
    item++;
    for (int bit = 0; bit < 8; bit++) {
      this->encode_bit_(item + bit, (data[i] >> bit) & 1, &level);
    }
    item += 8;
  }
  this->encode_bit_(item, true, &level);
  item++;
  for (int bit = 0; bit < 8; bit++) {
    this->encode_bit_(item + bit, (crc >> bit) & 1, &level);
  }
  item += 8;
  this->encode_bit_(item, true, &level);
  item++;
  for (int bit = 8; bit < 16; bit++) {
    this->encode_bit_(item, (crc >> bit) & 1, &level);
    item++;
  }
  for (int byte = 0; byte < 3; byte++) {
    this->encode_bit_(item, true, &level);
    item++;
    for (int bit = 0; bit < 8; bit++) {
      this->encode_bit_(item + bit, false, &level);
    }
    item += 8;
  }
  item->val = END_MARKER;
}

void FdxbEmulator::start() {
  if (this->is_failed() || this->running_) {
    return;
  }
  uint64_t national_id = (1ULL << 32) | esp_random();
  this->build_frame_(COUNTRY_CODE, national_id);

  esp_err_t err = rmt_enable(this->channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Enabling the RMT channel failed: %s", esp_err_to_name(err));
    return;
  }
  rmt_transmit_config_t transmit_config = {};
  transmit_config.loop_count = -1;
  err = rmt_transmit(this->channel_, this->encoder_, this->items_, sizeof(rmt_symbol_word_t) * TRANSMIT_SYMBOLS,
                     &transmit_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Starting the transmission failed: %s", esp_err_to_name(err));
    rmt_disable(this->channel_);
    return;
  }
  this->running_ = true;
  ESP_LOGI(TAG, "Started with ID: %llu", national_id);
}

void FdxbEmulator::stop() {
  if (this->is_failed() || !this->running_) {
    return;
  }
  esp_err_t err = rmt_disable(this->channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Disabling the RMT channel failed: %s", esp_err_to_name(err));
  }
  this->running_ = false;
  ESP_LOGI(TAG, "Stopped");
}

}  // namespace esphome::fdxb_emulator
