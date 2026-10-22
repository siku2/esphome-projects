#ifdef USE_ESP32

#include "sths34pf80.h"

#include <algorithm>
#include <cmath>

#include "freertos/task.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::sths34pf80 {

static const char *const TAG = "sths34pf80";

static constexpr uint8_t REG_FUNC_CFG_ADDR = 0x08;
static constexpr uint8_t REG_FUNC_CFG_DATA = 0x09;
static constexpr uint8_t REG_WHO_AM_I = 0x0f;
static constexpr uint8_t REG_AVG_TRIM = 0x10;
static constexpr uint8_t REG_PAGE_RW = 0x11;
static constexpr uint8_t REG_CTRL0 = 0x17;
static constexpr uint8_t REG_CTRL1 = 0x20;
static constexpr uint8_t REG_CTRL2 = 0x21;
static constexpr uint8_t REG_CTRL3 = 0x22;
static constexpr uint8_t REG_STATUS = 0x23;
static constexpr uint8_t REG_FUNC_STATUS = 0x25;
static constexpr uint8_t REG_TPRESENCE = 0x3a;
static constexpr uint8_t REG_TMOTION = 0x3c;

static constexpr uint8_t EMB_PRESENCE_THS = 0x20;
static constexpr uint8_t EMB_MOTION_THS = 0x22;
static constexpr uint8_t EMB_HYST_MOTION = 0x26;
static constexpr uint8_t EMB_HYST_PRESENCE = 0x27;
static constexpr uint8_t EMB_ALGO_CONFIG = 0x28;
static constexpr uint8_t EMB_RESET_ALGO = 0x2a;

static constexpr uint8_t WHO_AM_I = 0xd3;
static constexpr uint8_t STATUS_DRDY = 0x04;
static constexpr uint8_t FUNC_STATUS_PRESENCE = 0x04;
static constexpr int WHO_AM_I_ATTEMPTS = 5;
static constexpr int RESET_DRDY_TRIES = 5;
static constexpr uint32_t SETTLE_DELAY_MS = 10;
static constexpr uint32_t HOLD_MS = 2000;
static constexpr uint8_t ODR_30_HZ = 8;
static constexpr uint8_t AVG_CODE = 2;
static constexpr uint8_t INT_ROUTE = 2;
static constexpr uint8_t INT_MASK = 2;

namespace {
/// Serialises multi-step register sequences between the main loop and the scan task.
class Guard {
 public:
  explicit Guard(SemaphoreHandle_t lock) : lock_(lock) { xSemaphoreTake(this->lock_, portMAX_DELAY); }
  ~Guard() { xSemaphoreGive(this->lock_); }
  Guard(const Guard &) = delete;
  Guard &operator=(const Guard &) = delete;

 private:
  SemaphoreHandle_t lock_;
};

uint16_t number_value(const number::Number *number, uint16_t fallback) {
  if (number == nullptr || !number->has_state() || std::isnan(number->state)) {
    return fallback;
  }
  return static_cast<uint16_t>(std::clamp(std::lround(number->state), 0L, 65535L));
}
}  // namespace

void Sths34pf80::setup() {
  this->lock_ = xSemaphoreCreateMutex();
  if (this->lock_ == nullptr) {
    this->mark_failed();
    return;
  }
  this->interrupt_pin_->setup();

  uint8_t id = 0;
  bool found = false;
  for (int attempt = 1; attempt <= WHO_AM_I_ATTEMPTS && !found; attempt++) {
    found = this->read8_(REG_WHO_AM_I, id) && id == WHO_AM_I;
    if (!found) {
      ESP_LOGW(TAG, "WHO_AM_I attempt %d/%d: 0x%02X (want 0x%02X)", attempt, WHO_AM_I_ATTEMPTS, id, WHO_AM_I);
      vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
    }
  }
  if (!found) {
    ESP_LOGE(TAG, "ID invalid: 0x%02X, should be 0x%02X", id, WHO_AM_I);
    this->mark_failed();
    return;
  }

  bool ok;
  {
    Guard guard(this->lock_);
    ok = this->apply_thresholds_() && this->read8_(REG_WHO_AM_I, id) && id == WHO_AM_I;
  }
  if (!ok) {
    ESP_LOGE(TAG, "Setting up the sensor failed");
    this->mark_failed();
    return;
  }

  if (this->motion_threshold_number_ != nullptr) {
    this->motion_threshold_number_->add_on_state_callback([this](float) { this->on_threshold_change_(); });
  }
  if (this->presence_threshold_number_ != nullptr) {
    this->presence_threshold_number_->add_on_state_callback([this](float) { this->on_threshold_change_(); });
  }

  this->publish_initial_state(false);
  this->interrupt_pin_->attach_interrupt(&Sths34pf80::isr_, this, gpio::INTERRUPT_RISING_EDGE);
  this->disable_loop();
}

void Sths34pf80::loop() {
  this->disable_loop();
  if (!this->edge_) {
    return;
  }
  this->edge_ = false;
  this->publish_state(true);
  this->arm_hold_();
  for (const auto &callback : this->motion_callbacks_) {
    callback();
  }
}

void Sths34pf80::dump_config() {
  LOG_BINARY_SENSOR("", "STHS34PF80", this);
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Interrupt pin: ", this->interrupt_pin_);
  ESP_LOGCONFIG(TAG,
                "  Motion threshold: %u%s\n"
                "  Presence threshold: %u%s",
                this->motion_threshold_value_(), this->motion_threshold_number_ != nullptr ? " (number)" : "",
                this->presence_threshold_value_(), this->presence_threshold_number_ != nullptr ? " (number)" : "");
}

void IRAM_ATTR Sths34pf80::isr_(Sths34pf80 *self) {
  self->edge_ = true;
  self->enable_loop_soon_any_context();
}

void Sths34pf80::arm_hold_() {
  this->set_timeout("hold", HOLD_MS, [this]() { this->on_hold_expired_(); });
}

void Sths34pf80::on_hold_expired_() {
  uint8_t flags = 0;
  bool present;
  {
    Guard guard(this->lock_);
    present = this->read8_(REG_FUNC_STATUS, flags) && (flags & FUNC_STATUS_PRESENCE);
  }
  if (present) {
    this->arm_hold_();
  } else {
    this->publish_state(false);
  }
}

uint16_t Sths34pf80::motion_threshold_value_() const {
  return number_value(this->motion_threshold_number_, this->motion_threshold_);
}

uint16_t Sths34pf80::presence_threshold_value_() const {
  return number_value(this->presence_threshold_number_, this->presence_threshold_);
}

void Sths34pf80::on_threshold_change_() {
  if (!this->busy_) {
    this->apply_pending_thresholds_();
  }
}

void Sths34pf80::set_busy(bool busy) {
  this->busy_ = busy;
  if (!busy) {
    this->defer([this]() {
      if (!this->busy_) {
        this->apply_pending_thresholds_();
      }
    });
  }
}

void Sths34pf80::apply_pending_thresholds_() {
  if (this->is_failed() || (this->motion_threshold_value_() == this->applied_motion_threshold_ &&
                            this->presence_threshold_value_() == this->applied_presence_threshold_)) {
    return;
  }
  Guard guard(this->lock_);
  if (!this->apply_thresholds_()) {
    ESP_LOGW(TAG, "Applying the thresholds failed");
  }
}

bool Sths34pf80::read_sample(int16_t &motion, int16_t &presence) {
  if (this->is_failed()) {
    return false;
  }
  Guard guard(this->lock_);
  uint8_t status = 0;
  if (!this->read8_(REG_STATUS, status) || !(status & STATUS_DRDY)) {
    return false;
  }
  int16_t motion_raw;
  int16_t presence_raw;
  if (!this->read16_(REG_TMOTION, motion_raw) || !this->read16_(REG_TPRESENCE, presence_raw)) {
    return false;
  }
  motion = motion_raw;
  presence = presence_raw;
  uint8_t flags;
  this->read8_(REG_FUNC_STATUS, flags);
  return true;
}

bool Sths34pf80::read8_(uint8_t reg, uint8_t &value) { return this->read_register(reg, &value, 1) == i2c::ERROR_OK; }

bool Sths34pf80::write8_(uint8_t reg, uint8_t value) { return this->write_register(reg, &value, 1) == i2c::ERROR_OK; }

bool Sths34pf80::update8_(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t current;
  return this->read8_(reg, current) && this->write8_(reg, (current & mask) | value);
}

bool Sths34pf80::read16_(uint8_t reg, int16_t &value) {
  uint8_t data[2];
  if (this->read_register(reg, data, sizeof(data)) != i2c::ERROR_OK) {
    return false;
  }
  value = static_cast<int16_t>(data[0] | (data[1] << 8));
  return true;
}

bool Sths34pf80::apply_thresholds_() {
  const uint16_t motion = this->motion_threshold_value_();
  const uint16_t presence = this->presence_threshold_value_();
  ESP_LOGI(TAG, "Applying thresholds: motion %u, presence %u", motion, presence);

  bool ok = this->configure_algorithm_(motion, presence);
  ok &= this->update8_(REG_AVG_TRIM, 0xf8, AVG_CODE);
  ok &= this->update8_(REG_AVG_TRIM, 0xcf, AVG_CODE << 4);
  ok &= this->set_odr_(ODR_30_HZ);
  ok &= this->update8_(REG_CTRL1, 0xef, 1 << 4);
  ok &= this->configure_interrupt_();
  if (ok) {
    this->applied_motion_threshold_ = motion;
    this->applied_presence_threshold_ = presence;
  }
  return ok;
}

bool Sths34pf80::reset_() {
  bool ok = this->update8_(REG_CTRL2, 0x7f, 0x80);
  vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
  uint8_t value;
  ok &= this->read8_(REG_FUNC_STATUS, value);
  for (int tries = RESET_DRDY_TRIES; tries > 0; tries--) {
    if (this->read8_(REG_STATUS, value) && (value & STATUS_DRDY)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));
  }
  ok &= this->set_odr_(0);
  ok &= this->read8_(REG_FUNC_STATUS, value);
  return ok;
}

bool Sths34pf80::set_odr_(uint8_t odr) {
  this->powered_down_ = odr == 0;
  return this->update8_(REG_CTRL1, 0xf0, odr);
}

bool Sths34pf80::write_embedded_(uint8_t reg, uint8_t value) {
  bool ok = true;
  if (!this->powered_down_) {
    ok = this->reset_();
  }
  ok &= this->update8_(REG_CTRL2, 0xef, 0x10);
  ok &= this->update8_(REG_PAGE_RW, 0xbf, 0x40);
  ok &= this->write8_(REG_FUNC_CFG_ADDR, reg);
  ok &= this->write8_(REG_FUNC_CFG_DATA, value);
  ok &= this->update8_(REG_PAGE_RW, 0xbf, 0x00);
  ok &= this->update8_(REG_CTRL2, 0xef, 0x00);
  return ok;
}

bool Sths34pf80::configure_algorithm_(uint16_t motion, uint16_t presence) {
  bool ok = this->reset_();
  ok &= this->update8_(REG_CTRL0, 0x8f, 0x70);
  ok &= this->write_embedded_(EMB_MOTION_THS, motion & 0xff);
  ok &= this->write_embedded_(EMB_MOTION_THS + 1, motion >> 8);
  ok &= this->write_embedded_(EMB_PRESENCE_THS, presence & 0xff);
  ok &= this->write_embedded_(EMB_PRESENCE_THS + 1, presence >> 8);
  ok &= this->write_embedded_(EMB_HYST_MOTION, std::min<uint16_t>(motion / 10, 0xff));
  ok &= this->write_embedded_(EMB_HYST_PRESENCE, std::min<uint16_t>(presence / 10, 0xff));
  ok &= this->write_embedded_(EMB_ALGO_CONFIG, 8);
  ok &= this->write_embedded_(EMB_RESET_ALGO, 1);
  return ok;
}

bool Sths34pf80::configure_interrupt_() {
  bool ok = this->update8_(REG_CTRL3, 0xf8, INT_ROUTE);
  ok &= this->update8_(REG_CTRL3, 0xc7, INT_MASK << 3);
  ok &= this->update8_(REG_CTRL3, 0xfb, 0x00);
  ok &= this->update8_(REG_CTRL3, 0x3f, 0x00);
  return ok;
}

}  // namespace esphome::sths34pf80

#endif
