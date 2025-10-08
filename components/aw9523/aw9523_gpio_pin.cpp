#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "aw9523_gpio_pin.h"

namespace esphome {
namespace aw9523 {

static const char *const TAG = "aw9523_gpio_pin";

void AW9523GPIOPin::setup() {
  pin_mode(flags_);
  forward_interrupt(forward_interrupt_);
}
void AW9523GPIOPin::pin_mode(gpio::Flags flags) {
  this->flags_ = flags;
  this->parent_->pin_mode(this->pin_, flags);
}
void AW9523GPIOPin::forward_interrupt(bool enable) {
  this->forward_interrupt_ = enable;
  this->parent_->forward_interrupt(this->pin_, enable);
}
gpio::Flags AW9523GPIOPin::get_flags() const { return this->flags_; }
bool AW9523GPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void AW9523GPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string AW9523GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via AW9523", pin_);
  return buffer;
}
}  // namespace aw9523
}  // namespace esphome
