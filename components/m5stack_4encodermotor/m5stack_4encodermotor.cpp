#include "m5stack_4encodermotor.h"

#include "esphome/core/log.h"

#define REG_MOTOR_SPEED(motor) (0x40 + (motor))
#define REG_MOTOR_MODE(motor) (0x50 + (motor) * 0x10)
#define REG_SOFT_START_STOP 0xD1
#define REG_CURRENT 0x90
#define REG_VOLTAGE 0xB0
#define REG_BOOTLOADER_VERSION 0xFC
#define REG_FIRMWARE_VERSION 0xFE

namespace esphome
{
  namespace m5stack_4encodermotor
  {
    static const char *const TAG = "m5stack_4encodermotor";

    void M5Stack4EncoderMotor::setup()
    {
      this->fw_version_ = this->reg(REG_FIRMWARE_VERSION).get();
      if (this->fw_version_ == 0)
      {
        this->mark_failed();
      }
      else if (this->fw_version_ < 4)
      {
        ESP_LOGW(TAG, "M5Stack 4 Encoder Motor FW version is %u. Consider updating to version 4 since it contains important fixes.", this->fw_version_);
      }
      this->bootloader_version_ = this->reg(REG_BOOTLOADER_VERSION).get();

      ESP_LOGI(TAG, "FW version: %u, Bootloader version: %u", this->fw_version_, this->bootloader_version_);
    }

    void M5Stack4EncoderMotor::dump_config()
    {
      LOG_I2C_DEVICE(this);
      ESP_LOGCONFIG(TAG, "M5Stack 4 Encoder Motor:");
      LOG_SENSOR("  ", "Current", this->current_sensor_);
      LOG_SENSOR("  ", "Voltage", this->voltage_sensor_);
      LOG_UPDATE_INTERVAL(this);

      uint8_t soft_start_stop = this->reg(REG_SOFT_START_STOP).get();
      for (Motor motor = M1; motor <= M4; motor = static_cast<Motor>(motor + 1))
      {
        ESP_LOGCONFIG(TAG, "  Motor %d:", motor + 1);
        ESP_LOGCONFIG(TAG, "    Mode: %d", this->reg(REG_MOTOR_MODE(motor)).get());
        ESP_LOGCONFIG(TAG, "    Speed: %d", this->reg(REG_MOTOR_SPEED(motor)).get());
        ESP_LOGCONFIG(TAG, "    Soft Start/Stop: %s", (soft_start_stop & (1 << motor)) ? "ENABLED" : "DISABLED");
      }
    }

    void M5Stack4EncoderMotor::update()
    {
      if (this->current_sensor_ != nullptr)
      {
        auto current = this->read_current();
        if (current.has_value())
        {
          this->current_sensor_->publish_state(*current);
        }
      }

      if (this->voltage_sensor_ != nullptr)
      {
        auto voltage = this->read_voltage();
        if (voltage.has_value())
        {
          this->voltage_sensor_->publish_state(*voltage);
        }
      }
    }

    void M5Stack4EncoderMotor::loop()
    {
    }

    void M5Stack4EncoderMotor::set_motor_mode(Motor motor, Mode mode)
    {
      this->reg(REG_MOTOR_MODE(motor)) = mode;
      ESP_LOGD(TAG, "Set motor %d mode to %d", motor + 1, mode);
    }

    void M5Stack4EncoderMotor::set_motor_speed(Motor motor, int8_t speed)
    {
      this->reg(REG_MOTOR_SPEED(motor)) = speed;
      ESP_LOGD(TAG, "Set motor %d speed to %d", motor + 1, speed);
    }

    void M5Stack4EncoderMotor::set_motor_soft_start_stop(Motor motor, bool enable)
    {
      if (enable)
      {
        this->reg(REG_SOFT_START_STOP) |= (1 << motor);
      }
      else
      {
        this->reg(REG_SOFT_START_STOP) &= ~(1 << motor);
      }
      ESP_LOGD(TAG, "Set motor %d soft start/stop to %s", motor + 1, enable ? "enabled" : "disabled");
    }

    float M5Stack4EncoderMotor::get_current()
    {
      if (this->current_sensor_ != nullptr && this->current_sensor_->has_state())
      {
        return this->current_sensor_->get_state();
      }
      return this->read_current().value_or(NAN);
    }

    optional<float> M5Stack4EncoderMotor::read_current()
    {
      float current;
      auto err = this->read_register(REG_CURRENT, (uint8_t *)&current, sizeof(current));
      if (err != i2c::ERROR_OK)
        return {};
      return roundf(1000.0 * current) / 1000.0;
    }

    optional<float> M5Stack4EncoderMotor::read_voltage()
    {
      uint8_t data[2];
      auto err = this->read_register(REG_VOLTAGE, data, sizeof(data));
      if (err != i2c::ERROR_OK)
        return {};
      // From protocol PDF: ((ADC Value 12bits-L) + (ADC Value 12bits-H)*256)/255*3.3/0.16
      uint16_t raw = data[0] | (data[1] << 8);
      float voltage = raw / 4095.0 * 3.3 / 0.16;
      return roundf(1000.0 * voltage) / 1000.0;
    }
  }
}
