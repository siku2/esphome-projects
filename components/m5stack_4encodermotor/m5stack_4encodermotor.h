#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome
{
  namespace m5stack_4encodermotor
  {
    enum Motor
    {
      MOTOR_1 = 0,
      MOTOR_2 = 1,
      MOTOR_3 = 2,
      MOTOR_4 = 3,
    };

    enum Mode
    {
      MODE_NORMAL = 0,
    };

    class M5Stack4EncoderMotor : public PollingComponent, public i2c::I2CDevice
    {
    public:
      M5Stack4EncoderMotor() = default;

      void setup() override;
      void dump_config() override;
      void update() override;
      void loop() override;

      void set_current_sensor(sensor::Sensor *sensor) { this->current_sensor_ = sensor; }
      void set_voltage_sensor(sensor::Sensor *sensor) { this->voltage_sensor_ = sensor; }

      void set_motor_mode(Motor motor, Mode mode);
      void set_motor_speed(Motor motor, int8_t speed);
      void set_motor_soft_start_stop(Motor motor, bool enable);

    protected:
      uint8_t fw_version_;
      uint8_t bootloader_version_;
      sensor::Sensor *current_sensor_{nullptr};
      sensor::Sensor *voltage_sensor_{nullptr};

      optional<float> read_current();
      optional<float> read_voltage();
    };
  }
}
