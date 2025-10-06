import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.components.m5stack_4encodermotor import (
    MOTOR_OPTIONS,
    M5Stack4EncoderMotor,
    m5stack_4encodermotor_ns,
)
from esphome.const import CONF_CLOSE_DURATION, CONF_OPEN_DURATION

CONF_M5STACK_4ENCODERMOTOR_ID = "m5stack_4encodermotor_id"
CONF_MOTOR = "motor"
CONF_SOFT_START_STOP = "soft_start_stop"
CONF_MIN_CURRENT = "min_current"
CONF_ACCELERATION_WAIT_TIME = "acceleration_wait_time"
CONF_POSITIVE_IS_UP = "positive_is_up"

DEPENDENCIES = ["m5stack_4encodermotor"]

M5Stack4EncoderMotorCover = m5stack_4encodermotor_ns.class_(
    "M5Stack4EncoderMotorCover", cover.Cover, cg.Component
)

CONFIG_SCHEMA = (
    cover.cover_schema(M5Stack4EncoderMotorCover)
    .extend(
        {
            cv.GenerateID(CONF_M5STACK_4ENCODERMOTOR_ID): cv.use_id(
                M5Stack4EncoderMotor
            ),
            cv.Required(CONF_MOTOR): cv.enum(MOTOR_OPTIONS),
            cv.Required(CONF_MIN_CURRENT): cv.current,
            cv.Optional(CONF_SOFT_START_STOP, default=False): cv.boolean,
            cv.Optional(CONF_POSITIVE_IS_UP, default=True): cv.boolean,
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_ACCELERATION_WAIT_TIME, default=0
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    cg.add(var.set_parent(await cg.get_variable(config[CONF_M5STACK_4ENCODERMOTOR_ID])))
    cg.add(var.set_motor(config[CONF_MOTOR]))
    cg.add(var.set_min_current(config[CONF_MIN_CURRENT]))
    cg.add(var.set_soft_start_stop(config[CONF_SOFT_START_STOP]))
    cg.add(var.set_positive_is_up(config[CONF_POSITIVE_IS_UP]))
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_acceleration_wait_time(config[CONF_ACCELERATION_WAIT_TIME]))
