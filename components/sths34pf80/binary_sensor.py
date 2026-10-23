import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, i2c, number
from esphome.components.zeromouse_air import PresenceSampler
from esphome.const import DEVICE_CLASS_OCCUPANCY
from esphome.types import ConfigType

AUTO_LOAD = ["number"]
DEPENDENCIES = ["esp32", "i2c", "zeromouse_air"]

CONF_INTERRUPT_PIN = "interrupt_pin"
CONF_MOTION_THRESHOLD = "motion_threshold"
CONF_PRESENCE_THRESHOLD = "presence_threshold"

sths34pf80_ns = cg.esphome_ns.namespace("sths34pf80")
Sths34pf80 = sths34pf80_ns.class_(
    "Sths34pf80",
    binary_sensor.BinarySensor,
    cg.Component,
    i2c.I2CDevice,
    PresenceSampler,
)

_THRESHOLD = cv.Any(cv.uint16_t, cv.use_id(number.Number))

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(Sths34pf80, device_class=DEVICE_CLASS_OCCUPANCY)
    .extend(
        {
            cv.Required(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_MOTION_THRESHOLD): _THRESHOLD,
            cv.Optional(CONF_PRESENCE_THRESHOLD): _THRESHOLD,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x5A))
)


async def to_code(config: ConfigType) -> None:
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
    cg.add(var.set_interrupt_pin(pin))

    for key in (CONF_MOTION_THRESHOLD, CONF_PRESENCE_THRESHOLD):
        if (value := config.get(key)) is None:
            continue
        if isinstance(value, int):
            cg.add(getattr(var, f"set_{key}")(value))
        else:
            cg.add(getattr(var, f"set_{key}_number")(await cg.get_variable(value)))
