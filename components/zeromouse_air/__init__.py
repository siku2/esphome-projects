import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.components.esp32 import include_builtin_idf_component
from esphome.const import CONF_ID, CONF_NUMBER, CONF_TYPE
from esphome.types import ConfigType

DEPENDENCIES = ["esp32"]

CONF_ADC_PIN = "adc_pin"
CONF_LED_PIN = "led_pin"
CONF_CALIBRATION_TOLERANCE = "calibration_tolerance"
CONF_CALIBRATION_MAX = "calibration_max"

zeromouse_air_ns = cg.esphome_ns.namespace("zeromouse_air")
AirSensor = zeromouse_air_ns.class_("AirSensor", cg.Component)
PresenceSampler = zeromouse_air_ns.class_("PresenceSampler")
Pt2046b = zeromouse_air_ns.class_("Pt2046b", AirSensor)
Vcnl4040 = zeromouse_air_ns.class_("Vcnl4040", AirSensor, i2c.I2CDevice)

# GPIO1 to GPIO10 are the ADC1 channels of the ESP32-S3.
_ADC1_PINS = range(1, 11)


def _adc1_pin(value: object) -> int:
    pin = pins.internal_gpio_input_pin_number(value)
    if pin not in _ADC1_PINS:
        raise cv.Invalid("Only GPIO1 to GPIO10 are ADC1 pins on the ESP32-S3")
    return pin


PT2046B_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(Pt2046b),
        cv.Required(CONF_ADC_PIN): _adc1_pin,
        cv.Required(CONF_LED_PIN): pins.internal_gpio_output_pin_schema,
        cv.Optional(CONF_CALIBRATION_TOLERANCE, default=20): cv.uint16_t,
        cv.Optional(CONF_CALIBRATION_MAX, default=2000): cv.uint16_t,
    }
)

VCNL4040_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Vcnl4040),
            cv.Optional(CONF_CALIBRATION_TOLERANCE, default=5): cv.uint16_t,
            cv.Optional(CONF_CALIBRATION_MAX, default=650): cv.uint16_t,
        }
    )
    .extend(i2c.i2c_device_schema(0x60))
)

CONFIG_SCHEMA = cv.typed_schema(
    {"pt2046b": PT2046B_SCHEMA, "vcnl4040": VCNL4040_SCHEMA}, lower=True
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_calibration_tolerance(config[CONF_CALIBRATION_TOLERANCE]))
    cg.add(var.set_calibration_max(config[CONF_CALIBRATION_MAX]))

    if config[CONF_TYPE] == "vcnl4040":
        cg.add_define("USE_ZEROMOUSE_AIR_VCNL4040")
        await i2c.register_i2c_device(var, config)

    if config[CONF_TYPE] == "pt2046b":
        cg.add_define("USE_ZEROMOUSE_AIR_PT2046B")
        include_builtin_idf_component("esp_adc")
        cg.add(var.set_adc_pin(config[CONF_ADC_PIN]))
        cg.add(var.set_led_pin(config[CONF_LED_PIN][CONF_NUMBER]))
