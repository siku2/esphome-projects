import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_CHANNEL, CONF_ID, CONF_MAX_VOLTAGE

from . import CONF_AXP2101_ID, AXP2101Component, axp2101_ns

CONF_MIN_VOLTAGE = "min_voltage"

DEPENDENCIES = ["axp2101"]

AXP2101Output = axp2101_ns.class_("AXP2101Output", output.FloatOutput, cg.Component)
LdoChannel = axp2101_ns.enum("LdoChannel", is_class=True)

# name: (min mV, step mV, max step)
CHANNELS = {
    "ALDO1": (500, 100, 30),
    "ALDO2": (500, 100, 30),
    "ALDO3": (500, 100, 30),
    "ALDO4": (500, 100, 30),
    "BLDO1": (500, 100, 30),
    "BLDO2": (500, 100, 30),
    "CPUSLDO": (500, 50, 18),
    "DLDO1": (500, 100, 29),
    "DLDO2": (500, 100, 29),
}


def validate_voltages(config):
    min_mv, step_mv, max_step = CHANNELS[config[CONF_CHANNEL]]
    max_mv = min_mv + step_mv * max_step
    for key, default in ((CONF_MIN_VOLTAGE, min_mv), (CONF_MAX_VOLTAGE, max_mv)):
        value = round(config.get(key, default / 1000) * 1000)
        if not min_mv <= value <= max_mv or (value - min_mv) % step_mv != 0:
            raise cv.Invalid(
                f"{key} must be between {min_mv} mV and {max_mv} mV in {step_mv} mV steps",
                path=[key],
            )
        config[key] = value
    if config[CONF_MIN_VOLTAGE] >= config[CONF_MAX_VOLTAGE]:
        raise cv.Invalid(f"{CONF_MIN_VOLTAGE} must be below {CONF_MAX_VOLTAGE}")
    return config


CONFIG_SCHEMA = cv.All(
    output.FLOAT_OUTPUT_SCHEMA.extend(
        {
            cv.Required(CONF_ID): cv.declare_id(AXP2101Output),
            cv.GenerateID(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
            cv.Required(CONF_CHANNEL): cv.one_of(*CHANNELS, upper=True),
            cv.Optional(CONF_MIN_VOLTAGE): cv.voltage,
            cv.Optional(CONF_MAX_VOLTAGE): cv.voltage,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_voltages,
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AXP2101_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)
    cg.add(var.set_parent(hub))
    cg.add(var.set_channel(getattr(LdoChannel, config[CONF_CHANNEL])))
    cg.add(var.set_min_voltage(config[CONF_MIN_VOLTAGE]))
    cg.add(var.set_max_voltage(config[CONF_MAX_VOLTAGE]))
