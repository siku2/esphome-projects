import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_AXP2101_ID, AXP2101Component

DEPENDENCIES = ["axp2101"]

TYPES = {
    "current_limit": "set_current_limit_binary_sensor",
    "vbus_good": "set_vbus_good_binary_sensor",
}

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC
).extend(
    {
        cv.GenerateID(CONF_AXP2101_ID): cv.use_id(AXP2101Component),
        cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AXP2101_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(getattr(hub, TYPES[config[CONF_TYPE]])(var))
