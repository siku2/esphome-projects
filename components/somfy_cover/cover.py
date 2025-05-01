import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import (
    CONF_ID,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_RESTORE_MODE,
)
from esphome.elechouse_cc1101 import CONF_ELECHOUSE_CC1101_ID, ElechouseCc1101

DEPENDENCIES = ["elechouse_cc1101"]

CONF_REMOTE_CODE = "remote_code"

somfy_cover_ns = cg.esphome_ns.namespace("somfy_cover")

SomfyCover = somfy_cover_ns.class_("SomfyCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cv.All(
    cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(SomfyCover),
            cv.GenerateID(CONF_ELECHOUSE_CC1101_ID): cv.use_id(ElechouseCc1101),
            cv.Required(CONF_REMOTE_CODE): cv.uint32_t,
        },
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    # Undeclared internal dependency of Somfy_Remote_Lib
    cg.add_library("EEPROM", None)
    cg.add_library("Somfy_Remote_Lib", "0.4.1")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    paren = await cg.get_variable(config[CONF_ELECHOUSE_CC1101_ID])
    cg.add(var.set_cc1101(paren))
