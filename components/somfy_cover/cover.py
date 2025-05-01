import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.components.elechouse_cc1101 import (
    CONF_ELECHOUSE_CC1101_ID,
    ElechouseCc1101,
)
from esphome.const import CONF_CLOSE_DURATION, CONF_ID, CONF_OPEN_DURATION

DEPENDENCIES = ["elechouse_cc1101"]

CONF_COVER_ID = "cover_id"
CONF_REMOTE_CODE = "remote_code"

somfy_cover_ns = cg.esphome_ns.namespace("somfy_cover")

SomfyCover = somfy_cover_ns.class_("SomfyCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(SomfyCover),
        cv.GenerateID(CONF_ELECHOUSE_CC1101_ID): cv.use_id(ElechouseCc1101),
        cv.Optional(CONF_COVER_ID): cv.string,
        cv.Required(CONF_REMOTE_CODE): cv.uint32_t,
        cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
        cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
    },
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Undeclared internal dependency of Somfy_Remote_Lib
    cg.add_library("EEPROM", None)
    cg.add_library("Somfy_Remote_Lib", "0.4.1")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    var.set_cover_id(config[CONF_COVER_ID] or str(config[CONF_ID]))
    var.set_remote_code(config[CONF_REMOTE_CODE])
    remote = await cg.get_variable(config[CONF_ELECHOUSE_CC1101_ID])
    cg.add(var.set_cc1101(remote))

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
