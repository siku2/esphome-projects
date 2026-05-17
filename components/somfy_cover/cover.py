import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import cover
from esphome.components.cc1101 import (
    CONF_CC1101_ID,
    Cc1101,
)
from esphome.const import CONF_CLOSE_DURATION, CONF_ID, CONF_OPEN_DURATION

DEPENDENCIES = ["cc1101"]

CONF_REMOTE_CODE = "remote_code"

somfy_cover_ns = cg.esphome_ns.namespace("somfy_cover")

SomfyCover = somfy_cover_ns.class_("SomfyCover", cover.Cover, cg.Component)
SomfyCoverProgramAction = somfy_cover_ns.class_(
    "SomfyCoverProgramAction", automation.Action
)
SomfyCoverResetRollingCodeAction = somfy_cover_ns.class_(
    "SomfyCoverResetRollingCodeAction", automation.Action
)

CONFIG_SCHEMA = (
    cover.cover_schema(SomfyCover)
    .extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(SomfyCover),
            cv.GenerateID(CONF_CC1101_ID): cv.use_id(Cc1101),
            cv.Required(CONF_REMOTE_CODE): cv.uint32_t,
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    cg.add(var.set_cover_id(str(config[CONF_ID])))
    cg.add(var.set_remote_code(config[CONF_REMOTE_CODE]))
    cg.add(var.set_cc1101(await cg.get_variable(config[CONF_CC1101_ID])))
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))


@automation.register_action(
    "cover.somfy_cover.program",
    SomfyCoverProgramAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(SomfyCover),
        }
    ),
)
async def program_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "cover.somfy_cover.reset_rolling_code",
    SomfyCoverResetRollingCodeAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(SomfyCover),
        }
    ),
)
async def reset_rolling_code_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)
