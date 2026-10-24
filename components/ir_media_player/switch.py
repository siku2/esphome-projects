import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_TYPE

from . import CONF_MEDIA_PLAYER_ID, IrMediaPlayer, ir_media_player_ns

IrStateSwitch = ir_media_player_ns.class_("IrStateSwitch", switch.Switch, cg.Component)
StateType = ir_media_player_ns.enum("StateType", is_class=True)

TYPES = {
    "power": StateType.POWER,
    "mute": StateType.MUTE,
}

CONFIG_SCHEMA = (
    switch.switch_schema(IrStateSwitch)
    .extend(
        {
            cv.GenerateID(CONF_MEDIA_PLAYER_ID): cv.use_id(IrMediaPlayer),
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    player = await cg.get_variable(config[CONF_MEDIA_PLAYER_ID])
    cg.add(var.set_media_player(player))
    cg.add(var.set_type(config[CONF_TYPE]))
