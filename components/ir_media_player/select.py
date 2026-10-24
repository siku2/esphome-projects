import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_NAME, CONF_OPTIONS

from . import (
    CONF_MEDIA_PLAYER_ID,
    NAMED_CODE_SCHEMA,
    IrMediaPlayer,
    code_args,
    ir_media_player_ns,
)

IrSceneSelect = ir_media_player_ns.class_("IrSceneSelect", select.Select, cg.Component)


def _validate_unique_names(options):
    names = [option[CONF_NAME] for option in options]
    if len(names) != len(set(names)):
        raise cv.Invalid("Option names must be unique")
    return options


CONFIG_SCHEMA = (
    select.select_schema(IrSceneSelect)
    .extend(
        {
            cv.GenerateID(CONF_MEDIA_PLAYER_ID): cv.use_id(IrMediaPlayer),
            cv.Required(CONF_OPTIONS): cv.All(
                cv.ensure_list(NAMED_CODE_SCHEMA),
                cv.Length(min=1),
                _validate_unique_names,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    options = config[CONF_OPTIONS]
    var = await select.new_select(
        config, options=[option[CONF_NAME] for option in options]
    )
    await cg.register_component(var, config)

    player = await cg.get_variable(config[CONF_MEDIA_PLAYER_ID])
    cg.add(var.set_media_player(player))
    cg.add(player.set_scene_select(var))
    for option in options:
        cg.add(var.add_scene(*code_args(option)))
