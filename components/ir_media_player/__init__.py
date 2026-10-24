import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_COMMAND, CONF_NAME

CODEOWNERS = ["@siku2"]
AUTO_LOAD = ["media_player", "remote_base", "select", "switch"]

CONF_MEDIA_PLAYER_ID = "media_player_id"
CONF_COMMAND_REPEATS = "command_repeats"

ir_media_player_ns = cg.esphome_ns.namespace("ir_media_player")
IrMediaPlayer = ir_media_player_ns.class_("IrMediaPlayer", cg.Component)

CODE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ADDRESS): cv.hex_uint16_t,
        cv.Required(CONF_COMMAND): cv.hex_uint16_t,
        cv.Optional(CONF_COMMAND_REPEATS, default=1): cv.int_range(min=1, max=65535),
    }
)

NAMED_CODE_SCHEMA = CODE_SCHEMA.extend({cv.Required(CONF_NAME): cv.string})


def code_args(config):
    return (config[CONF_ADDRESS], config[CONF_COMMAND], config[CONF_COMMAND_REPEATS])
