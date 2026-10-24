import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import media_player, remote_base

from . import CODE_SCHEMA, IrMediaPlayer, code_args

CONF_ECHO_WINDOW = "echo_window"
CONF_MIN_GAP = "min_gap"
CONF_POWER_TOGGLE = "power_toggle"
CONF_MUTE_TOGGLE = "mute_toggle"
CONF_VOLUME_UP = "volume_up"
CONF_VOLUME_DOWN = "volume_down"


def _validate_volume(config):
    if (CONF_VOLUME_UP in config) != (CONF_VOLUME_DOWN in config):
        raise cv.Invalid(
            f"{CONF_VOLUME_UP} and {CONF_VOLUME_DOWN} must be set together"
        )
    return config


CONFIG_SCHEMA = cv.All(
    media_player.media_player_schema(IrMediaPlayer)
    .extend(remote_base.REMOTE_LISTENER_SCHEMA)
    .extend(remote_base.REMOTE_TRANSMITTABLE_SCHEMA)
    .extend(
        {
            cv.Optional(
                CONF_ECHO_WINDOW, default="1s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_MIN_GAP, default="200ms"
            ): cv.positive_time_period_milliseconds,
            cv.Required(CONF_POWER_TOGGLE): CODE_SCHEMA,
            cv.Optional(CONF_MUTE_TOGGLE): CODE_SCHEMA,
            cv.Optional(CONF_VOLUME_UP): CODE_SCHEMA,
            cv.Optional(CONF_VOLUME_DOWN): CODE_SCHEMA,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_volume,
)


async def to_code(config):
    var = await media_player.new_media_player(config)
    await cg.register_component(var, config)
    await remote_base.register_listener(var, config)
    await remote_base.register_transmittable(var, config)

    cg.add(var.set_echo_window(config[CONF_ECHO_WINDOW].total_milliseconds))
    cg.add(var.set_min_gap(config[CONF_MIN_GAP].total_milliseconds))
    cg.add(var.set_power_toggle(*code_args(config[CONF_POWER_TOGGLE])))
    if mute := config.get(CONF_MUTE_TOGGLE):
        cg.add(var.set_mute_toggle(*code_args(mute)))
    if volume_up := config.get(CONF_VOLUME_UP):
        cg.add(var.set_volume_up(*code_args(volume_up)))
        cg.add(var.set_volume_down(*code_args(config[CONF_VOLUME_DOWN])))
