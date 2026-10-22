import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import number, select, switch, time
from esphome.components.aws_iot import AwsIotClient
from esphome.components.burst_camera import BurstCamera
from esphome.const import CONF_ID, CONF_TIME_ID
from esphome.types import ConfigType

AUTO_LOAD = ["json", "number", "select", "switch"]
DEPENDENCIES = ["esp32", "aws_iot", "burst_camera", "time"]

CONF_THING_NAME = "thing_name"
CONF_AWS_IOT_ID = "aws_iot_id"
CONF_OWNER_ID = "owner_id"
CONF_IDENTITY_ID = "identity_id"
CONF_EVENT_COUNT = "event_count"
CONF_SW_VERSION = "sw_version"
CONF_ON_CLASSIFICATION = "on_classification"
CONF_CAMERA_ID = "camera_id"
CONF_ACCEPT_SETTINGS = "accept_settings"
CONF_UPLOAD_EVENTS = "upload_events"
CONF_SETTINGS = "settings"
CONF_ON_SETTING_CHANGE = "on_setting_change"

zeromouse_cloud_ns = cg.esphome_ns.namespace("zeromouse_cloud")
ZeroMouseCloud = zeromouse_cloud_ns.class_("ZeroMouseCloud", cg.Component)
PublishTestEventAction = zeromouse_cloud_ns.class_(
    "PublishTestEventAction", automation.Action
)

Setting = zeromouse_cloud_ns.enum("Setting", is_class=True)

# Config key -> (entity class, bind method)
_NUMBER = (number.Number, "bind_number")
_SWITCH = (switch.Switch, "bind_switch")
_SELECT = (select.Select, "bind_select")
SETTINGS = {
    "clean_event_time": _NUMBER,
    "undecidable_mode": _SELECT,
    "finale_interval_max_count": _NUMBER,
    "finale_interval_delta": _NUMBER,
    "upload_empty": _SWITCH,
    "log_enabled": _SWITCH,
    "auto_exp_level": _NUMBER,
    "gain_ceiling": _NUMBER,
    "brightness": _NUMBER,
    "auto_gain_ctrl_min": _NUMBER,
    "auto_gain_ctrl_max": _NUMBER,
    "auto_gain_stbl_min": _NUMBER,
    "auto_gain_stbl_max": _NUMBER,
    "cam_shots_before_t0": _NUMBER,
    "ir_range_ignore_event": _NUMBER,
    "ir_prey_event_time": _NUMBER,
    "ir_median_count": _NUMBER,
    "ir_ambient": _NUMBER,
    "ir_slope_thresh_value": _NUMBER,
    "ir_motion_thresh": _NUMBER,
    "ir_presence_thresh": _NUMBER,
    "block_enabled": _SWITCH,
    "response_timeout": _NUMBER,
    "last_ref_event_max_count": _NUMBER,
    "last_ref_event_max_time_sec": _NUMBER,
}

_VERSION = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def sw_version(value: object) -> tuple[int, int, int]:
    value = cv.string_strict(value)
    if not (match := _VERSION.match(value)):
        raise cv.Invalid("Expected a version like 0.17.12")
    return tuple(cv.uint8_t(part) for part in match.groups())


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZeroMouseCloud),
        cv.Required(CONF_AWS_IOT_ID): cv.use_id(AwsIotClient),
        cv.Required(CONF_THING_NAME): cv.All(cv.string_strict, cv.Length(min=1)),
        cv.Required(CONF_OWNER_ID): cv.All(cv.string_strict, cv.Length(min=1)),
        cv.Required(CONF_IDENTITY_ID): cv.All(cv.string_strict, cv.Length(min=1)),
        cv.Optional(CONF_EVENT_COUNT, default=0): cv.uint32_t,
        cv.Optional(CONF_SW_VERSION, default="0.17.12"): sw_version,
        cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Optional(CONF_ACCEPT_SETTINGS): cv.use_id(switch.Switch),
        cv.Optional(CONF_UPLOAD_EVENTS): cv.use_id(switch.Switch),
        cv.Optional(CONF_SETTINGS): cv.Schema(
            {cv.Optional(key): cv.use_id(kind[0]) for key, kind in SETTINGS.items()}
        ),
        cv.Optional(CONF_ON_CLASSIFICATION): automation.validate_automation(
            single=True
        ),
        cv.Optional(CONF_ON_SETTING_CHANGE): automation.validate_automation(
            single=True
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_aws_iot(await cg.get_variable(config[CONF_AWS_IOT_ID])))
    cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))
    cg.add(var.set_thing_name(config[CONF_THING_NAME]))
    cg.add(var.set_owner_id(config[CONF_OWNER_ID]))
    cg.add(var.set_identity_id(config[CONF_IDENTITY_ID]))
    cg.add(var.set_event_count(config[CONF_EVENT_COUNT]))
    cg.add(var.set_sw_version(*config[CONF_SW_VERSION]))

    if accept_id := config.get(CONF_ACCEPT_SETTINGS):
        cg.add(var.set_accept_settings(await cg.get_variable(accept_id)))

    if upload_id := config.get(CONF_UPLOAD_EVENTS):
        cg.add(var.set_upload_events(await cg.get_variable(upload_id)))

    for key, entity_id in config.get(CONF_SETTINGS, {}).items():
        bind = getattr(var, SETTINGS[key][1])
        cg.add(bind(getattr(Setting, key.upper()), await cg.get_variable(entity_id)))

    if action := config.get(CONF_ON_SETTING_CHANGE):
        await automation.build_automation(
            var.get_setting_change_trigger(),
            [
                (cg.std_string, "key"),
                (cg.std_string, "previous"),
                (cg.std_string, "value"),
                (cg.bool_, "applied"),
            ],
            action,
        )

    if action := config.get(CONF_ON_CLASSIFICATION):
        await automation.build_automation(
            var.get_classification_trigger(),
            [
                (cg.std_string, "class_name"),
                (cg.int_, "class_index"),
                (cg.bool_, "block"),
            ],
            action,
        )


PUBLISH_TEST_EVENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(ZeroMouseCloud),
        cv.Required(CONF_CAMERA_ID): cv.use_id(BurstCamera),
    }
)


@automation.register_action(
    "zeromouse_cloud.publish_test_event",
    PublishTestEventAction,
    PUBLISH_TEST_EVENT_SCHEMA,
    synchronous=True,
)
async def publish_test_event_to_code(
    config: ConfigType,
    action_id: cg.ID,
    template_arg: cg.TemplateArguments,
    args: list,
) -> cg.MockObj:
    parent = await cg.get_variable(config[CONF_ID])
    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    return cg.new_Pvariable(action_id, template_arg, parent, camera)
