import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import time
from esphome.components.aws_iot import AwsIotClient
from esphome.components.burst_camera import BurstCamera
from esphome.const import CONF_ID, CONF_TIME_ID
from esphome.types import ConfigType

AUTO_LOAD = ["json"]
DEPENDENCIES = ["esp32", "aws_iot", "burst_camera", "time"]

CONF_THING_NAME = "thing_name"
CONF_AWS_IOT_ID = "aws_iot_id"
CONF_OWNER_ID = "owner_id"
CONF_IDENTITY_ID = "identity_id"
CONF_EVENT_COUNT = "event_count"
CONF_SW_VERSION = "sw_version"
CONF_ON_CLASSIFICATION = "on_classification"
CONF_CAMERA_ID = "camera_id"

zeromouse_cloud_ns = cg.esphome_ns.namespace("zeromouse_cloud")
ZeroMouseCloud = zeromouse_cloud_ns.class_("ZeroMouseCloud", cg.Component)
PublishTestEventAction = zeromouse_cloud_ns.class_(
    "PublishTestEventAction", automation.Action
)

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
        cv.Optional(CONF_ON_CLASSIFICATION): automation.validate_automation(
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
