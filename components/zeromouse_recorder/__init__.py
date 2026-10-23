import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import switch
from esphome.components.burst_camera import BurstCamera
from esphome.components.s3 import S3Client
from esphome.components.zeromouse import ZeroMouse
from esphome.const import CONF_ID
from esphome.types import ConfigType

AUTO_LOAD = ["json"]
DEPENDENCIES = ["burst_camera", "esp32", "s3", "time", "zeromouse"]

CONF_ENABLED = "enabled"
CONF_S3_ID = "s3_id"
CONF_CONTROLLER_ID = "controller_id"
CONF_CAMERA_ID = "camera_id"
CONF_SECONDARY_JPEG_QUALITY = "secondary_jpeg_quality"
CONF_MAX_PENDING = "max_pending"
CONF_MAX_PENDING_BYTES = "max_pending_bytes"
CONF_BOARD = "board"
CONF_ON_RECORDED = "on_recorded"

zeromouse_recorder_ns = cg.esphome_ns.namespace("zeromouse_recorder")
ZeroMouseRecorder = zeromouse_recorder_ns.class_("ZeroMouseRecorder", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZeroMouseRecorder),
        cv.Required(CONF_S3_ID): cv.use_id(S3Client),
        cv.Required(CONF_CONTROLLER_ID): cv.use_id(ZeroMouse),
        cv.Required(CONF_CAMERA_ID): cv.use_id(BurstCamera),
        cv.Optional(CONF_ENABLED): cv.use_id(switch.Switch),
        cv.Optional(CONF_SECONDARY_JPEG_QUALITY, default=80): cv.int_range(
            min=1, max=100
        ),
        cv.Optional(CONF_MAX_PENDING, default=3): cv.int_range(min=1, max=16),
        cv.Optional(CONF_MAX_PENDING_BYTES, default=3000000): cv.int_range(min=1),
        cv.Required(CONF_BOARD): cv.All(cv.string_strict, cv.Length(min=1)),
        cv.Optional(CONF_ON_RECORDED): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_s3(await cg.get_variable(config[CONF_S3_ID])))
    cg.add(var.set_controller(await cg.get_variable(config[CONF_CONTROLLER_ID])))
    cg.add(var.set_camera(await cg.get_variable(config[CONF_CAMERA_ID])))
    if enabled := config.get(CONF_ENABLED):
        cg.add(var.set_enabled(await cg.get_variable(enabled)))
    cg.add(var.set_secondary_jpeg_quality(config[CONF_SECONDARY_JPEG_QUALITY]))
    cg.add(var.set_max_pending(config[CONF_MAX_PENDING]))
    cg.add(var.set_max_pending_bytes(config[CONF_MAX_PENDING_BYTES]))
    cg.add(var.set_board(config[CONF_BOARD]))

    if action := config.get(CONF_ON_RECORDED):
        await automation.build_automation(
            var.get_recorded_trigger(),
            [
                (cg.std_string, "path"),
                (cg.uint32, "event_id"),
                (cg.std_string, "class_name"),
            ],
            action,
        )
