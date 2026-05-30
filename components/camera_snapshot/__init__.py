import logging
from typing import Any

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import esp32
from esphome.const import (
    CONF_ID,
)
from esphome.types import ConfigType

_LOGGER = logging.getLogger(__name__)

AUTO_LOAD = ["camera"]

CONF_BEFORE_SNAPSHOT = "before_snapshot"
CONF_AFTER_SNAPSHOT = "after_snapshot"
CONF_ON_SNAPSHOT = "on_snapshot"
CONF_ROTATE = "rotate"
CONF_DRAIN_FRAME_BUFFER_COUNT = "drain_frame_buffer_count"

camera_ns = cg.esphome_ns.namespace("camera")
snapshot_ns = camera_ns.namespace("snapshot")
Snapshotter = snapshot_ns.class_("Snapshotter", cg.PollingComponent)
Snapshot = snapshot_ns.class_("Snapshot")
SnapshotConstRef = Snapshot.operator("const").operator("ref")
SnapshotListener = snapshot_ns.class_("SnapshotListener")
jpeg_rotate_t = cg.global_ns.enum("jpeg_rotate_t")


_ROTATE_MAP = {
    0: jpeg_rotate_t.JPEG_ROTATE_0D,
    90: jpeg_rotate_t.JPEG_ROTATE_90D,
    180: jpeg_rotate_t.JPEG_ROTATE_180D,
    270: jpeg_rotate_t.JPEG_ROTATE_270D,
}


def _validate_rotate(value: Any) -> cg.MockObj:
    value = cv.string(value)
    value = value.removesuffix("°")
    return cv.enum(_ROTATE_MAP, int=True)(value)


CONFIG_SCHEMA = cv.polling_component_schema("60s").extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(Snapshotter),
        cv.Optional(CONF_ROTATE, default="0°"): _validate_rotate,
        cv.Optional(CONF_DRAIN_FRAME_BUFFER_COUNT, default=0): cv.uint8_t,
        cv.Optional(CONF_BEFORE_SNAPSHOT): automation.validate_automation(single=True),
        cv.Optional(CONF_AFTER_SNAPSHOT): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_SNAPSHOT): automation.validate_automation(single=True),
    }
)


async def to_code(config: ConfigType) -> None:
    esp32.add_idf_component(name="espressif/esp_new_jpeg", ref="1.0.1")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_rotate(config[CONF_ROTATE]))
    cg.add(var.set_drain_frame_buffer_count(config[CONF_DRAIN_FRAME_BUFFER_COUNT]))

    if action := config.get(CONF_BEFORE_SNAPSHOT):
        await automation.build_automation(var.get_pre_snapshot_trigger(), [], action)

    if action := config.get(CONF_AFTER_SNAPSHOT):
        await automation.build_automation(var.get_post_snapshot_trigger(), [], action)

    if action := config.get(CONF_ON_SNAPSHOT):
        await automation.build_automation(
            var.get_on_snapshot_trigger(),
            [(SnapshotConstRef, "x")],
            action,
        )
