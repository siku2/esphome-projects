from typing import Any

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import i2c
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
from esphome.const import (
    CONF_BRIGHTNESS,
    CONF_CONTRAST,
    CONF_DATA_PINS,
    CONF_FREQUENCY,
    CONF_I2C_ID,
    CONF_ID,
    CONF_PIN,
    CONF_RESET_PIN,
    CONF_RESOLUTION,
    CONF_SCL,
    CONF_SDA,
    CONF_UPDATE_INTERVAL,
    CONF_VSYNC_PIN,
)
from esphome.core.entity_helpers import setup_entity
from esphome.types import ConfigType

AUTO_LOAD = ["camera", "task_util"]
DEPENDENCIES = ["esp32", "psram"]

CONF_HREF_PIN = "href_pin"
CONF_PIXEL_CLOCK_PIN = "pixel_clock_pin"
CONF_EXTERNAL_CLOCK = "external_clock"
CONF_I2C_PINS = "i2c_pins"
CONF_POWER_DOWN_PIN = "power_down_pin"
CONF_ROTATE = "rotate"
CONF_DRAIN_FRAME_BUFFER_COUNT = "drain_frame_buffer_count"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_HORIZONTAL_MIRROR = "horizontal_mirror"
CONF_SATURATION = "saturation"
CONF_AEC_MODE = "aec_mode"
CONF_AEC_VALUE = "aec_value"
CONF_AGC_MODE = "agc_mode"
CONF_AGC_VALUE = "agc_value"
CONF_WB_MODE = "wb_mode"
CONF_BEFORE_CAPTURE = "before_capture"
CONF_AFTER_CAPTURE = "after_capture"
CONF_ON_CYCLE_END = "on_cycle_end"

still_camera_ns = cg.esphome_ns.namespace("still_camera")
Camera = cg.esphome_ns.namespace("camera").class_("Camera", cg.EntityBase, cg.Component)
StillCamera = still_camera_ns.class_("StillCamera", Camera)
FrameProcessor = still_camera_ns.class_("FrameProcessor")
Rotation = still_camera_ns.enum("Rotation", is_class=True)
CaptureAction = still_camera_ns.class_("CaptureAction", automation.Action)

ROTATIONS = {
    0: Rotation.ROTATE_0,
    90: Rotation.ROTATE_90,
    180: Rotation.ROTATE_180,
    270: Rotation.ROTATE_270,
}

_FRAME_SIZES = {
    "160X120": "QQVGA",
    "QQVGA": "QQVGA",
    "176X144": "QCIF",
    "QCIF": "QCIF",
    "240X176": "HQVGA",
    "HQVGA": "HQVGA",
    "320X240": "QVGA",
    "QVGA": "QVGA",
    "400X296": "CIF",
    "CIF": "CIF",
    "640X480": "VGA",
    "VGA": "VGA",
    "800X600": "SVGA",
    "SVGA": "SVGA",
    "1024X768": "XGA",
    "XGA": "XGA",
    "1280X1024": "SXGA",
    "SXGA": "SXGA",
}
FRAME_SIZES = {
    name: getattr(cg.global_ns, f"FRAMESIZE_{size}")
    for name, size in _FRAME_SIZES.items()
}

CONTROL_MODES = {"MANUAL": False, "AUTO": True}
WB_MODES = {"AUTO": 0, "SUNNY": 1, "CLOUDY": 2, "OFFICE": 3, "HOME": 4}

camera_range_param = cv.int_range(min=-2, max=2)


def _validate_rotate(value: Any) -> cg.MockObj:
    value = cv.string(value).removesuffix("°")
    return cv.enum(ROTATIONS, int=True)(value)


CONFIG_SCHEMA = cv.All(
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(StillCamera),
            cv.Required(CONF_DATA_PINS): cv.All(
                [pins.internal_gpio_input_pin_number], cv.Length(min=8, max=8)
            ),
            cv.Required(CONF_VSYNC_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_HREF_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_PIXEL_CLOCK_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_EXTERNAL_CLOCK): cv.Schema(
                {
                    cv.Required(CONF_PIN): pins.internal_gpio_input_pin_number,
                    cv.Optional(CONF_FREQUENCY, default="20MHz"): cv.All(
                        cv.frequency, cv.float_range(min=8e6, max=20e6)
                    ),
                }
            ),
            cv.Optional(CONF_I2C_PINS): cv.Schema(
                {
                    cv.Required(CONF_SDA): pins.internal_gpio_output_pin_number,
                    cv.Required(CONF_SCL): pins.internal_gpio_output_pin_number,
                }
            ),
            cv.Optional(CONF_I2C_ID): cv.Any(
                cv.use_id(i2c.InternalI2CBus),
                msg="I2C bus must be an internal ESP32 I2C bus",
            ),
            cv.Optional(CONF_RESET_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_POWER_DOWN_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_RESOLUTION, default="640X480"): cv.enum(
                FRAME_SIZES, upper=True
            ),
            cv.Optional(CONF_ROTATE, default="0°"): _validate_rotate,
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
            cv.Optional(CONF_DRAIN_FRAME_BUFFER_COUNT, default=0): cv.uint8_t,
            cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=1, max=100),
            cv.Optional(CONF_CONTRAST, default=0): camera_range_param,
            cv.Optional(CONF_BRIGHTNESS, default=0): camera_range_param,
            cv.Optional(CONF_SATURATION, default=0): camera_range_param,
            cv.Optional(CONF_VERTICAL_FLIP, default=True): cv.boolean,
            cv.Optional(CONF_HORIZONTAL_MIRROR, default=True): cv.boolean,
            cv.Optional(CONF_AEC_MODE, default="AUTO"): cv.enum(
                CONTROL_MODES, upper=True
            ),
            cv.Optional(CONF_AEC_VALUE, default=300): cv.int_range(min=0, max=1200),
            cv.Optional(CONF_AGC_MODE, default="AUTO"): cv.enum(
                CONTROL_MODES, upper=True
            ),
            cv.Optional(CONF_AGC_VALUE, default=0): cv.int_range(min=0, max=30),
            cv.Optional(CONF_WB_MODE, default="AUTO"): cv.enum(WB_MODES, upper=True),
            cv.Optional(CONF_BEFORE_CAPTURE): automation.validate_automation(
                single=True
            ),
            cv.Optional(CONF_AFTER_CAPTURE): automation.validate_automation(
                single=True
            ),
            cv.Optional(CONF_ON_CYCLE_END): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_exactly_one_key(CONF_I2C_PINS, CONF_I2C_ID),
)

SETTERS = {
    CONF_DATA_PINS: "set_data_pins",
    CONF_VSYNC_PIN: "set_vsync_pin",
    CONF_HREF_PIN: "set_href_pin",
    CONF_PIXEL_CLOCK_PIN: "set_pixel_clock_pin",
    CONF_RESET_PIN: "set_reset_pin",
    CONF_POWER_DOWN_PIN: "set_power_down_pin",
    CONF_RESOLUTION: "set_frame_size",
    CONF_ROTATE: "set_rotation",
    CONF_DRAIN_FRAME_BUFFER_COUNT: "set_drain_frame_buffer_count",
    CONF_JPEG_QUALITY: "set_jpeg_quality",
    CONF_CONTRAST: "set_contrast",
    CONF_BRIGHTNESS: "set_brightness",
    CONF_SATURATION: "set_saturation",
    CONF_VERTICAL_FLIP: "set_vertical_flip",
    CONF_HORIZONTAL_MIRROR: "set_horizontal_mirror",
    CONF_AEC_MODE: "set_aec_auto",
    CONF_AEC_VALUE: "set_aec_value",
    CONF_AGC_MODE: "set_agc_auto",
    CONF_AGC_VALUE: "set_agc_value",
    CONF_WB_MODE: "set_wb_mode",
}


async def to_code(config: ConfigType) -> None:
    cg.add_define("USE_CAMERA")
    add_idf_component(name="espressif/esp32-camera", ref="2.1.7")
    add_idf_sdkconfig_option("CONFIG_SCCB_HARDWARE_I2C_DRIVER_NEW", True)
    add_idf_sdkconfig_option("CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY", False)

    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)

    for key, setter in SETTERS.items():
        if key in config:
            cg.add(getattr(var, setter)(config[key]))

    clock = config[CONF_EXTERNAL_CLOCK]
    cg.add(var.set_external_clock(clock[CONF_PIN], clock[CONF_FREQUENCY]))
    if i2c_id := config.get(CONF_I2C_ID):
        cg.add(var.set_i2c_id(await cg.get_variable(i2c_id)))
    else:
        i2c_pins = config[CONF_I2C_PINS]
        cg.add(var.set_i2c_pins(i2c_pins[CONF_SDA], i2c_pins[CONF_SCL]))

    if conf := config.get(CONF_BEFORE_CAPTURE):
        before = await automation.build_automation(
            var.get_before_capture_trigger(), [], conf
        )
        cg.add(before.add_action(var.get_before_capture_done_action()))
        cg.add(var.set_has_before_capture(True))
    if conf := config.get(CONF_AFTER_CAPTURE):
        await automation.build_callback_automation(
            var, "add_after_capture_callback", [], conf
        )
    if conf := config.get(CONF_ON_CYCLE_END):
        await automation.build_callback_automation(
            var, "add_cycle_end_callback", [], conf
        )


CAPTURE_SCHEMA = automation.maybe_simple_id({cv.GenerateID(): cv.use_id(StillCamera)})


@automation.register_action(
    "still_camera.capture", CaptureAction, CAPTURE_SCHEMA, synchronous=True
)
async def capture_to_code(
    config: ConfigType,
    action_id: cg.ID,
    template_arg: cg.TemplateArguments,
    args: list,
) -> cg.MockObj:
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
