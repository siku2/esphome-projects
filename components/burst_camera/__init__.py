import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import output
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
from esphome.const import (
    CONF_DATA_PINS,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_PIN,
    CONF_RESOLUTION,
    CONF_SCL,
    CONF_SDA,
    CONF_VSYNC_PIN,
)
from esphome.core.entity_helpers import setup_entity
from esphome.types import ConfigType

AUTO_LOAD = ["camera", "output"]
DEPENDENCIES = ["esp32", "psram"]

burst_camera_ns = cg.esphome_ns.namespace("burst_camera")
Camera = cg.esphome_ns.namespace("camera").class_("Camera", cg.EntityBase, cg.Component)
BurstCamera = burst_camera_ns.class_("BurstCamera", Camera)

CONF_HREF_PIN = "href_pin"
CONF_PIXEL_CLOCK_PIN = "pixel_clock_pin"
CONF_EXTERNAL_CLOCK = "external_clock"
CONF_I2C_PINS = "i2c_pins"
CONF_POWER_DOWN_PIN = "power_down_pin"
CONF_SUPPLY_PIN = "supply_pin"
CONF_ENABLE_PIN = "enable_pin"
CONF_LIGHT = "light"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_MAX_FRAMERATE = "max_framerate"
CONF_IDLE_FRAMERATE = "idle_framerate"
CONF_FRAME_BUFFER_COUNT = "frame_buffer_count"

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
    "1600X1200": "UXGA",
    "UXGA": "UXGA",
}
FRAME_SIZES = {
    name: getattr(cg.global_ns, f"FRAMESIZE_{size}")
    for name, size in _FRAME_SIZES.items()
}

# Only formats that frame2jpg can convert.
PIXEL_FORMATS = {
    name: getattr(cg.global_ns, f"PIXFORMAT_{name}")
    for name in ("GRAYSCALE", "RGB565", "YUV422", "RGB888")
}

CONFIG_SCHEMA = cv.All(
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(BurstCamera),
            cv.Required(CONF_DATA_PINS): cv.All(
                [pins.internal_gpio_input_pin_number], cv.Length(min=8, max=8)
            ),
            cv.Required(CONF_VSYNC_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_HREF_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_PIXEL_CLOCK_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_EXTERNAL_CLOCK): cv.Schema(
                {
                    cv.Required(CONF_PIN): pins.internal_gpio_output_pin_number,
                    cv.Optional(CONF_FREQUENCY, default="10MHz"): cv.All(
                        cv.frequency, cv.float_range(min=8e6, max=24e6)
                    ),
                }
            ),
            cv.Required(CONF_I2C_PINS): cv.Schema(
                {
                    cv.Required(CONF_SDA): pins.internal_gpio_output_pin_number,
                    cv.Required(CONF_SCL): pins.internal_gpio_output_pin_number,
                }
            ),
            cv.Optional(CONF_POWER_DOWN_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_SUPPLY_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_ENABLE_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_LIGHT): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_RESOLUTION, default="640X480"): cv.enum(
                FRAME_SIZES, upper=True
            ),
            cv.Optional(CONF_PIXEL_FORMAT, default="GRAYSCALE"): cv.enum(
                PIXEL_FORMATS, upper=True
            ),
            cv.Optional(CONF_JPEG_QUALITY, default=30): cv.int_range(min=1, max=100),
            cv.Optional(CONF_FRAME_BUFFER_COUNT, default=2): cv.int_range(
                min=1, max=12
            ),
            cv.Optional(CONF_MAX_FRAMERATE, default="10 fps"): cv.All(
                cv.framerate, cv.Range(min=0, min_included=False, max=60)
            ),
            cv.Optional(CONF_IDLE_FRAMERATE, default="0.1 fps"): cv.All(
                cv.framerate, cv.Range(min=0, max=1)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config: ConfigType) -> None:
    cg.add_define("USE_CAMERA")
    add_idf_component(name="espressif/esp32-camera", ref="2.1.7")
    add_idf_sdkconfig_option("CONFIG_SCCB_HARDWARE_I2C_DRIVER_NEW", True)
    add_idf_sdkconfig_option("CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY", False)

    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)

    cg.add(var.set_data_pins(config[CONF_DATA_PINS]))
    cg.add(var.set_vsync_pin(config[CONF_VSYNC_PIN]))
    cg.add(var.set_href_pin(config[CONF_HREF_PIN]))
    cg.add(var.set_pixel_clock_pin(config[CONF_PIXEL_CLOCK_PIN]))
    clock = config[CONF_EXTERNAL_CLOCK]
    cg.add(var.set_external_clock(clock[CONF_PIN], clock[CONF_FREQUENCY]))
    i2c_pins = config[CONF_I2C_PINS]
    cg.add(var.set_i2c_pins(i2c_pins[CONF_SDA], i2c_pins[CONF_SCL]))

    cg.add(var.set_frame_size(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_frame_buffer_count(config[CONF_FRAME_BUFFER_COUNT]))
    cg.add(var.set_max_update_interval(1000 / config[CONF_MAX_FRAMERATE]))
    if config[CONF_IDLE_FRAMERATE] == 0:
        cg.add(var.set_idle_update_interval(0))
    else:
        cg.add(var.set_idle_update_interval(1000 / config[CONF_IDLE_FRAMERATE]))

    for key, setter in (
        (CONF_POWER_DOWN_PIN, "set_power_down_pin"),
        (CONF_SUPPLY_PIN, "set_supply_pin"),
        (CONF_ENABLE_PIN, "set_enable_pin"),
    ):
        if conf := config.get(key):
            pin = await cg.gpio_pin_expression(conf)
            cg.add(getattr(var.power(), setter)(pin))

    if light := config.get(CONF_LIGHT):
        cg.add(var.power().set_light(await cg.get_variable(light)))
