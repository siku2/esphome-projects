import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import camera_snapshot, sensor
from esphome.const import CONF_ID
from esphome.types import ConfigType

from . import InterpreterComponent, tflite_ns

CONF_SNAPSHOTTER = "snapshotter"
CONF_INTERPRETER = "interpreter"
CONF_CROP = "crop"

CameraSnapshotSensor = tflite_ns.class_(
    "CameraSnapshotSensor",
    sensor.Sensor,
    cg.Component,
    camera_snapshot.SnapshotListener,
)
Point = tflite_ns.struct("Point")
Rect = tflite_ns.struct("Rect")

POINT_SCHEMA = cv.Schema(
    {
        cv.Required("x"): cv.uint16_t,
        cv.Required("y"): cv.uint16_t,
    }
)


def point_to_code(config: ConfigType) -> cg.StructInitializer:
    return cg.StructInitializer(
        Point,
        ("x", config["x"]),
        ("y", config["y"]),
    )


def _validate_rect(config: ConfigType) -> ConfigType:
    top_left = config["top_left"]
    (tl_x, tl_y) = (top_left["x"], top_left["y"])
    bottom_right = config["bottom_right"]
    (br_x, br_y) = (bottom_right["x"], bottom_right["y"])
    if tl_x == br_x or tl_y == br_y:
        raise cv.Invalid("rect must not be empty")
    if tl_x > br_x or tl_y > br_y:
        raise cv.Invalid(
            "rect's top-left point must be above and to the left of bottom-right"
        )
    return config


RECT_SCHEMA = cv.All(
    cv.Any(
        cv.Schema(
            {
                cv.Required("top_left"): POINT_SCHEMA,
                cv.Required("bottom_right"): POINT_SCHEMA,
            }
        ),
        cv.All(
            cv.Schema(
                {
                    cv.Required("top_left"): POINT_SCHEMA,
                    cv.Required("width"): cv.uint16_t,
                    cv.Required("height"): cv.uint16_t,
                }
            ),
            lambda config: {
                "top_left": config["top_left"],
                "bottom_right": {
                    "x": config["top_left"]["x"] + config["width"] + 1,
                    "y": config["top_left"]["y"] + config["height"] + 1,
                },
            },
        ),
    ),
    _validate_rect,
)


def rect_to_code(config: ConfigType) -> cg.StructInitializer:
    return cg.StructInitializer(
        Rect,
        (
            "top_left",
            point_to_code(config["top_left"]),
        ),
        (
            "bottom_right",
            point_to_code(config["bottom_right"]),
        ),
    )


CONFIG_SCHEMA = sensor.sensor_schema(CameraSnapshotSensor).extend(
    {
        cv.GenerateID(CONF_SNAPSHOTTER): cv.use_id(camera_snapshot.Snapshotter),
        cv.GenerateID(CONF_INTERPRETER): cv.use_id(InterpreterComponent),
        cv.Required(CONF_CROP): RECT_SCHEMA,
    }
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)

    snapshotter = await cg.get_variable(config[CONF_SNAPSHOTTER])
    cg.add(var.set_snapshotter(snapshotter))
    interpreter = await cg.get_variable(config[CONF_INTERPRETER])
    cg.add(var.set_interpreter_component(interpreter))
    cg.add(var.set_crop(rect_to_code(config[CONF_CROP])))
