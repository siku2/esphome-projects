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


RECT_SCHEMA = cv.Schema(
    {
        cv.Required("top_left"): POINT_SCHEMA,
        cv.Required("bottom_right"): POINT_SCHEMA,
    }
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
