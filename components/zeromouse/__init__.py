import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import binary_sensor, number, sensor, switch, time
from esphome.components.zeromouse_air import AirSensor
from esphome.const import (
    CONF_ID,
    CONF_TIME_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)
from esphome.types import ConfigType

AUTO_LOAD = ["binary_sensor", "number", "sensor", "switch"]
DEPENDENCIES = ["esp32", "network", "time", "zeromouse_air"]

CONF_AIR_SENSOR = "air_sensor"
CONF_PRESENCE = "presence"
CONF_STARTUP_DELAY = "startup_delay"
CONF_SETTINGS = "settings"
CONF_IR_FREE_VALUE = "ir_free_value"
CONF_IR_FREE_VALUE_RAW = "ir_free_value_raw"
CONF_ON_EVENT_START = "on_event_start"
CONF_ON_TIME_ZERO = "on_time_zero"
CONF_ON_SCAN_END = "on_scan_end"
CONF_ON_READY = "on_ready"

zeromouse_ns = cg.esphome_ns.namespace("zeromouse")
ZeroMouse = zeromouse_ns.class_("ZeroMouse", cg.Component)

SETTINGS = {
    "ir_slope_thresh_value": number.Number,
    "ir_median_count": number.Number,
    "ir_range_ignore_event": number.Number,
    "ir_ambient": number.Number,
    "cam_shots_before_t0": number.Number,
    "upload_empty": switch.Switch,
}

_DIAGNOSTIC_SENSOR = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZeroMouse),
        cv.Required(CONF_AIR_SENSOR): cv.use_id(AirSensor),
        cv.Required(CONF_PRESENCE): cv.use_id(binary_sensor.BinarySensor),
        cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Optional(
            CONF_STARTUP_DELAY, default="13s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SETTINGS): cv.Schema(
            {cv.Optional(key): cv.use_id(kind) for key, kind in SETTINGS.items()}
        ),
        cv.Optional(CONF_IR_FREE_VALUE): _DIAGNOSTIC_SENSOR,
        cv.Optional(CONF_IR_FREE_VALUE_RAW): _DIAGNOSTIC_SENSOR,
        cv.Optional(CONF_ON_EVENT_START): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_TIME_ZERO): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_SCAN_END): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_READY): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_air_sensor(await cg.get_variable(config[CONF_AIR_SENSOR])))
    cg.add(var.set_presence(await cg.get_variable(config[CONF_PRESENCE])))
    cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))
    cg.add(var.set_startup_delay(config[CONF_STARTUP_DELAY]))

    for key, entity_id in config.get(CONF_SETTINGS, {}).items():
        cg.add(getattr(var, f"set_{key}")(await cg.get_variable(entity_id)))

    if conf := config.get(CONF_IR_FREE_VALUE):
        cg.add(var.set_ir_free_value_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_IR_FREE_VALUE_RAW):
        cg.add(var.set_ir_free_value_raw_sensor(await sensor.new_sensor(conf)))

    if action := config.get(CONF_ON_EVENT_START):
        await automation.build_automation(var.get_event_start_trigger(), [], action)
    if action := config.get(CONF_ON_TIME_ZERO):
        await automation.build_automation(
            var.get_time_zero_trigger(), [(cg.int_, "pir_to_t0_ms")], action
        )
    if action := config.get(CONF_ON_SCAN_END):
        await automation.build_automation(
            var.get_scan_end_trigger(), [(cg.std_string, "result")], action
        )
    if action := config.get(CONF_ON_READY):
        await automation.build_automation(var.get_ready_trigger(), [], action)
