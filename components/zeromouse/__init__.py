import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import (
    binary_sensor,
    number,
    select,
    sensor,
    switch,
    text_sensor,
    time,
)
from esphome.components.burst_camera import BurstCamera
from esphome.components.zeromouse_air import AirSensor, PresenceSampler
from esphome.const import (
    CONF_ID,
    CONF_TIME_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)
from esphome.types import ConfigType

AUTO_LOAD = [
    "binary_sensor",
    "number",
    "select",
    "sensor",
    "switch",
    "task_util",
    "text_sensor",
]
DEPENDENCIES = ["burst_camera", "esp32", "network", "time", "zeromouse_air"]

CONF_AIR_SENSOR = "air_sensor"
CONF_PRESENCE = "presence"
CONF_PRESENCE_SAMPLER = "presence_sampler"
CONF_STARTUP_DELAY = "startup_delay"
CONF_EVENT_FRAME_SPACING = "event_frame_spacing"
CONF_SETTINGS = "settings"
CONF_IR_FREE_VALUE = "ir_free_value"
CONF_IR_FREE_VALUE_RAW = "ir_free_value_raw"
CONF_ON_EVENT_START = "on_event_start"
CONF_ON_TIME_ZERO = "on_time_zero"
CONF_ON_SCAN_END = "on_scan_end"
CONF_ON_READY = "on_ready"
CONF_ON_EVENT_END = "on_event_end"
CONF_ON_PUBLISHED = "on_published"
CONF_ON_WATCHDOG = "on_watchdog"
CONF_WATCHDOG_ID = "watchdog_id"
CONF_CAMERA = "camera"
CONF_BLOCKER = "blocker"
CONF_CLOUD = "cloud"
CONF_STATE = "state"

zeromouse_ns = cg.esphome_ns.namespace("zeromouse")
ZeroMouse = zeromouse_ns.class_("ZeroMouse", cg.Component)
ZeroMouseWatchdog = zeromouse_ns.class_("ZeroMouseWatchdog", cg.Component)
# Declared here so that the controller works when the cloud component is not loaded.
ZeroMouseCloud = cg.esphome_ns.namespace("zeromouse_cloud").class_(
    "ZeroMouseCloud", cg.Component
)

SETTINGS = {
    "ir_slope_thresh_value": number.Number,
    "ir_median_count": number.Number,
    "ir_range_ignore_event": number.Number,
    "ir_ambient": number.Number,
    "cam_shots_before_t0": number.Number,
    "upload_empty": switch.Switch,
    "block_enabled": switch.Switch,
    "undecidable_mode": select.Select,
    "clean_event_time": number.Number,
    "finale_interval_max_count": number.Number,
    "finale_interval_delta": number.Number,
    "ir_prey_event_time": number.Number,
    "last_ref_event_max_count": number.Number,
    "last_ref_event_max_time_sec": number.Number,
    "brightness": number.Number,
    "auto_exp_level": number.Number,
    "auto_gain_ctrl_min": number.Number,
    "auto_gain_ctrl_max": number.Number,
    "auto_gain_stbl_min": number.Number,
    "auto_gain_stbl_max": number.Number,
    "gain_ceiling": number.Number,
}

_DIAGNOSTIC_SENSOR = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZeroMouse),
        cv.GenerateID(CONF_WATCHDOG_ID): cv.declare_id(ZeroMouseWatchdog),
        cv.Required(CONF_AIR_SENSOR): cv.use_id(AirSensor),
        cv.Required(CONF_PRESENCE): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_PRESENCE_SAMPLER): cv.use_id(PresenceSampler),
        cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Required(CONF_CAMERA): cv.use_id(BurstCamera),
        cv.Optional(CONF_BLOCKER): cv.use_id(switch.Switch),
        cv.Optional(CONF_CLOUD): cv.use_id(ZeroMouseCloud),
        cv.Optional(
            CONF_STARTUP_DELAY, default="13s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_EVENT_FRAME_SPACING, default="80ms"
        ): cv.positive_time_period_microseconds,
        cv.Optional(CONF_SETTINGS): cv.Schema(
            {cv.Optional(key): cv.use_id(kind) for key, kind in SETTINGS.items()}
        ),
        cv.Optional(CONF_IR_FREE_VALUE): _DIAGNOSTIC_SENSOR,
        cv.Optional(CONF_IR_FREE_VALUE_RAW): _DIAGNOSTIC_SENSOR,
        cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_ON_EVENT_START): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_TIME_ZERO): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_SCAN_END): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_READY): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_EVENT_END): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_PUBLISHED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_WATCHDOG): automation.validate_automation(single=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_air_sensor(await cg.get_variable(config[CONF_AIR_SENSOR])))
    cg.add(var.set_presence(await cg.get_variable(config[CONF_PRESENCE])))
    if sampler := config.get(CONF_PRESENCE_SAMPLER):
        cg.add(var.set_presence_sampler(await cg.get_variable(sampler)))
    cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))
    cg.add(var.set_startup_delay(config[CONF_STARTUP_DELAY]))
    cg.add(var.set_event_frame_spacing(config[CONF_EVENT_FRAME_SPACING]))
    cg.add(var.set_camera(await cg.get_variable(config[CONF_CAMERA])))
    watchdog = cg.new_Pvariable(config[CONF_WATCHDOG_ID])
    await cg.register_component(watchdog, config)
    cg.add(watchdog.set_controller(var))
    if blocker := config.get(CONF_BLOCKER):
        blocker_var = await cg.get_variable(blocker)
        cg.add(var.set_blocker(blocker_var))
        cg.add(watchdog.set_blocker(blocker_var))
    if cloud := config.get(CONF_CLOUD):
        cg.add_define("USE_ZEROMOUSE_CONTROLLER_CLOUD")
        cg.add(var.set_cloud(await cg.get_variable(cloud)))

    for key, entity_id in config.get(CONF_SETTINGS, {}).items():
        cg.add(getattr(var, f"set_{key}")(await cg.get_variable(entity_id)))

    if conf := config.get(CONF_IR_FREE_VALUE):
        cg.add(var.set_ir_free_value_sensor(await sensor.new_sensor(conf)))
    if conf := config.get(CONF_IR_FREE_VALUE_RAW):
        cg.add(var.set_ir_free_value_raw_sensor(await sensor.new_sensor(conf)))

    if conf := config.get(CONF_STATE):
        cg.add(var.set_state_sensor(await text_sensor.new_text_sensor(conf)))

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
    if action := config.get(CONF_ON_EVENT_END):
        await automation.build_automation(
            var.get_event_end_trigger(), [(cg.std_string, "reason")], action
        )
    if action := config.get(CONF_ON_PUBLISHED):
        await automation.build_automation(
            var.get_published_trigger(), [(cg.uint32, "event_id")], action
        )
    if action := config.get(CONF_ON_WATCHDOG):
        await automation.build_automation(
            watchdog.get_trigger(), [(cg.std_string, "reason")], action
        )
