import math

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import camera_snapshot, number, sensor, text_sensor
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_ID,
    CONF_MODE,
    CONF_NAME,
    CONF_RESOLUTION,
    CONF_SOURCE,
    CONF_STATUS,
    CONF_TOLERANCE,
    DEVICE_CLASS_WATER,
    DEVICE_CLASS_VOLUME_FLOW_RATE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_COUNTER,
    ICON_GAUGE,
    ICON_PERCENT,
    ICON_RESTART_ALERT,
    ICON_WATER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_LITRE_PER_SECOND,
    UNIT_PERCENT,
)
from esphome.components.number import NUMBER_MODES

AUTO_LOAD = ["number", "sensor", "text_sensor"]
DEPENDENCIES = ["camera_snapshot"]

CONF_MAX_FLOW = "max_flow"
CONF_WHEELS = "wheels"
CONF_LEVEL = "level"
CONF_SNAPSHOTTER = "snapshotter"
CONF_CORROBORATIONS = "corroborations"
CONF_PENDING_WINDOW = "pending_window"
CONF_STALE_AFTER = "stale_after"
CONF_REANCHOR_TOLERANCE = "reanchor_tolerance"
CONF_BACK_TOLERANCE = "back_tolerance"
CONF_READING = "reading"
CONF_CONSUMPTION = "consumption"
CONF_CONFIDENCE = "confidence"
CONF_CONSISTENCY = "consistency"
CONF_REJECTED = "rejected"
CONF_REBASE = "rebase"
CONF_QUANTUM = "_quantum"

meter_reader_ns = cg.esphome_ns.namespace("meter_reader")

MeterReader = meter_reader_ns.class_("MeterReader", cg.Component)
MeterRebaseNumber = meter_reader_ns.class_("MeterRebaseNumber", number.Number)

WHEEL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_SOURCE): cv.use_id(sensor.Sensor),
        cv.Required(CONF_RESOLUTION): cv.positive_float,
        cv.Optional(CONF_TOLERANCE, default=1.5): cv.positive_float,
    }
)

READING_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="L",
    icon=ICON_WATER,
    device_class=DEVICE_CLASS_WATER,
    state_class=STATE_CLASS_TOTAL,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Reading"): cv.string,
    }
)

CONSUMPTION_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_LITRE_PER_SECOND,
    icon=ICON_GAUGE,
    device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Consumption"): cv.string,
    }
)

CONFIDENCE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    icon=ICON_PERCENT,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Confidence"): cv.string,
    }
)

CONSISTENCY_SCHEMA = sensor.sensor_schema(
    icon=ICON_GAUGE,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Consistency"): cv.string,
    }
)

REJECTED_SCHEMA = sensor.sensor_schema(
    icon=ICON_COUNTER,
    accuracy_decimals=0,
    state_class=STATE_CLASS_TOTAL_INCREASING,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Rejected"): cv.string,
    }
)

STATUS_SCHEMA = text_sensor.text_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Status"): cv.string,
    }
)

REBASE_SCHEMA = number.number_schema(
    MeterRebaseNumber,
    icon=ICON_RESTART_ALERT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="L",
).extend(
    {
        cv.Optional(CONF_NAME, default="Meter Rebase"): cv.string,
        cv.Optional(CONF_MODE, default="BOX"): cv.enum(NUMBER_MODES, upper=True),
    }
)


def _wheel_level(resolution: float, quantum: float) -> int:
    exact = math.log10(resolution / quantum) - 1.0
    level = round(exact)
    if level < 0 or abs(exact - level) > 1e-6:
        raise cv.Invalid(
            f"wheel resolution {resolution} must be a power-of-ten multiple of the finest resolution"
        )
    return level


def _validate_wheels(config):
    wheels = sorted(config[CONF_WHEELS], key=lambda w: w[CONF_RESOLUTION])
    quantum = wheels[0][CONF_RESOLUTION] / 10.0
    levels = [_wheel_level(w[CONF_RESOLUTION], quantum) for w in wheels]
    if len(set(levels)) != len(levels):
        raise cv.Invalid("wheel resolutions must be distinct")
    decimals = max(0, round(-math.log10(quantum)))
    for wheel, level in zip(wheels, levels):
        wheel[CONF_LEVEL] = level
    config[CONF_WHEELS] = wheels
    config[CONF_QUANTUM] = quantum
    config[CONF_READING].setdefault(CONF_ACCURACY_DECIMALS, decimals)
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
        cv.GenerateID(): cv.declare_id(MeterReader),
        cv.Required(CONF_SNAPSHOTTER): cv.use_id(camera_snapshot.Snapshotter),
        cv.Optional(CONF_MAX_FLOW, default=1.5): cv.positive_float,
            cv.Optional(CONF_CORROBORATIONS, default=1): cv.int_range(min=1),
            cv.Optional(
                CONF_PENDING_WINDOW, default="6s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_STALE_AFTER, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_REANCHOR_TOLERANCE, default=100.0): cv.positive_float,
            cv.Optional(CONF_BACK_TOLERANCE, default=20.0): cv.positive_float,
            cv.Required(CONF_WHEELS): cv.All(
                cv.ensure_list(WHEEL_SCHEMA),
                cv.Length(min=2),
            ),
            cv.Optional(CONF_READING, default=lambda: {CONF_NAME: "Meter Reading"}): READING_SCHEMA,
            cv.Optional(
                CONF_CONSUMPTION, default=lambda: {CONF_NAME: "Meter Consumption"}
            ): CONSUMPTION_SCHEMA,
            cv.Optional(
                CONF_CONFIDENCE, default=lambda: {CONF_NAME: "Meter Confidence"}
            ): CONFIDENCE_SCHEMA,
            cv.Optional(
                CONF_CONSISTENCY, default=lambda: {CONF_NAME: "Meter Consistency"}
            ): CONSISTENCY_SCHEMA,
            cv.Optional(
                CONF_REJECTED, default=lambda: {CONF_NAME: "Meter Rejected"}
            ): REJECTED_SCHEMA,
            cv.Optional(
                CONF_STATUS, default=lambda: {CONF_NAME: "Meter Status"}
            ): STATUS_SCHEMA,
            cv.Optional(
                CONF_REBASE, default=lambda: {CONF_NAME: "Meter Rebase"}
            ): REBASE_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_wheels,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    snapshotter = await cg.get_variable(config[CONF_SNAPSHOTTER])
    cg.add(var.set_snapshotter(snapshotter))

    reading = await sensor.new_sensor(config[CONF_READING])
    cg.add(var.set_reading_sensor(reading))
    consumption = await sensor.new_sensor(config[CONF_CONSUMPTION])
    cg.add(var.set_consumption_sensor(consumption))
    confidence = await sensor.new_sensor(config[CONF_CONFIDENCE])
    cg.add(var.set_confidence_sensor(confidence))
    consistency = await sensor.new_sensor(config[CONF_CONSISTENCY])
    cg.add(var.set_consistency_sensor(consistency))
    rejected = await sensor.new_sensor(config[CONF_REJECTED])
    cg.add(var.set_rejected_sensor(rejected))
    status = await text_sensor.new_text_sensor(config[CONF_STATUS])
    cg.add(var.set_status_sensor(status))
    rebase = await number.new_number(
        config[CONF_REBASE],
        min_value=0.0,
        max_value=1e9,
        step=config[CONF_QUANTUM],
    )
    cg.add(var.set_rebase_number(rebase))
    cg.add(rebase.set_parent(var))

    cg.add(var.set_max_flow(config[CONF_MAX_FLOW]))
    cg.add(var.set_corroborations(config[CONF_CORROBORATIONS]))
    cg.add(var.set_pending_window_ms(config[CONF_PENDING_WINDOW]))
    cg.add(var.set_stale_after_ms(config[CONF_STALE_AFTER]))
    cg.add(var.set_reanchor_tolerance(config[CONF_REANCHOR_TOLERANCE]))
    cg.add(var.set_back_tolerance(config[CONF_BACK_TOLERANCE]))

    for wheel in config[CONF_WHEELS]:
        cg.add(var.add_wheel(wheel[CONF_LEVEL], wheel[CONF_RESOLUTION], wheel[CONF_TOLERANCE]))
        source = await cg.get_variable(wheel[CONF_SOURCE])
        callback = cg.RawExpression(
            f"[{var}](float result, float fit, bool accepted) {{ "
            f"{var}->on_observation({wheel[CONF_LEVEL]}, result, fit, accepted); }}"
        )
        cg.add(source.add_on_inference_callback(callback))
