import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import climate, number, sensor, text_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_TIME_ID,
    CONF_UPDATE_INTERVAL,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["climate", "number", "sensor", "text_sensor", "time"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_ZONES = "zones"
CONF_WEST = "west"
CONF_EAST = "east"
CONF_CLIMATE = "climate"
CONF_PROBE = "probe"
CONF_MEAT_PROBE = "meat_probe"
CONF_MEAT_TARGET = "meat_target"
CONF_PHASE = "phase"
CONF_MEAT_RATE = "meat_rate"
CONF_ETA = "eta"
CONF_REMAINING_MINUTES = "remaining_minutes"
CONF_ON_PHASE = "on_phase"
CONF_ON_COOK_STARTED = "on_cook_started"
CONF_ON_COOK_ENDED = "on_cook_ended"

grill_cook_ns = cg.esphome_ns.namespace("grill_cook")
GrillCook = grill_cook_ns.class_("GrillCook", cg.PollingComponent)
Zone = grill_cook_ns.enum("Zone", is_class=True)

ZONE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_CLIMATE): cv.use_id(climate.Climate),
        cv.Required(CONF_PROBE): cv.use_id(sensor.Sensor),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GrillCook),
        cv.GenerateID(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Required(CONF_ZONES): cv.Schema(
            {
                cv.Required(CONF_WEST): ZONE_SCHEMA,
                cv.Required(CONF_EAST): ZONE_SCHEMA,
            }
        ),
        cv.Required(CONF_MEAT_PROBE): cv.use_id(sensor.Sensor),
        cv.Required(CONF_MEAT_TARGET): cv.use_id(number.Number),
        cv.Optional(CONF_PHASE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_MEAT_RATE): sensor.sensor_schema(
            unit_of_measurement="°C/min",
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ETA): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_REMAINING_MINUTES): sensor.sensor_schema(
            unit_of_measurement="min",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ON_PHASE): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_COOK_STARTED): automation.validate_automation(single=True),
        cv.Optional(CONF_ON_COOK_ENDED): automation.validate_automation(single=True),
    }
).extend(cv.polling_component_schema("5s"))
CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA,
    cv.Schema(
        {cv.Optional(CONF_UPDATE_INTERVAL): cv.Range(min=cv.TimePeriod(seconds=2))},
        extra=cv.ALLOW_EXTRA,
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    time_var = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(time_var))

    for zone_key, zone_enum in ((CONF_WEST, Zone.WEST), (CONF_EAST, Zone.EAST)):
        zone_config = config[CONF_ZONES][zone_key]
        zone_climate = await cg.get_variable(zone_config[CONF_CLIMATE])
        cg.add(var.set_zone_climate(zone_enum, zone_climate))
        zone_probe = await cg.get_variable(zone_config[CONF_PROBE])
        cg.add(var.set_zone_probe(zone_enum, zone_probe))

    meat_probe = await cg.get_variable(config[CONF_MEAT_PROBE])
    cg.add(var.set_meat_probe(meat_probe))

    meat_target_number = await cg.get_variable(config[CONF_MEAT_TARGET])
    cg.add(var.set_meat_target_number(meat_target_number))

    if phase_config := config.get(CONF_PHASE):
        phase_sensor = await text_sensor.new_text_sensor(phase_config)
        cg.add(var.set_phase_sensor(phase_sensor))

    if meat_rate_config := config.get(CONF_MEAT_RATE):
        meat_rate_sensor = await sensor.new_sensor(meat_rate_config)
        cg.add(var.set_meat_rate_sensor(meat_rate_sensor))

    if eta_config := config.get(CONF_ETA):
        eta_sensor = await text_sensor.new_text_sensor(eta_config)
        cg.add(var.set_eta_sensor(eta_sensor))

    if remaining_minutes_config := config.get(CONF_REMAINING_MINUTES):
        remaining_minutes_sensor = await sensor.new_sensor(remaining_minutes_config)
        cg.add(var.set_remaining_minutes_sensor(remaining_minutes_sensor))

    if action := config.get(CONF_ON_PHASE):
        await automation.build_automation(
            var.get_phase_trigger(), [(cg.std_string, "x")], action
        )

    if action := config.get(CONF_ON_COOK_STARTED):
        await automation.build_automation(var.get_cook_started_trigger(), [], action)

    if action := config.get(CONF_ON_COOK_ENDED):
        await automation.build_automation(var.get_cook_ended_trigger(), [], action)
