import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number, sensor, text_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_TIME_ID,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["number", "sensor", "text_sensor", "time"]
AUTO_LOAD = ["text_sensor"]

CONF_GRILL_PROBE = "grill_probe"
CONF_MEAT_PROBE = "meat_probe"
CONF_TARGET_NUMBER = "target_number"
CONF_HUMIDITY_SENSOR = "humidity_sensor"
CONF_FINISH_TIME = "finish_time"
CONF_PULL_TIME = "pull_time"
CONF_COOK_PHASE = "cook_phase"
CONF_THERMAL_MASS_INDEX = "thermal_mass_index"
CONF_REST_END_TIME = "rest_end_time"
CONF_REST_REMAINING_MIN = "rest_remaining_min"
CONF_REST_DURATION_MIN = "rest_duration_min"


grill_thermal_model_ns = cg.esphome_ns.namespace("grill_thermal_model")
GrillThermalModel = grill_thermal_model_ns.class_(
    "GrillThermalModel", cg.PollingComponent
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GrillThermalModel),
        cv.GenerateID(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        cv.Required(CONF_GRILL_PROBE): cv.use_id(sensor.Sensor),
        cv.Required(CONF_MEAT_PROBE): cv.use_id(sensor.Sensor),
        cv.Required(CONF_TARGET_NUMBER): cv.use_id(number.Number),
        cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
        cv.Required(CONF_FINISH_TIME): text_sensor.text_sensor_schema(),
        cv.Required(CONF_PULL_TIME): text_sensor.text_sensor_schema(),
        cv.Required(CONF_COOK_PHASE): text_sensor.text_sensor_schema(),
        cv.Required(CONF_THERMAL_MASS_INDEX): sensor.sensor_schema(
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REST_END_TIME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_REST_REMAINING_MIN): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REST_DURATION_MIN, default=30): cv.int_range(min=5, max=240),
    }
).extend(cv.polling_component_schema("10s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    rtc = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_rtc(rtc))

    grill_probe = await cg.get_variable(config[CONF_GRILL_PROBE])
    cg.add(var.set_grill_probe(grill_probe))

    meat_probe = await cg.get_variable(config[CONF_MEAT_PROBE])
    cg.add(var.set_meat_probe(meat_probe))

    target_number = await cg.get_variable(config[CONF_TARGET_NUMBER])
    cg.add(var.set_target_number(target_number))

    if humidity_sensor_config := config.get(CONF_HUMIDITY_SENSOR):
        humidity_sensor = await cg.get_variable(humidity_sensor_config)
        cg.add(var.set_humidity_sensor(humidity_sensor))

    finish_time = await text_sensor.new_text_sensor(config[CONF_FINISH_TIME])
    cg.add(var.set_finish_time_sensor(finish_time))

    pull_time = await text_sensor.new_text_sensor(config[CONF_PULL_TIME])
    cg.add(var.set_pull_time_sensor(pull_time))

    cook_phase = await text_sensor.new_text_sensor(config[CONF_COOK_PHASE])
    cg.add(var.set_cook_phase_sensor(cook_phase))

    thermal_mass_index = await sensor.new_sensor(config[CONF_THERMAL_MASS_INDEX])
    cg.add(var.set_thermal_mass_index_sensor(thermal_mass_index))

    if rest_end_time_config := config.get(CONF_REST_END_TIME):
        rest_end_time = await text_sensor.new_text_sensor(rest_end_time_config)
        cg.add(var.set_rest_end_time_sensor(rest_end_time))

    if rest_remaining_min_config := config.get(CONF_REST_REMAINING_MIN):
        rest_remaining_min = await sensor.new_sensor(rest_remaining_min_config)
        cg.add(var.set_rest_remaining_min_sensor(rest_remaining_min))

    cg.add(var.set_rest_duration_minutes(config[CONF_REST_DURATION_MIN]))
