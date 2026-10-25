import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

DEPENDENCIES = ["esp32"]

nvs_usage_ns = cg.esphome_ns.namespace("nvs_usage")
NVSUsageSensor = nvs_usage_ns.class_(
    "NVSUsageSensor", sensor.Sensor, cg.PollingComponent
)

CONFIG_SCHEMA = sensor.sensor_schema(
    NVSUsageSensor,
    unit_of_measurement=UNIT_PERCENT,
    icon="mdi:database",
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend(cv.polling_component_schema("1h"))


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
