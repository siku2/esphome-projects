from typing import cast

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import cover, sensor
from esphome.const import (
    CONF_CLOSE_ACTION,
    CONF_CLOSE_DURATION,
    CONF_OPEN_ACTION,
    CONF_OPEN_DURATION,
    CONF_STOP_ACTION,
)

DEPENDENCIES = []

CONF_MOVING_COVERS = "moving_covers"
CONF_MOVING_CHANGE_TIMEOUT = "moving_change_timeout"

measured_endstop_cover_ns = cg.esphome_ns.namespace("measured_endstop_cover")

MeasuredEndstopCover = measured_endstop_cover_ns.class_(
    "MeasuredEndstopCover", cover.Cover, cg.Component
)

CONFIG_SCHEMA = (
    cover.cover_schema(MeasuredEndstopCover)
    .extend(
        {
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_OPEN_ACTION): automation.validate_automation(single=True),
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_ACTION): automation.validate_automation(single=True),
            cv.Required(CONF_STOP_ACTION): automation.validate_automation(single=True),
            cv.Required(CONF_MOVING_COVERS): cv.use_id(sensor.Sensor),
            cv.Required(
                CONF_MOVING_CHANGE_TIMEOUT
            ): cv.positive_time_period_milliseconds,
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    await automation.build_automation(
        var.get_open_trigger(),
        [],
        config[CONF_OPEN_ACTION],
    )

    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    await automation.build_automation(
        var.get_close_trigger(),
        [],
        config[CONF_CLOSE_ACTION],
    )

    await automation.build_automation(
        var.get_stop_trigger(),
        [],
        config[CONF_STOP_ACTION],
    )

    cg.add(
        var.set_moving_covers_sensor(
            cast(sensor.Sensor, await cg.get_variable(config[CONF_MOVING_COVERS]))
        )
    )
    cg.add(var.set_moving_change_timeout(config[CONF_MOVING_CHANGE_TIMEOUT]))
