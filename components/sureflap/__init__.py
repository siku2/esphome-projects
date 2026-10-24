import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor
from esphome.components.ieee802154_sniffer import Ieee802154Sniffer
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_TOTAL_INCREASING,
)

CODEOWNERS = ["@siku2"]
DEPENDENCIES = ["ieee802154_sniffer"]
AUTO_LOAD = ["sensor", "binary_sensor"]

CONF_SNIFFER_ID = "sniffer_id"
CONF_HEARTBEAT_TIMEOUT = "heartbeat_timeout"
CONF_FRAMES = "frames"
CONF_FLAP_ONLINE = "flap_online"

sureflap_ns = cg.esphome_ns.namespace("sureflap")
SureFlap = sureflap_ns.class_("SureFlap", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(SureFlap),
        cv.GenerateID(CONF_SNIFFER_ID): cv.use_id(Ieee802154Sniffer),
        cv.Optional(
            CONF_HEARTBEAT_TIMEOUT, default="30s"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_FRAMES): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_FLAP_ONLINE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    sniffer = await cg.get_variable(config[CONF_SNIFFER_ID])
    cg.add(var.set_sniffer(sniffer))
    cg.add(var.set_heartbeat_timeout(config[CONF_HEARTBEAT_TIMEOUT]))

    if CONF_FRAMES in config:
        frames = await sensor.new_sensor(config[CONF_FRAMES])
        cg.add(var.set_frames_sensor(frames))

    if CONF_FLAP_ONLINE in config:
        flap_online = await binary_sensor.new_binary_sensor(config[CONF_FLAP_ONLINE])
        cg.add(var.set_flap_online(flap_online))
