import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.components.aws_iot import AwsIotClient
from esphome.components.s3 import S3Client
from esphome.components.zeromouse import ZeroMouse
from esphome.components.zeromouse_cloud import ZeroMouseCloud
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.types import ConfigType

AUTO_LOAD = ["binary_sensor"]
DEPENDENCIES = ["esp32", "aws_iot", "zeromouse", "zeromouse_cloud"]

CONF_AWS_IOT_ID = "aws_iot_id"
CONF_CLOUD_ID = "cloud_id"
CONF_CONTROLLER_ID = "controller_id"
CONF_S3_ID = "s3_id"
CONF_CLOUD_PROVISIONED = "cloud_provisioned"
CONF_S3_PROVISIONED = "s3_provisioned"

zeromouse_provisioning_ns = cg.esphome_ns.namespace("zeromouse_provisioning")
ZeroMouseProvisioning = zeromouse_provisioning_ns.class_(
    "ZeroMouseProvisioning", cg.Component
)

_PROVISIONED_SCHEMA = binary_sensor.binary_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ZeroMouseProvisioning),
        cv.Required(CONF_AWS_IOT_ID): cv.use_id(AwsIotClient),
        cv.Required(CONF_CLOUD_ID): cv.use_id(ZeroMouseCloud),
        cv.Required(CONF_CONTROLLER_ID): cv.use_id(ZeroMouse),
        cv.Optional(CONF_S3_ID): cv.use_id(S3Client),
        cv.Optional(CONF_CLOUD_PROVISIONED): _PROVISIONED_SCHEMA,
        cv.Optional(CONF_S3_PROVISIONED): _PROVISIONED_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_aws_iot(await cg.get_variable(config[CONF_AWS_IOT_ID])))
    cg.add(var.set_cloud(await cg.get_variable(config[CONF_CLOUD_ID])))
    cg.add(var.set_controller(await cg.get_variable(config[CONF_CONTROLLER_ID])))
    if s3_id := config.get(CONF_S3_ID):
        cg.add(var.set_s3(await cg.get_variable(s3_id)))

    if sensor_config := config.get(CONF_CLOUD_PROVISIONED):
        sensor = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(var.set_cloud_provisioned_sensor(sensor))
    if sensor_config := config.get(CONF_S3_PROVISIONED):
        sensor = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(var.set_s3_provisioned_sensor(sensor))
