import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import time
from esphome.components.esp32 import (
    add_idf_component,
    add_idf_sdkconfig_option,
    idf_version,
    include_builtin_idf_component,
)
from esphome.const import (
    CONF_CERTIFICATE,
    CONF_CLIENT_ID,
    CONF_ID,
    CONF_ON_CONNECT,
    CONF_ON_DISCONNECT,
    CONF_PORT,
    CONF_TIME_ID,
    CONF_TRIGGER_ID,
    CONF_USERNAME,
)
from esphome.types import ConfigType

DEPENDENCIES = ["esp32", "network", "time"]

CONF_ENDPOINT = "endpoint"
CONF_PRIVATE_KEY = "private_key"
CONF_ROOT_CA = "root_ca"

aws_iot_ns = cg.esphome_ns.namespace("aws_iot")
AwsIotClient = aws_iot_ns.class_("AwsIotClient", cg.Component)
ConnectTrigger = aws_iot_ns.class_("ConnectTrigger", automation.Trigger.template())
DisconnectTrigger = aws_iot_ns.class_(
    "DisconnectTrigger", automation.Trigger.template()
)

_PEM_BEGIN = re.compile(r"-----BEGIN [A-Z0-9 ]+-----")
_PEM_END = re.compile(r"-----END [A-Z0-9 ]+-----")


def pem(value: object) -> str:
    value = cv.string_strict(value).strip()
    if not _PEM_BEGIN.search(value) or not _PEM_END.search(value):
        raise cv.Invalid("Expected a PEM block with BEGIN and END markers")
    return value + "\n"


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AwsIotClient),
            cv.Required(CONF_ENDPOINT): cv.domain,
            cv.Optional(CONF_PORT, default=8883): cv.port,
            cv.Required(CONF_CLIENT_ID): cv.All(cv.string_strict, cv.Length(min=1)),
            cv.Required(CONF_CERTIFICATE): pem,
            cv.Required(CONF_PRIVATE_KEY): cv.sensitive(pem),
            cv.Optional(CONF_ROOT_CA): pem,
            cv.Optional(CONF_USERNAME): cv.string_strict,
            cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ConnectTrigger)}
            ),
            cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DisconnectTrigger)}
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework("esp-idf"),
)


async def to_code(config: ConfigType) -> None:
    # The IDF mqtt component is excluded from ESPHome builds by default.
    if idf_version() >= cv.Version(6, 0, 0):
        add_idf_component(name="espressif/mqtt", ref="1.0.0")
    else:
        include_builtin_idf_component("mqtt")
    include_builtin_idf_component("esp-tls")
    # The outbox holds a copy of every unacknowledged message, which can be 128 KiB.
    add_idf_sdkconfig_option("CONFIG_MQTT_OUTBOX_DATA_ON_EXTERNAL_MEMORY", True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_endpoint(config[CONF_ENDPOINT]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_client_id(config[CONF_CLIENT_ID]))
    cg.add(var.set_certificate(config[CONF_CERTIFICATE]))
    cg.add(var.set_private_key(config[CONF_PRIVATE_KEY]))
    if root_ca := config.get(CONF_ROOT_CA):
        cg.add(var.set_root_ca(root_ca))
    if username := config.get(CONF_USERNAME):
        cg.add(var.set_username(username))
    cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))

    for conf in config.get(CONF_ON_CONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
    for conf in config.get(CONF_ON_DISCONNECT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)
