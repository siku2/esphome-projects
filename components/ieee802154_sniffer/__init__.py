import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import (
    VARIANT_ESP32C5,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    add_idf_sdkconfig_option,
    include_builtin_idf_component,
    only_on_variant,
)
from esphome.const import CONF_CHANNEL, CONF_ID

CODEOWNERS = ["@siku2"]
DEPENDENCIES = ["esp32"]

ieee802154_sniffer_ns = cg.esphome_ns.namespace("ieee802154_sniffer")
Ieee802154Sniffer = ieee802154_sniffer_ns.class_("Ieee802154Sniffer", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(Ieee802154Sniffer),
            cv.Optional(CONF_CHANNEL, default=11): cv.int_range(min=11, max=26),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(supported=[VARIANT_ESP32C5, VARIANT_ESP32C6, VARIANT_ESP32H2]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_channel(config[CONF_CHANNEL]))

    for name in ("ieee802154", "esp_hal_ieee802154", "esp_phy", "esp_coex"):
        include_builtin_idf_component(name)
    add_idf_sdkconfig_option("CONFIG_IEEE802154_ENABLED", True)
