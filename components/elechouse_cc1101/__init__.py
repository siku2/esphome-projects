import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_RX_PIN,
    CONF_TX_PIN,
)

# Defined for other components
CONF_ELECHOUSE_CC1101_ID = "elechouse_cc1101_id"

cc1101_ns = cg.esphome_ns.namespace("elechouse_cc1101")
ElechouseCc1101 = cc1101_ns.class_("ElechouseCc1101", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(ElechouseCc1101),
            cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_FREQUENCY): cv.frequency,
        },
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    # Undeclared internal dependency of SmartRC-CC1101-Driver-Lib
    cg.add_library("SPI", None)
    cg.add_library("SmartRC-CC1101-Driver-Lib", "2.5.7")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))
    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))

    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))
