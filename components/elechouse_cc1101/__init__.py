import esphome.codegen as cg
from esphome import pins
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TX_PIN,
    CONF_RX_PIN,
    CONF_FREQUENCY,
)

cc1101_ns = cg.global_ns.namespace("siku2").namespace("elechouse_cc1101")
ElechouseCc1101 = cc1101_ns.class_("ElechouseCc1101", cg.Component)

CONF_CC1101_ID = "cc1101_id"


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
    cg.add_library("SmartRC-CC1101-Driver-Lib", "2.5.7")

    var = cg.new_Pvariable(config[CONF_ID])
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))
    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))

    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))
