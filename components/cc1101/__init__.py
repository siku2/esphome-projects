import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_CHANNEL,
)

DEPENDENCIES = ["spi"]
MULTI_CONF = True

# Defined for other components
CONF_CC1101_ID = "cc1101_id"
CONF_CC_MODE = "cc_mode"

cc1101_ns = cg.esphome_ns.namespace("cc1101")
Cc1101 = cc1101_ns.class_("Cc1101", cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(Cc1101),
            cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_FREQUENCY, default="433.92 Mhz"): cv.frequency,
            cv.Optional(CONF_CHANNEL, default=0): cv.uint8_t,
            cv.Optional(CONF_CC_MODE, default=False): cv.boolean,
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_tx_pin(await cg.gpio_pin_expression(config[CONF_TX_PIN])))
    cg.add(var.set_rx_pin(await cg.gpio_pin_expression(config[CONF_RX_PIN])))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_cc_mode(config[CONF_CC_MODE]))
