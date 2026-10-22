import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import switch
from esphome.components.esp32 import include_builtin_idf_component
from esphome.const import CONF_PIN
from esphome.types import ConfigType

from . import fdxb_emulator_ns

DEPENDENCIES = ["esp32"]

FdxbEmulator = fdxb_emulator_ns.class_("FdxbEmulator", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.switch_schema(FdxbEmulator, default_restore_mode="ALWAYS_OFF")
    .extend({cv.Required(CONF_PIN): pins.internal_gpio_output_pin_number})
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config: ConfigType) -> None:
    include_builtin_idf_component("esp_driver_rmt")

    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    cg.add(var.set_pin(config[CONF_PIN]))
