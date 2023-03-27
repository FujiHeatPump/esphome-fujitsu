from esphome import pins
from esphome.components import climate, sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.core import CORE
from esphome.const import (
    CONF_ID,
    CONF_SWITCH_DATAPOINT,
    CONF_SUPPORTS_COOL,
    CONF_SUPPORTS_HEAT,
)

CODEOWNERS = ["@rabbit-aaron", "@dgrnbrg"]
AUTO_LOAD = ["sensor"]

fujitsu_climate_ns = cg.esphome_ns.namespace("fujitsu")
FujitsuClimateComponent = fujitsu_climate_ns.class_("FujitsuClimate",
        climate.Climate,
        cg.Component,
)

CONF_IS_MASTER = "master"
CONF_REMOTE_TEMPERATURE = "remote_temperature_id"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"

def validate_tx_pin(value):
    value = pins.internal_gpio_output_pin_schema(value)
    if CORE.is_esp8266:
        raise cv.Invalid("Fujitsu Heat Pump doesn't support reassigning tx pin on esp8266")
    return value

def validate_rx_pin(value):
    value = pins.internal_gpio_input_pin_schema(value)
    if CORE.is_esp8266:
        raise cv.Invalid("Fujitsu Heat Pump doesn't support reassigning rx pin on esp8266")
    return value

CONFIG_SCHEMA = (
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(FujitsuClimateComponent),
            cv.Optional(CONF_IS_MASTER, default=True): cv.boolean,
            cv.Optional(CONF_REMOTE_TEMPERATURE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_TX_PIN): validate_tx_pin,
            cv.Optional(CONF_RX_PIN): validate_rx_pin,
        }
    ).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    cg.add(var.set_master(config[CONF_IS_MASTER]))
    if CONF_TX_PIN in config:
        tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
        cg.add(var.set_tx_pin(tx_pin))
    if CONF_RX_PIN in config:
        rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
        cg.add(var.set_rx_pin(rx_pin))
    if CONF_REMOTE_TEMPERATURE in config:
        remote_var = await cg.get_variable(config[CONF_REMOTE_TEMPERATURE])
        cg.add(var.set_remote_temperature(remote_var))
