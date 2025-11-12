import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

fujitsu_heatpump_ns = cg.esphome_ns.namespace("fujitsu_heatpump")
FujitsuClimate = fujitsu_heatpump_ns.class_("FujitsuClimate", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(FujitsuClimate),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
