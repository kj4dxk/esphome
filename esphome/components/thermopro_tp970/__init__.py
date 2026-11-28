import esphome.codegen as cg
from esphome.components import ble_client
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TIMEOUT
from esphome.types import ConfigType

CODEOWNERS = ["@kj4dxk"]
DEPENDENCIES = ["ble_client"]
DOMAIN = "thermopro_tp970"
MULTI_CONF = True

CONF_THERMOPRO_TP970_ID = "thermopro_tp970_id"

thermopro_tp970_ns = cg.esphome_ns.namespace(DOMAIN)
ThermoProTP970 = thermopro_tp970_ns.class_(
    "ThermoProTP970", cg.Component, ble_client.BLEClientNode
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ThermoProTP970),
            cv.Optional(
                CONF_TIMEOUT, default="15s"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_timeout_ms(config[CONF_TIMEOUT].total_milliseconds))
