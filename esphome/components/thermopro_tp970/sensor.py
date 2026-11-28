import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_INTERNAL_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.types import ConfigType

from . import CONF_THERMOPRO_TP970_ID, ThermoProTP970

CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_NOTIFICATION_COUNT = "notification_count"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_THERMOPRO_TP970_ID): cv.use_id(ThermoProTP970),
        cv.Optional(CONF_INTERNAL_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement="°C", device_class="temperature", state_class="measurement", accuracy_decimals=1
        ),
        cv.Optional(CONF_AMBIENT_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement="°C", device_class="temperature", state_class="measurement", accuracy_decimals=1
        ),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement="%", device_class="battery", state_class="measurement", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_NOTIFICATION_COUNT): sensor.sensor_schema(
            accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_THERMOPRO_TP970_ID])
    if conf := config.get(CONF_INTERNAL_TEMPERATURE):
        var = await sensor.new_sensor(conf)
        cg.add(hub.set_internal_temperature_sensor(var))
    if conf := config.get(CONF_AMBIENT_TEMPERATURE):
        var = await sensor.new_sensor(conf)
        cg.add(hub.set_ambient_temperature_sensor(var))
    if conf := config.get(CONF_BATTERY_LEVEL):
        var = await sensor.new_sensor(conf)
        cg.add(hub.set_battery_level_sensor(var))
    if conf := config.get(CONF_NOTIFICATION_COUNT):
        var = await sensor.new_sensor(conf)
        cg.add(hub.set_notification_count_sensor(var))
