"""CANopenNode registration and configuration boundaries."""

from pathlib import Path

import pytest

from esphome.components.cia402 import CONFIG_SCHEMA
from esphome.config import read_config
import esphome.config_validation as cv
from esphome.core import CORE


@pytest.mark.parametrize("timeout", ["0ms", "65536ms", "10min"])
def test_heartbeat_timeout_rejected(timeout):
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA(
            {
                "node_id": 32,
                "profile_velocity": 1000,
                "acceleration": 1000,
                "deceleration": 1000,
                "heartbeat_timeout": timeout,
            }
        )


def test_registration(generate_main, tmp_path):
    config = tmp_path / "cia402.yaml"
    common = Path(__file__).parents[2] / "components/cia402/test.esp32-idf.yaml"
    config.write_text(
        "esphome:\n  name: cia402-test\nesp32:\n  board: esp32dev\n"
        "  framework:\n    type: esp-idf\nlogger:\npackages:\n"
        f"  drive: !include {common.as_posix()}\n"
    )
    main = generate_main(config)
    assert "cia402_test_hub->monitor_node(32, 1500);" in main
    assert "canopen::CANopenComponent(16)" in main
    defines = {define.name for define in CORE.defines}
    assert "USE_CANOPEN_SDO_CLIENT" in defines
    assert "USE_CANOPEN_NMT_MASTER" in defines


def test_local_node_id_cannot_be_drive(tmp_path):
    config = tmp_path / "invalid.yaml"
    config.write_text(
        "esphome:\n  name: cia402-test\nesp32:\n  board: esp32dev\n"
        "canbus:\n  - platform: esp32_can\n    id: bus\n    can_id: 0\n"
        "    tx_pin: GPIO16\n    rx_pin: GPIO17\n"
        "canopen:\n  id: hub\n  canbus_id: bus\n  node_id: 32\n"
        "cia402:\n  canopen_id: hub\n  node_id: 32\n"
        "  profile_velocity: 1000\n  acceleration: 1000\n  deceleration: 1000\n"
    )
    CORE.config_path = config
    assert read_config({}) is None
