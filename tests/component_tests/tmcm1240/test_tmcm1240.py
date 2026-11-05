"""Device configuration boundaries and CANopenNode registration."""

from pathlib import Path

import pytest

from esphome.components.tmcm1240 import CONFIG_SCHEMA
import esphome.config_validation as cv


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("run_current", float("nan")),
        ("run_current", 2.1),
        ("standby_current", 0.7),
        ("stallguard_threshold", 64),
        ("stallguard_min_velocity", 0),
    ],
)
def test_invalid_device_settings(key: str, value: float) -> None:
    config = {
        "node_id": 32,
        "profile_velocity": 3000,
        "acceleration": 5000,
        "deceleration": 5000,
        "run_current": 0.5,
        "standby_current": 0.25,
        "stallguard_threshold": 8,
        "stallguard_min_velocity": 2800,
    }
    config[key] = value
    with pytest.raises(cv.Invalid):
        CONFIG_SCHEMA(config)


def test_registration(generate_main, tmp_path: Path) -> None:
    config = tmp_path / "motor.yaml"
    common = Path(__file__).parents[2] / "components/cia402/test.esp32-idf.yaml"
    config.write_text(
        "esphome:\n  name: motor-test\nesp32:\n  board: esp32dev\n"
        "  framework:\n    type: esp-idf\nlogger:\npackages:\n"
        f"  base: !include {common.as_posix()}\n"
        "tmcm1240:\n  id: motor\n  canopen_id: cia402_test_hub\n  node_id: 33\n"
        "  profile_velocity: 3000\n  acceleration: 5000\n  deceleration: 5000\n"
        "  run_current: 0.5\n  standby_current: 0.25\n"
        "  stallguard_threshold: 8\n  stallguard_min_velocity: 2800\n"
    )
    main = generate_main(config)
    assert "tmcm1240::TMCM1240(cia402_test_hub, 33)" in main
    assert "motor->set_master_node(16)" in main
    assert "cia402_test_hub->monitor_node(33, 1500)" in main
