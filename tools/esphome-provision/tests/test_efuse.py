import json

import pytest

from esphome_provision.common import ProvisionError
from esphome_provision.efuse import (
    SAFETY_NAMES,
    SafetyInfo,
    check_safe_to_flash,
    parse_summary_json,
    safety_info,
    safety_names,
)

NAMES = SAFETY_NAMES["esp32s3"]


def sample(**changes: object) -> dict[str, object]:
    values: dict[str, object] = {
        "SPI_BOOT_CRYPT_CNT": "Disable",
        "SECURE_BOOT_EN": False,
        "DIS_DOWNLOAD_MODE": False,
        "ENABLE_SECURITY_DOWNLOAD": False,
    }
    values.update(changes)
    return values


def as_json(values: dict[str, object]) -> str:
    body = json.dumps({k: {"value": v} for k, v in values.items()})
    return 'espefuse v5.2.0\n\n=== Run "summary" command ===\n' + body


def test_json_parser_skips_the_banner() -> None:
    values = parse_summary_json(as_json(sample()), NAMES)
    check_safe_to_flash(safety_info(values, NAMES))


@pytest.mark.parametrize(
    "change",
    [
        {"SPI_BOOT_CRYPT_CNT": "Enable"},
        {"SECURE_BOOT_EN": True},
        {"DIS_DOWNLOAD_MODE": True},
        {"ENABLE_SECURITY_DOWNLOAD": True},
    ],
)
def test_unsafe_chips_are_refused(change: dict[str, object]) -> None:
    values = parse_summary_json(as_json(sample(**change)), NAMES)
    with pytest.raises(ProvisionError):
        check_safe_to_flash(safety_info(values, NAMES))


def test_missing_and_malformed_values_are_refused() -> None:
    with pytest.raises(ProvisionError):
        _ = parse_summary_json(as_json({"SECURE_BOOT_EN": False}), NAMES)
    with pytest.raises(ProvisionError):
        _ = parse_summary_json("no json here", NAMES)
    with pytest.raises(ProvisionError):
        _ = parse_summary_json(as_json(sample(SECURE_BOOT_EN=3)), NAMES)


def test_safety_info_fields() -> None:
    values = parse_summary_json(as_json(sample()), NAMES)
    assert safety_info(values, NAMES) == SafetyInfo("Disable", False, False, False)


def test_unknown_chips_are_refused() -> None:
    assert safety_names("esp32s3", "ESP32-S3")
    with pytest.raises(ProvisionError, match="not implemented"):
        _ = safety_names("esp32c3", "ESP32-C3")
