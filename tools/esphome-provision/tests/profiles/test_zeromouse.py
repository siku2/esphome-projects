import argparse
import json
import subprocess
import sys
from pathlib import Path

import pytest

from esphome_provision.common import ProvisionError
from esphome_provision.efuse import (
    SAFETY_NAMES,
    parse_summary_json,
    parse_summary_text,
    safety_info,
)
from esphome_provision.firmware import CHIPS, read_firmware
from esphome_provision.flasher import ChipInfo
from esphome_provision.partitions import parse_partition_table
from esphome_provision.profiles import all_profiles
from esphome_provision.profiles.zeromouse.efuse import USER_DATA, board_id, check_board
from esphome_provision.profiles.zeromouse.names import entity_names
from esphome_provision.profiles.zeromouse.nvs import identity_from_image, parse_nvs
from esphome_provision.profiles.zeromouse.settings import STOCK_SETTINGS, stock_settings

NAMESPACES = {"iot", "system", "prox", "camera", "rfid"}
BOARDS = [
    (
        "scratch/2026-10-15-esp32s3/nvs.bin",
        "dumps/2026-10-15-esp32s3/flash-full-16mb.bin",
        "dumps/2026-10-15-esp32s3/efuse-summary.txt",
        0,
    ),
    (
        "scratch/2026-10-20-esp32s3/nvs.bin",
        "dumps/2026-10-20-esp32s3/esp32s3_flash.bin",
        "dumps/2026-10-20-esp32s3/esp32s3_efuses.txt",
        1,
    ),
]
NAMES = [*SAFETY_NAMES["esp32s3"], USER_DATA]
PROFILE = all_profiles()["zeromouse"]


def chip(label: str, size_mib: int) -> ChipInfo:
    found = next(c for c in CHIPS if c.label == label)
    return ChipInfo(found, "aa:bb:cc:dd:ee:ff", size_mib << 20)


def args(**values: object) -> argparse.Namespace:
    defaults: dict[str, object] = {
        "board": None,
        "device": None,
        "observe_only": False,
        "s3_endpoint": None,
        "s3_bucket": None,
    }
    return argparse.Namespace(**{**defaults, **values})


@pytest.mark.parametrize(("nvs", "flash", "efuses", "board"), BOARDS)
def test_efuse_text_parser(
    dumps: Path, nvs: str, flash: str, efuses: str, board: int
) -> None:
    values = parse_summary_text((dumps / efuses).read_text(), NAMES)
    info = safety_info(values, SAFETY_NAMES["esp32s3"])
    assert info.flash_encryption == "Disable"
    assert not info.secure_boot
    assert board_id(str(values[USER_DATA])) == board


def test_efuse_json_parser_on_virtual_chip() -> None:
    sample = json.dumps(
        {
            "SPI_BOOT_CRYPT_CNT": {"value": "Disable"},
            "SECURE_BOOT_EN": {"value": False},
            "DIS_DOWNLOAD_MODE": {"value": False},
            "ENABLE_SECURITY_DOWNLOAD": {"value": False},
            USER_DATA: {"value": " ".join(["01"] + ["00"] * 31)},
        }
    )
    values = parse_summary_json(
        'espefuse v5.2.0\n\n=== Run "summary" command ===\n' + sample, NAMES
    )
    assert board_id(str(values[USER_DATA])) == 1


def test_board_id_errors() -> None:
    with pytest.raises(ProvisionError):
        _ = board_id("01 00")
    check_board(1, "v2")
    with pytest.raises(ProvisionError, match="V2"):
        check_board(1, "v1")


@pytest.mark.parametrize(("nvs", "flash", "efuses", "board"), BOARDS)
def test_flash_image_has_stock_identity(
    dumps: Path, nvs: str, flash: str, efuses: str, board: int
) -> None:
    image = (dumps / flash).read_bytes()
    _, offset, partition = identity_from_image(image)
    assert offset == 0xC000
    assert (partition.offset, partition.size) == (0xD000, 0x6000)
    assert parse_partition_table(image, offset)


@pytest.mark.parametrize(("nvs", "flash", "efuses", "board"), BOARDS)
def test_nvs_matches_idf_tool(
    dumps: Path, nvs: str, flash: str, efuses: str, board: int
) -> None:
    tool_dir = (
        Path.home()
        / ".cache/esphome/idf/frameworks/5.5.5/components/nvs_flash/nvs_partition_tool"
    )
    if not tool_dir.is_dir():
        pytest.skip("IDF nvs_tool is not available")
    out = subprocess.run(
        [
            sys.executable,
            "nvs_tool.py",
            "-f",
            "json",
            "-d",
            "minimal",
            str(dumps / nvs),
        ],
        cwd=tool_dir,
        capture_output=True,
        check=True,
    ).stdout
    reference: dict[tuple[str, str], object] = {}
    for entry in json.loads(out):
        if entry["state"] == "Written" and entry["namespace"] in NAMESPACES:
            data = entry["data"]
            reference[entry["namespace"], entry["key"]] = (
                data.rstrip("\x00") if isinstance(data, str) else data
            )
    mine = {
        k: v.rstrip("\x00") if isinstance(v, str) else v
        for k, v in parse_nvs((dumps / nvs).read_bytes()).items()
        if k[0] in NAMESPACES
    }
    assert reference
    assert mine == reference


def test_board_markers_on_both_firmware_images(
    firmware_v1: Path, firmware_v2: Path
) -> None:
    for path, board, other in ((firmware_v1, "v1", "v2"), (firmware_v2, "v2", "v1")):
        data = read_firmware(path).data
        PROFILE.check_firmware(data, args(board=board))
        with pytest.raises(ProvisionError, match="not a build"):
            PROFILE.check_firmware(data, args(board=other))
        assert PROFILE.project_name in data


def test_firmware_options_are_checked() -> None:
    with pytest.raises(ProvisionError, match="--board"):
        PROFILE.check_firmware(b"", args())
    with pytest.raises(ProvisionError, match="both"):
        PROFILE.check_firmware(b"", args(board="v1", s3_endpoint="e"))


def test_chip_must_be_an_esp32s3_with_16_mb() -> None:
    PROFILE.check_chip(chip("ESP32-S3", 16))
    with pytest.raises(ProvisionError):
        PROFILE.check_chip(chip("ESP32-S3", 8))
    with pytest.raises(ProvisionError):
        PROFILE.check_chip(chip("ESP32-C3", 16))


def test_resume_arguments() -> None:
    values = args(device="kopi", observe_only=True, s3_endpoint="e", s3_bucket="b")
    assert PROFILE.resume_arguments(values) == [
        "--device", "kopi", "--s3-endpoint", "e", "--s3-bucket", "b", "--observe-only",
    ]  # fmt: skip


def test_entity_names_exist_in_both_languages() -> None:
    root = Path(__file__).resolve().parents[4]
    if not (root / "projects/zeromouse/translations").is_dir():
        pytest.skip("The repository translations are not available")
    for key in (
        "accept_cloud_settings",
        "block_enabled",
        "cloud_connected",
        "s3_endpoint",
        "s3_bucket",
    ):
        assert len(entity_names(key)) == 2


def test_stock_settings_cover_the_entities(dumps: Path) -> None:
    image = (dumps / BOARDS[1][1]).read_bytes()
    _, _, partition = identity_from_image(image)
    content = parse_nvs(image[partition.offset : partition.offset + partition.size])
    settings = stock_settings(content, skip=set())
    assert len(settings) == len(STOCK_SETTINGS)
    assert settings[0].key == "brightness"
    for setting in settings:
        assert entity_names(setting.key)
    observed = stock_settings(content, skip={"block_enabled"})
    assert "block_enabled" not in {s.key for s in observed}


def test_stock_settings_skip_missing_keys() -> None:
    settings = stock_settings({("prox", "irPreyTime"): 120}, skip=set())
    assert [(s.key, s.kind, s.value) for s in settings] == [
        ("ir_prey_event_time", "number", 120)
    ]
    with pytest.raises(ProvisionError):
        _ = stock_settings({("system", "cleanET"): b"x"}, skip=set())
