import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from zeromouse_provision.common import ProvisionError
from zeromouse_provision.efuse import parse_summary_json, parse_summary_text
from zeromouse_provision.flasher import check_firmware
from zeromouse_provision.nvs import identity_from_image, parse_nvs
from zeromouse_provision.partitions import parse_partition_table

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


@pytest.mark.parametrize(("nvs", "flash", "efuses", "board"), BOARDS)
def test_efuse_text_parser(
    dumps: Path, nvs: str, flash: str, efuses: str, board: int
) -> None:
    info = parse_summary_text((dumps / efuses).read_text())
    assert info.board_id == board
    assert info.flash_encryption == "Disable"
    assert not info.secure_boot


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


def test_efuse_json_parser_on_virtual_chip() -> None:
    sample = json.dumps(
        {
            "SPI_BOOT_CRYPT_CNT": {"value": "Disable"},
            "SECURE_BOOT_EN": {"value": False},
            "DIS_DOWNLOAD_MODE": {"value": False},
            "ENABLE_SECURITY_DOWNLOAD": {"value": False},
            "BLOCK_USR_DATA": {"value": " ".join(["01"] + ["00"] * 31)},
        }
    )
    assert (
        parse_summary_json(
            'espefuse v5.2.0\n\n=== Run "summary" command ===\n' + sample
        ).board_id
        == 1
    )


def test_firmware_header() -> None:
    value = os.environ.get("ZM_FIRMWARE")
    if not value or not Path(value).is_file():
        pytest.skip("ZM_FIRMWARE is not set")
    assert check_firmware(Path(value), "v2")
    with pytest.raises(ProvisionError):
        _ = check_firmware(Path(value), "v1")
