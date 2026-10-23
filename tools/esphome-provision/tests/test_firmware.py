from pathlib import Path

import pytest
from factory_image import build_image

from esphome_provision.common import ProvisionError
from esphome_provision.firmware import (
    CHIPS,
    Chip,
    app_name,
    check_project,
    check_target,
    read_firmware,
)

MIB = 1024 * 1024
S3 = next(chip for chip in CHIPS if chip.label == "ESP32-S3")


def write(tmp_path: Path, data: bytes) -> Path:
    path = tmp_path / "firmware.bin"
    _ = path.write_bytes(data)
    return path


@pytest.mark.parametrize("chip", CHIPS, ids=lambda chip: chip.label)
def test_every_chip_id_matches_only_its_chip(tmp_path: Path, chip: Chip) -> None:
    firmware = read_firmware(write(tmp_path, build_image(chip_id=chip.image_id)))
    check_target(firmware, chip, 16 * MIB)
    other = next(c for c in CHIPS if c is not chip)
    with pytest.raises(ProvisionError, match="chip id"):
        check_target(firmware, other, 16 * MIB)


def test_declared_flash_must_fit_the_device(tmp_path: Path) -> None:
    firmware = read_firmware(write(tmp_path, build_image(size_code=4)))
    check_target(firmware, S3, 16 * MIB)
    with pytest.raises(ProvisionError, match="16 MiB"):
        check_target(firmware, S3, 4 * MIB)
    small = read_firmware(write(tmp_path, build_image(size_code=2)))
    check_target(small, S3, 16 * MIB)


def test_app_name_is_read_from_the_app_description(tmp_path: Path) -> None:
    firmware = read_firmware(write(tmp_path, build_image(name="bean-kopi")))
    assert app_name(firmware) == "bean-kopi"


def test_project_name_must_be_in_the_image(tmp_path: Path) -> None:
    firmware = read_firmware(write(tmp_path, build_image(extra=b"siku2.zeromouse")))
    check_project(firmware, b"siku2.zeromouse")
    with pytest.raises(ProvisionError):
        check_project(firmware, b"other.project")


def test_rejects_images_that_are_not_factory_images(tmp_path: Path) -> None:
    good = build_image()
    cases = {
        "magic": b"\x00" + good[1:],
        "chip": good[:12] + b"\x63\x00" + good[14:],
        "size": good[:3] + b"\xf0" + good[4:],
        "table": good[:0x8000] + b"\xff" * 0x1000 + good[0x9000:],
        "short": good[:0x8000],
    }
    for name, data in cases.items():
        with pytest.raises(ProvisionError):
            _ = read_firmware(write(tmp_path, data))
        del name


def test_rejects_a_missing_file(tmp_path: Path) -> None:
    with pytest.raises(ProvisionError):
        _ = read_firmware(tmp_path / "missing.bin")


def test_real_images_are_valid(firmware_v1: Path, firmware_v2: Path) -> None:
    for path in (firmware_v1, firmware_v2):
        firmware = read_firmware(path)
        check_target(firmware, S3, 16 * MIB)
        assert app_name(firmware)
