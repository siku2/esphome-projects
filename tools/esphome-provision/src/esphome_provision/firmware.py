"""Check ESP factory images before they are written."""

import struct
from dataclasses import dataclass
from pathlib import Path

from esphome_provision.common import ProvisionError
from esphome_provision.partitions import Partition, parse_partition_table

PARTITION_TABLE_OFFSET = 0x8000
APP_DESC_MAGIC = 0xABCD5432
APP_DESC_NAME_OFFSET = 48
FLASH_SIZES_MIB = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16, 5: 32, 6: 64, 7: 128}


@dataclass(frozen=True)
class Chip:
    label: str
    esptool_id: str
    image_id: int


CHIPS = [
    Chip("ESP32", "esp32", 0),
    Chip("ESP32-S2", "esp32s2", 2),
    Chip("ESP32-C3", "esp32c3", 5),
    Chip("ESP32-S3", "esp32s3", 9),
    Chip("ESP32-C2", "esp32c2", 12),
    Chip("ESP32-C6", "esp32c6", 13),
    Chip("ESP32-H2", "esp32h2", 16),
]


def chip_by_label(label: str) -> Chip | None:
    return next((chip for chip in CHIPS if chip.label == label), None)


def chip_by_esptool_id(esptool_id: object) -> Chip | None:
    return next((chip for chip in CHIPS if chip.esptool_id == esptool_id), None)


@dataclass(frozen=True)
class Firmware:
    data: bytes
    chip_id: int
    flash_size: int
    table: list[Partition]


def read_firmware(path: Path) -> Firmware:
    """Read a factory image and check that its structure is valid."""
    try:
        data = path.read_bytes()
    except OSError as error:
        raise ProvisionError(f"Cannot read the firmware {path}") from error
    if len(data) < 0x10000 or data[0] != 0xE9:
        raise ProvisionError("The firmware is not an ESP image")
    (chip_id,) = struct.unpack_from("<H", data, 12)
    if not any(chip.image_id == chip_id for chip in CHIPS):
        raise ProvisionError(f"The firmware is for the unknown chip id {chip_id}")
    size_code = data[3] >> 4
    if size_code not in FLASH_SIZES_MIB:
        raise ProvisionError("The firmware header declares an unknown flash size")
    flash_size = FLASH_SIZES_MIB[size_code] * 1024 * 1024
    if len(data) > flash_size:
        raise ProvisionError("The firmware is larger than the flash it declares")
    try:
        table = parse_partition_table(data, PARTITION_TABLE_OFFSET)
    except ProvisionError as error:
        raise ProvisionError(
            "The firmware is not a factory image, it has no partition table at 0x8000"
        ) from error
    if not any(p.type == 0 and p.offset < len(data) for p in table):
        raise ProvisionError("The firmware has no app partition")
    return Firmware(data, chip_id, flash_size, table)


def check_target(firmware: Firmware, chip: Chip, flash_size: int) -> None:
    """Check that the image fits the chip and flash that were detected."""
    if firmware.chip_id != chip.image_id:
        raise ProvisionError(
            f"The firmware is for chip id {firmware.chip_id}, but the device is an {chip.label}"
        )
    if firmware.flash_size > flash_size:
        raise ProvisionError(
            f"The firmware needs {firmware.flash_size >> 20} MiB of flash, "
            + f"the device has {flash_size >> 20} MiB"
        )
    if len(firmware.data) > flash_size:
        raise ProvisionError("The firmware is larger than the flash")


def check_project(firmware: Firmware, project_name: bytes) -> None:
    if project_name not in firmware.data:
        raise ProvisionError(f"The firmware does not contain {project_name.decode()}")


def app_name(firmware: Firmware) -> str | None:
    """Return the name from the app description, which is the ESPHome node name."""
    apps = [p for p in firmware.table if p.type == 0 and p.offset < len(firmware.data)]
    if not apps:
        return None
    start = min(apps, key=lambda p: p.offset).offset
    desc = firmware.data[start + 32 : start + 32 + 80]
    if len(desc) < 80 or struct.unpack_from("<I", desc)[0] != APP_DESC_MAGIC:
        return None
    raw = desc[APP_DESC_NAME_OFFSET : APP_DESC_NAME_OFFSET + 32]
    name = raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return name or None
