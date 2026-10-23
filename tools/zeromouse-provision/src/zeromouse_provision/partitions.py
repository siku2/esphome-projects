"""Parse ESP-IDF partition tables."""

import hashlib
import struct
from dataclasses import dataclass

from zeromouse_provision.common import ProvisionError

ENTRY_SIZE = 32
NVS_PAGE_SIZE = 4096


@dataclass(frozen=True)
class Partition:
    name: str
    type: int
    subtype: int
    offset: int
    size: int


def parse_partition_table(image: bytes, offset: int) -> list[Partition]:
    """Parse the partition table at offset. Raise ProvisionError if it is not valid."""
    table = image[offset : offset + 0x1000]
    if len(table) < 0x1000:
        raise ProvisionError(
            f"The image ends before the partition table at {offset:#x}"
        )
    partitions: list[Partition] = []
    for position in range(0, len(table), ENTRY_SIZE):
        entry = table[position : position + ENTRY_SIZE]
        if entry == b"\xff" * ENTRY_SIZE:
            break
        if entry[:2] == b"\xeb\xeb":
            if entry[16:] != hashlib.md5(table[:position]).digest():
                raise ProvisionError(
                    f"The partition table checksum at {offset:#x} is wrong"
                )
            return partitions
        if entry[:2] != b"\xaa\x50":
            raise ProvisionError(f"No partition table at {offset:#x}")
        ptype, subtype, start, size, raw_name, _ = struct.unpack("<BBII16sI", entry[2:])
        name = raw_name.split(b"\x00", 1)[0].decode("ascii", errors="replace")
        partitions.append(Partition(name, ptype, subtype, start, size))
    raise ProvisionError(f"The partition table at {offset:#x} has no checksum entry")


def find_nvs_partition(image: bytes) -> tuple[int, Partition, list[str]]:
    """Return (table offset, nvs partition) using the partition table in the image."""
    problems: list[str] = []
    for offset in (0xC000, 0x8000):
        try:
            table = parse_partition_table(image, offset)
        except ProvisionError as error:
            problems.append(str(error))
            continue
        nvs = [p for p in table if p.type == 1 and p.subtype == 2 and p.name == "nvs"]
        if len(nvs) != 1:
            problems.append(
                f"The partition table at {offset:#x} has {len(nvs)} nvs partitions"
            )
            continue
        part = nvs[0]
        if part.size % NVS_PAGE_SIZE or part.offset + part.size > len(image):
            problems.append(f"The nvs partition at {offset:#x} does not fit the image")
            continue
        return offset, part, problems
    raise ProvisionError(
        "No stock partition table with an nvs partition found. " + "; ".join(problems)
    )
