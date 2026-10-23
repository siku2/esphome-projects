import hashlib
import struct

APP_OFFSET = 0x10000


def partition_table(entries: list[tuple[int, int, int, int, str]]) -> bytes:
    table = b""
    for ptype, subtype, offset, size, name in entries:
        table += b"\xaa\x50" + struct.pack(
            "<BBII16sI", ptype, subtype, offset, size, name.encode(), 0
        )
    table += b"\xeb\xeb" + b"\xff" * 14 + hashlib.md5(table).digest()
    return table.ljust(0x1000, b"\xff")


def build_image(
    chip_id: int = 9,
    size_code: int = 4,
    name: str = "bean-test",
    extra: bytes = b"",
) -> bytes:
    """Build a small factory image with a partition table and an app description."""
    data = bytearray(b"\xff" * (APP_OFFSET + 0x1000))
    data[0] = 0xE9
    data[3] = size_code << 4
    struct.pack_into("<H", data, 12, chip_id)
    table = partition_table(
        [
            (1, 2, 0xD000, 0x6000, "nvs"),
            (0, 0, APP_OFFSET, 0x100000, "app0"),
        ]
    )
    data[0x8000 : 0x8000 + len(table)] = table
    data[APP_OFFSET] = 0xE9
    struct.pack_into("<I", data, APP_OFFSET + 32, 0xABCD5432)
    data[APP_OFFSET + 80 : APP_OFFSET + 112] = name.encode().ljust(32, b"\x00")
    data += extra
    return bytes(data)
