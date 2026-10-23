"""Read-only parser for ESP-IDF NVS (format version 2) and partition tables."""

import struct
import zlib
from dataclasses import dataclass, field

from cryptography import x509
from cryptography.hazmat.primitives import serialization

from esphome_provision.common import ProvisionError
from esphome_provision.partitions import Partition, find_nvs_partition

PAGE_SIZE = 4096
ENTRY_SIZE = 32
ENTRIES_PER_PAGE = 126
PAGE_HEADER_SIZE = 32
BITMAP_SIZE = 32
ENTRIES_OFFSET = PAGE_HEADER_SIZE + BITMAP_SIZE
VERSION_2 = 0xFE
PAGE_STATES = {0xFFFFFFFE: "active", 0xFFFFFFFC: "full", 0xFFFFFFF8: "freeing"}
PAGE_UNINITIALIZED = 0xFFFFFFFF

ENTRY_WRITTEN = 0b10
ENTRY_ERASED = 0b00

TYPE_U8, TYPE_I8 = 0x01, 0x11
TYPE_U16, TYPE_I16 = 0x02, 0x12
TYPE_U32, TYPE_I32 = 0x04, 0x14
TYPE_U64, TYPE_I64 = 0x08, 0x18
TYPE_STR = 0x21
TYPE_BLOB = 0x41
TYPE_BLOB_DATA = 0x42
TYPE_BLOB_IDX = 0x48
INTEGER_FORMATS = {
    TYPE_U8: "<B",
    TYPE_I8: "<b",
    TYPE_U16: "<H",
    TYPE_I16: "<h",
    TYPE_U32: "<I",
    TYPE_I32: "<i",
    TYPE_U64: "<Q",
    TYPE_I64: "<q",
}

NvsValue = int | str | bytes
NvsContent = dict[tuple[str, str], NvsValue]


class NvsError(ProvisionError):
    pass


def esp_crc32(data: bytes) -> int:
    return zlib.crc32(data, 0xFFFFFFFF)


@dataclass
class _Page:
    sequence: int
    index: int
    data: bytes


def _pages(data: bytes) -> list[_Page]:
    if len(data) == 0 or len(data) % PAGE_SIZE:
        raise NvsError(f"The NVS size {len(data)} is not a multiple of {PAGE_SIZE}")
    pages: list[_Page] = []
    for index in range(len(data) // PAGE_SIZE):
        page = data[index * PAGE_SIZE : (index + 1) * PAGE_SIZE]
        state, sequence = struct.unpack_from("<II", page, 0)
        if state == PAGE_UNINITIALIZED:
            continue
        if state not in PAGE_STATES:
            raise NvsError(f"NVS page {index} has an unknown state {state:#x}")
        if page[8] != VERSION_2:
            raise NvsError(f"NVS page {index} is not format version 2")
        (stored_crc,) = struct.unpack_from("<I", page, 28)
        if stored_crc != esp_crc32(page[4:28]):
            raise NvsError(f"NVS page {index} has a wrong header checksum")
        pages.append(_Page(sequence, index, page))
    pages.sort(key=lambda p: p.sequence)
    return pages


def _entry_state(page: bytes, slot: int) -> int:
    return (page[PAGE_HEADER_SIZE + slot // 4] >> ((slot % 4) * 2)) & 0b11


def parse_nvs(data: bytes) -> NvsContent:
    """Return the newest value of every (namespace, key) in an NVS partition image."""
    namespaces: dict[int, str] = {}
    singles: dict[tuple[int, str], tuple[int, bytes]] = {}
    blob_chunks: dict[tuple[int, str], dict[int, bytes]] = {}
    blob_indexes: dict[tuple[int, str], tuple[int, int, int]] = {}

    for page in _pages(data):
        slot = 0
        while slot < ENTRIES_PER_PAGE:
            state = _entry_state(page.data, slot)
            if state != ENTRY_WRITTEN:
                slot += 1
                continue
            start = ENTRIES_OFFSET + slot * ENTRY_SIZE
            entry = page.data[start : start + ENTRY_SIZE]
            where = f"page {page.index} entry {slot}"
            ns, kind, span, chunk, crc = struct.unpack_from("<BBBBI", entry, 0)
            if crc != esp_crc32(entry[:4] + entry[8:]):
                raise NvsError(f"NVS {where} has a wrong checksum")
            key = entry[8:24].split(b"\x00", 1)[0].decode("ascii", errors="replace")
            payload = entry[24:]
            if span < 1 or slot + span > ENTRIES_PER_PAGE:
                raise NvsError(f"NVS {where} has an invalid span")
            if kind in (TYPE_STR, TYPE_BLOB, TYPE_BLOB_DATA):
                size, _, data_crc = struct.unpack("<HHI", payload)
                extra_start = start + ENTRY_SIZE
                extra = page.data[extra_start : extra_start + (span - 1) * ENTRY_SIZE]
                if size > len(extra):
                    raise NvsError(f"NVS {where} has an invalid size")
                value = extra[:size]
                if data_crc != esp_crc32(value):
                    raise NvsError(f"NVS {where} has a wrong data checksum")
                if kind == TYPE_BLOB_DATA:
                    blob_chunks.setdefault((ns, key), {})[chunk] = value
                else:
                    singles[(ns, key)] = (kind, value)
            elif span != 1:
                raise NvsError(f"NVS {where} has an invalid span")
            elif kind == TYPE_BLOB_IDX:
                size, count, chunk_start = struct.unpack_from("<IBB", payload)
                blob_indexes[(ns, key)] = (size, count, chunk_start)
            elif kind in INTEGER_FORMATS:
                width = struct.calcsize(INTEGER_FORMATS[kind])
                singles[(ns, key)] = (kind, payload[:width])
                if ns == 0:
                    namespaces[payload[0]] = key
            else:
                raise NvsError(f"NVS {where} has the unknown type {kind:#x}")
            slot += span

    content: NvsContent = {}
    for (ns, key), (kind, raw) in singles.items():
        if ns == 0:
            continue
        content[_name(namespaces, ns), key] = _decode(kind, raw)
    for (ns, key), (size, count, chunk_start) in blob_indexes.items():
        chunks = blob_chunks.get((ns, key), {})
        parts = [chunks.get(chunk_start + i) for i in range(count)]
        if any(part is None for part in parts):
            raise NvsError(f"The blob {key} is missing a chunk")
        blob = b"".join(part for part in parts if part is not None)
        if len(blob) != size:
            raise NvsError(f"The blob {key} has the wrong size")
        content[_name(namespaces, ns), key] = blob
    return content


def _name(namespaces: dict[int, str], ns: int) -> str:
    if ns not in namespaces:
        raise NvsError(f"NVS namespace {ns} has no name")
    return namespaces[ns]


def _decode(kind: int, raw: bytes) -> NvsValue:
    if kind == TYPE_STR:
        return raw.rstrip(b"\x00").decode("utf-8")
    if kind == TYPE_BLOB:
        return raw
    (number,) = struct.unpack(INTEGER_FORMATS[kind], raw)
    return int(number)


REQUIRED_KEYS = [
    ("iot", "thingName"),
    ("iot", "cert"),
    ("iot", "privKey"),
    ("system", "ownerID"),
    ("system", "identityID"),
    ("system", "eventCount"),
    ("system", "bootCount"),
]


@dataclass(frozen=True)
class StockIdentity:
    thing_name: str
    owner_id: str
    identity_id: str
    event_count: int
    boot_count: int
    certificate: str = field(repr=False)
    private_key: str = field(repr=False)


def _text(content: NvsContent, ns: str, key: str) -> str:
    value = content.get((ns, key))
    if isinstance(value, bytes):
        value = value.rstrip(b"\x00").decode("utf-8")
    if not isinstance(value, str) or not value.strip():
        raise NvsError(f"The stock NVS has no usable text value {ns}/{key}")
    return value


def _count(content: NvsContent, ns: str, key: str) -> int:
    value = content.get((ns, key))
    if not isinstance(value, int) or not 0 <= value < 2**31:
        raise NvsError(f"The stock NVS has no usable counter {ns}/{key}")
    return value


def check_certificate_and_key(certificate: str, private_key: str) -> None:
    try:
        cert = x509.load_pem_x509_certificate(certificate.encode())
    except ValueError as error:
        raise NvsError("The device certificate is not valid PEM") from error
    try:
        key = serialization.load_pem_private_key(private_key.encode(), password=None)
    except (ValueError, TypeError) as error:
        raise NvsError("The device private key is not valid PEM") from error
    der = serialization.Encoding.DER
    spki = serialization.PublicFormat.SubjectPublicKeyInfo
    if cert.public_key().public_bytes(der, spki) != key.public_key().public_bytes(
        der, spki
    ):
        raise NvsError("The private key does not match the certificate")


def stock_identity(content: NvsContent) -> StockIdentity:
    missing = [f"{ns}/{key}" for ns, key in REQUIRED_KEYS if (ns, key) not in content]
    if missing:
        raise NvsError(
            "The NVS is not from a stock image, missing " + ", ".join(missing)
        )
    identity = StockIdentity(
        thing_name=_text(content, "iot", "thingName"),
        owner_id=_text(content, "system", "ownerID"),
        identity_id=_text(content, "system", "identityID"),
        event_count=_count(content, "system", "eventCount"),
        boot_count=_count(content, "system", "bootCount"),
        certificate=_text(content, "iot", "cert"),
        private_key=_text(content, "iot", "privKey"),
    )
    check_certificate_and_key(identity.certificate, identity.private_key)
    return identity


def identity_from_image(image: bytes) -> tuple[StockIdentity, int, Partition]:
    """Find the stock NVS in a full flash image and return the validated identity."""
    offset, partition, _ = find_nvs_partition(image)
    content = parse_nvs(image[partition.offset : partition.offset + partition.size])
    return stock_identity(content), offset, partition
