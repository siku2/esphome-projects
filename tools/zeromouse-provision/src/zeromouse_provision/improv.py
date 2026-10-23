"""Client for the Improv serial protocol (https://www.improv-wifi.com/serial/)."""

import re
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Protocol
from urllib.parse import urlparse

from zeromouse_provision.common import ProvisionError, strip_ansi

HEADER = b"IMPROV"
VERSION = 1
IPV4 = re.compile(r"\d{1,3}(?:\.\d{1,3}){3}")
LOG_IP = re.compile(r"IP Address:\s*(\d{1,3}(?:\.\d{1,3}){3})")
MAX_TRANSCRIPT = 1 << 20


class PacketType(IntEnum):
    CURRENT_STATE = 0x01
    ERROR_STATE = 0x02
    RPC_COMMAND = 0x03
    RPC_RESULT = 0x04


class State(IntEnum):
    READY = 0x01
    AUTHORIZED = 0x02
    PROVISIONING = 0x03
    PROVISIONED = 0x04


class Command(IntEnum):
    WIFI_SETTINGS = 0x01
    GET_CURRENT_STATE = 0x02
    GET_DEVICE_INFO = 0x03


ERROR_NAMES = {
    0x01: "invalid RPC packet",
    0x02: "unknown RPC command",
    0x03: "unable to connect to the Wi-Fi network",
    0x04: "not authorized",
    0xFF: "unknown error",
}


@dataclass(frozen=True)
class Packet:
    type: int
    data: bytes


class Port(Protocol):
    def write(self, data: bytes, /) -> int | None: ...
    def read(self, size: int = 1, /) -> bytes: ...


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def encode_packet(packet_type: int, data: bytes) -> bytes:
    if len(data) > 255:
        raise ProvisionError("An Improv packet can hold at most 255 data bytes")
    body = HEADER + bytes([VERSION, packet_type, len(data)]) + data
    return body + bytes([checksum(body)])


def encode_rpc(command: int, strings: list[str] | None = None) -> bytes:
    payload = b""
    for text in strings or []:
        raw = text.encode("utf-8")
        if len(raw) > 255:
            raise ProvisionError("An Improv string can hold at most 255 bytes")
        payload += bytes([len(raw)]) + raw
    if len(payload) > 254:
        raise ProvisionError("The Improv command is too long")
    return encode_packet(
        PacketType.RPC_COMMAND, bytes([command, len(payload)]) + payload
    )


def decode_rpc_result(data: bytes) -> tuple[int, list[str]]:
    if len(data) < 2 or len(data) < 2 + data[1]:
        raise ProvisionError("The Improv RPC result is truncated")
    strings: list[str] = []
    payload = data[2 : 2 + data[1]]
    position = 0
    while position < len(payload):
        length = payload[position]
        item = payload[position + 1 : position + 1 + length]
        if len(item) != length:
            raise ProvisionError("The Improv RPC result is truncated")
        strings.append(item.decode("utf-8", errors="replace"))
        position += 1 + length
    return data[0], strings


class PacketParser:
    """Extract Improv packets from a byte stream that also carries log text."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[Packet]:
        self._buffer += chunk
        packets: list[Packet] = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                del self._buffer[: max(0, len(self._buffer) - len(HEADER) + 1)]
                return packets
            del self._buffer[:start]
            if len(self._buffer) < len(HEADER) + 3:
                return packets
            length = self._buffer[len(HEADER) + 2]
            total = len(HEADER) + 3 + length + 1
            if self._buffer[len(HEADER)] != VERSION:
                del self._buffer[0]
                continue
            if len(self._buffer) < total:
                return packets
            frame = bytes(self._buffer[:total])
            if frame[-1] != checksum(frame[:-1]):
                del self._buffer[0]
                continue
            packets.append(Packet(frame[len(HEADER) + 1], frame[len(HEADER) + 3 : -1]))
            del self._buffer[:total]


@dataclass
class ImprovResult:
    urls: list[str]
    log_ip: str | None

    def host(self) -> str | None:
        for url in self.urls:
            hostname = urlparse(url).hostname
            if hostname and IPV4.fullmatch(hostname):
                return hostname
        return self.log_ip


def log_ip(transcript: bytes) -> str | None:
    matches = LOG_IP.findall(strip_ansi(transcript.decode("utf-8", errors="replace")))
    return matches[-1] if matches else None


class ImprovClient:
    def __init__(self, port: Port) -> None:
        self._port = port
        self._parser = PacketParser()
        self._transcript = bytearray()

    def _read(self) -> list[Packet]:
        chunk = self._port.read(4096)
        if not chunk:
            return []
        self._transcript += chunk
        del self._transcript[:-MAX_TRANSCRIPT]
        return self._parser.feed(chunk)

    def _send(self, data: bytes) -> None:
        _ = self._port.write(data)

    def wait_authorized(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_request = 0.0
        state: int | None = None
        while time.monotonic() < deadline:
            if time.monotonic() >= next_request:
                self._send(encode_rpc(Command.GET_CURRENT_STATE))
                next_request = time.monotonic() + 1.0
            for packet in self._read():
                if packet.type == PacketType.CURRENT_STATE and len(packet.data) == 1:
                    state = packet.data[0]
            if state == State.AUTHORIZED:
                return
            if state == State.READY:
                raise ProvisionError(
                    "The Improv port wants authorization, which this tool cannot give"
                )
            if state in (State.PROVISIONING, State.PROVISIONED):
                raise ProvisionError(
                    "The device already has Wi-Fi settings, so it is not freshly flashed"
                )
        raise ProvisionError(
            f"The device did not answer over Improv serial within {timeout:.0f} s"
        )

    def send_wifi(
        self, ssid: str, password: str, timeout: float, settle: float = 5.0
    ) -> ImprovResult:
        if not 1 <= len(ssid.encode()) <= 32:
            raise ProvisionError("The Wi-Fi SSID must be 1 to 32 bytes long")
        if len(password.encode()) > 63:
            raise ProvisionError("The Wi-Fi password must be at most 63 bytes long")
        self._send(encode_rpc(Command.WIFI_SETTINGS, [ssid, password]))
        deadline = time.monotonic() + timeout
        provisioned_at: float | None = None
        urls: list[str] | None = None
        while time.monotonic() < deadline:
            for packet in self._read():
                if (
                    packet.type == PacketType.ERROR_STATE
                    and packet.data
                    and packet.data[0] != 0
                ):
                    raise ProvisionError(
                        "Improv error: "
                        + ERROR_NAMES.get(packet.data[0], f"code {packet.data[0]:#x}")
                    )
                if packet.type == PacketType.CURRENT_STATE and packet.data[:1] == bytes(
                    [State.PROVISIONED]
                ):
                    provisioned_at = provisioned_at or time.monotonic()
                if packet.type == PacketType.RPC_RESULT:
                    command, strings = decode_rpc_result(packet.data)
                    if command in (Command.WIFI_SETTINGS, Command.GET_CURRENT_STATE):
                        urls = strings
            if provisioned_at is not None and (
                urls is not None or time.monotonic() - provisioned_at > settle
            ):
                break
        else:
            raise ProvisionError(
                f"The device did not join the Wi-Fi network within {timeout:.0f} s"
            )
        drain_until = time.monotonic() + 1.0
        while time.monotonic() < drain_until:
            _ = self._read()
        return ImprovResult(urls or [], log_ip(bytes(self._transcript)))

    def device_name(self, timeout: float = 5.0) -> str | None:
        self._send(encode_rpc(Command.GET_DEVICE_INFO))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for packet in self._read():
                if packet.type == PacketType.RPC_RESULT:
                    command, strings = decode_rpc_result(packet.data)
                    if command == Command.GET_DEVICE_INFO and len(strings) >= 4:
                        return strings[3]
        return None
