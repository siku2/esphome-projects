import pytest

from zeromouse_provision.common import ProvisionError
from zeromouse_provision.improv import (
    HEADER,
    Command,
    ImprovClient,
    PacketParser,
    PacketType,
    State,
    checksum,
    decode_rpc_result,
    encode_packet,
    encode_rpc,
)


def test_round_trip_with_log_noise() -> None:
    packet = encode_packet(PacketType.CURRENT_STATE, bytes([State.AUTHORIZED]))
    parser = PacketParser()
    stream = b"[I][wifi:1]: log IMPRO\r\n" + packet[:5]
    assert parser.feed(stream) == []
    found = parser.feed(packet[5:] + b"more log\r\n")
    assert [(p.type, p.data) for p in found] == [
        (PacketType.CURRENT_STATE, bytes([State.AUTHORIZED]))
    ]


def test_checksum_is_sum_of_preceding_bytes() -> None:
    packet = encode_packet(PacketType.ERROR_STATE, b"\x03")
    assert packet[-1] == sum(packet[:-1]) & 0xFF == checksum(packet[:-1])


def test_bad_checksum_is_dropped_and_parser_resyncs() -> None:
    good = encode_packet(PacketType.CURRENT_STATE, b"\x04")
    bad = good[:-1] + bytes([(good[-1] + 1) & 0xFF])
    found = PacketParser().feed(bad + good)
    assert len(found) == 1


def test_wifi_rpc_layout() -> None:
    packet = encode_rpc(Command.WIFI_SETTINGS, ["ab", "xyz"])
    assert packet.startswith(
        HEADER
        + bytes([1, PacketType.RPC_COMMAND, 9, 1, 7, 2])
        + b"ab"
        + bytes([3])
        + b"xyz"
    )


def test_rpc_result_decoding() -> None:
    command, strings = decode_rpc_result(bytes([1, 7, 4]) + b"http" + bytes([1]) + b"x")
    assert command == 1
    assert strings == ["http", "x"]


def test_rpc_result_truncated() -> None:
    with pytest.raises(ProvisionError):
        decode_rpc_result(bytes([1, 9, 3]) + b"ab")


class FakePort:
    def __init__(self, replies: list[bytes]) -> None:
        self.replies = replies
        self.written: list[bytes] = []

    def write(self, data: bytes, /) -> int:
        self.written.append(data)
        return len(data)

    def read(self, size: int = 1, /) -> bytes:
        del size
        return self.replies.pop(0) if self.replies else b""


def test_client_provisions_and_finds_ip_in_url() -> None:
    url = b"http://192.168.1.50:80"
    result = encode_packet(
        PacketType.RPC_RESULT, bytes([1, len(url) + 1, len(url)]) + url
    )
    port = FakePort(
        [
            encode_packet(PacketType.CURRENT_STATE, bytes([State.AUTHORIZED])),
            encode_packet(PacketType.CURRENT_STATE, bytes([State.PROVISIONING])),
            encode_packet(PacketType.CURRENT_STATE, bytes([State.PROVISIONED]))
            + result,
        ]
    )
    client = ImprovClient(port)
    client.wait_authorized(5)
    assert client.send_wifi("net", "password1", 5).host() == "192.168.1.50"


def test_client_falls_back_to_log_ip() -> None:
    result = encode_packet(PacketType.RPC_RESULT, bytes([1, 0]))
    port = FakePort(
        [
            encode_packet(PacketType.CURRENT_STATE, bytes([State.AUTHORIZED])),
            b"\x1b[0;32m[I][wifi:1]:   IP Address: 10.0.0.7\x1b[0m\r\n",
            encode_packet(PacketType.CURRENT_STATE, bytes([State.PROVISIONED]))
            + result,
        ]
    )
    client = ImprovClient(port)
    client.wait_authorized(5)
    assert client.send_wifi("net", "password1", 5).host() == "10.0.0.7"


def test_client_reports_error_state() -> None:
    port = FakePort(
        [
            encode_packet(PacketType.CURRENT_STATE, bytes([State.AUTHORIZED])),
            encode_packet(PacketType.ERROR_STATE, b"\x03"),
        ]
    )
    client = ImprovClient(port)
    client.wait_authorized(5)
    with pytest.raises(ProvisionError, match="unable to connect"):
        client.send_wifi("net", "password1", 5)
