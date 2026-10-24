"""Read frames from the ieee802154_sniffer firmware and write them as pcap."""

import argparse
import struct
import sys
import time
from collections import Counter
from typing import BinaryIO

import serial

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

RECORD_FRAME = 0x01
RECORD_STATUS = 0x02
COMMAND_SET_CHANNEL = 0x10

LINKTYPE_IEEE802_15_4_TAP = 283
TAP_FCS_TYPE = 0
TAP_RSS = 1
TAP_CHANNEL = 3
TAP_LQI = 10

CHANNELS = range(11, 27)


def slip_encode(data: bytes) -> bytes:
    body = data.replace(bytes([SLIP_ESC]), bytes([SLIP_ESC, SLIP_ESC_ESC]))
    body = body.replace(bytes([SLIP_END]), bytes([SLIP_ESC, SLIP_ESC_END]))
    return bytes([SLIP_END]) + body + bytes([SLIP_END])


class SlipDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.escape = False

    def feed(self, data: bytes) -> list[bytes]:
        records: list[bytes] = []
        for byte in data:
            if byte == SLIP_END:
                if self.buf:
                    records.append(bytes(self.buf))
                self.buf.clear()
                self.escape = False
            elif byte == SLIP_ESC:
                self.escape = True
            elif self.escape:
                self.buf.append(SLIP_END if byte == SLIP_ESC_END else SLIP_ESC)
                self.escape = False
            else:
                self.buf.append(byte)
        return records


def tlv(kind: int, value: bytes) -> bytes:
    pad = b"\0" * (-len(value) % 4)
    return struct.pack("<HH", kind, len(value)) + value + pad


class PcapWriter:
    def __init__(self, out: BinaryIO) -> None:
        self.out = out
        out.write(
            struct.pack(
                "<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, LINKTYPE_IEEE802_15_4_TAP
            )
        )
        out.flush()

    def write(self, ts: float, channel: int, rssi: int, lqi: int, psdu: bytes) -> None:
        tlvs = (
            tlv(TAP_FCS_TYPE, bytes([0]))
            + tlv(TAP_RSS, struct.pack("<f", rssi))
            + tlv(TAP_CHANNEL, struct.pack("<HB", channel, 0))
            + tlv(TAP_LQI, bytes([lqi]))
        )
        packet = struct.pack("<BBH", 0, 0, 4 + len(tlvs)) + tlvs + psdu
        sec = int(ts)
        usec = int((ts - sec) * 1_000_000)
        self.out.write(struct.pack("<IIII", sec, usec, len(packet), len(packet)))
        self.out.write(packet)
        self.out.flush()


def set_channel(port: serial.Serial, channel: int) -> None:
    port.write(slip_encode(bytes([COMMAND_SET_CHANNEL, channel])))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--channel", type=int, choices=CHANNELS, default=11)
    parser.add_argument(
        "--scan",
        type=float,
        metavar="SECONDS",
        help="hop through all channels, staying this long on each",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="pcap file to write, '-' for stdout (pipe to wireshark -k -i -)",
    )
    parser.add_argument(
        "-q", "--quiet", action="store_true", help="don't print each frame"
    )
    args = parser.parse_args()

    if args.output == "-":
        pcap = PcapWriter(sys.stdout.buffer)
    elif args.output:
        pcap = PcapWriter(open(args.output, "wb"))
    else:
        pcap = None

    port = serial.Serial(args.port, timeout=0.05)
    # The device buffers records while no host is reading. Their timestamps
    # would anchor the clock wrong.
    drain_until = time.monotonic() + 0.5
    while time.monotonic() < drain_until:
        port.read(4096)
    decoder = SlipDecoder()
    offset = None
    per_channel: Counter[int] = Counter()
    dropped = 0

    channel_index = CHANNELS.index(args.channel)
    set_channel(port, args.channel)
    hop_at = time.monotonic() + args.scan if args.scan else None

    try:
        while True:
            if hop_at is not None and time.monotonic() >= hop_at:
                channel_index = (channel_index + 1) % len(CHANNELS)
                if channel_index == 0:
                    summary = " ".join(f"{ch}:{per_channel[ch]}" for ch in CHANNELS)
                    print(f"frames per channel: {summary}", file=sys.stderr)
                set_channel(port, CHANNELS[channel_index])
                hop_at = time.monotonic() + args.scan

            for record in decoder.feed(port.read(4096)):
                if record[0] == RECORD_FRAME and len(record) >= 12:
                    ts_us, channel, rssi, lqi = struct.unpack_from("<QBbB", record, 1)
                    psdu = record[12:]
                    if offset is None:
                        offset = time.time() - ts_us / 1e6
                    per_channel[channel] += 1
                    if pcap:
                        pcap.write(offset + ts_us / 1e6, channel, rssi, lqi, psdu)
                    if not args.quiet:
                        print(
                            f"ch{channel} {rssi:4d}dBm lqi{lqi:3d} {len(psdu):3d}B {psdu.hex(' ')}",
                            file=sys.stderr,
                        )
                elif record[0] == RECORD_STATUS and len(record) == 10:
                    _, received, total_dropped = struct.unpack_from("<BII", record, 1)
                    if total_dropped != dropped:
                        print(
                            f"dropped {total_dropped - dropped} frames", file=sys.stderr
                        )
                        dropped = total_dropped
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
