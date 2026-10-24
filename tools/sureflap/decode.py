"""Decode SureFlap cat flap traffic from an 802.15.4 pcap.

The flap and hub talk over 802.15.4 on PAN 0x3421. Application messages of type
0x01 are encrypted with a stream cipher. The keystream is fixed for a session
(the span between two 0x0e session-setup messages) and reused for every message,
which lets us recover it from known plaintext and decrypt without the key. See
scratch/sureflap-protocol.md.
"""

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field

PAN = 0x3421
HUB_SUFFIX = "98:27:82"  # hub extended address OUI part
LINKTYPE_IEEE802_15_4_TAP = 283


@dataclass
class Msg:
    ts: float
    src_is_hub: bool
    seq: int
    type: int
    payload: bytes  # application payload after the 802.15.4 header

    @property
    def who(self) -> str:
        return "hub" if self.src_is_hub else "flap"


def read_pcap(path: str) -> list[Msg]:
    data = open(path, "rb").read()
    if struct.unpack_from("<I", data, 0)[0] != 0xA1B2C3D4:
        raise ValueError("not a little-endian pcap")
    linktype = struct.unpack_from("<I", data, 20)[0]
    if linktype != LINKTYPE_IEEE802_15_4_TAP:
        raise ValueError(f"unexpected linktype {linktype}")
    out: list[Msg] = []
    off = 24
    while off + 16 <= len(data):
        sec, usec, caplen, _ = struct.unpack_from("<IIII", data, off)
        off += 16
        packet = data[off : off + caplen]
        off += caplen
        msg = _parse_tap(sec + usec / 1e6, packet)
        if msg is not None:
            out.append(msg)
    return out


def _parse_tap(ts: float, packet: bytes) -> "Msg | None":
    # TLV header: version, reserved, length (bytes), then the 802.15.4 frame.
    if len(packet) < 4:
        return None
    hdr_len = struct.unpack_from("<H", packet, 2)[0]
    frame = packet[hdr_len:]
    return _parse_frame(ts, frame)


def _parse_frame(ts: float, f: bytes) -> "Msg | None":
    if len(f) < 3:
        return None
    fcf = struct.unpack_from("<H", f)[0]
    dst_mode = (fcf >> 10) & 3
    src_mode = (fcf >> 14) & 3
    pan_compressed = (fcf >> 6) & 1
    off = 3
    pan = None
    src = None

    def take_addr(mode: int) -> "str | None":
        nonlocal off
        if mode == 2:
            a = f[off : off + 2][::-1].hex()
            off += 2
            return a
        if mode == 3:
            a = ":".join(f"{b:02x}" for b in f[off : off + 8][::-1])
            off += 8
            return a
        return None

    if dst_mode:
        pan = struct.unpack_from("<H", f, off)[0]
        off += 2
        take_addr(dst_mode)
    if src_mode:
        if not pan_compressed:
            pan = struct.unpack_from("<H", f, off)[0]
            off += 2
        src = take_addr(src_mode)

    if pan != PAN or src is None:
        return None
    payload = f[off:]
    if len(payload) < 2:
        return None
    return Msg(ts, src.startswith(HUB_SUFFIX), payload[1], payload[0], payload)


def split_sessions(msgs: list[Msg]) -> list[list[Msg]]:
    """A session is the run of messages between two 0x0e session-setup messages."""
    sessions: list[list[Msg]] = []
    cur: list[Msg] = []
    for m in msgs:
        if m.type == 0x0E:
            if cur:
                sessions.append(cur)
            cur = []
        else:
            cur.append(m)
    if cur:
        sessions.append(cur)
    return sessions


@dataclass
class Keystream:
    """Per-session keystream, applied to type 0x01 payloads from byte 2 on."""

    START = 2
    data: bytearray = field(default_factory=lambda: bytearray(128))
    known: bytearray = field(default_factory=lambda: bytearray(128))

    def learn(self, offset: int, cipher: int, plain: int) -> None:
        self.data[offset] = cipher ^ plain
        self.known[offset] = 1

    def decrypt(self, msg: Msg) -> "bytes | None":
        out = bytearray(msg.payload)
        for i in range(self.START, len(out)):
            if not self.known[i]:
                return None  # a gap, can't fully decrypt
            out[i] ^= self.data[i]
        return bytes(out)

    def decrypt_partial(self, msg: Msg) -> bytes:
        out = bytearray(msg.payload)
        for i in range(self.START, len(out)):
            if self.known[i]:
                out[i] ^= self.data[i]
            else:
                out[i] = 0  # mark unknown as 0, shown as '..' by the caller
        return bytes(out)


# Lock state values, from PetHubLocal's CatFlapLockState enum.
LOCK_STATES = {2: "normal", 3: "keepin", 4: "locked", 5: "keepout", 6: "unlocked"}


def hexdump_known(ks: Keystream, msg: Msg) -> str:
    out = []
    plain = ks.decrypt_partial(msg)
    for i, b in enumerate(msg.payload):
        if i < ks.START:
            out.append(f"[{b:02x}]")  # cleartext header
        elif ks.known[i]:
            out.append(f"{plain[i]:02x}")
        else:
            out.append("..")
    return " ".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcap")
    parser.add_argument(
        "--sessions", action="store_true", help="list sessions and their messages"
    )
    args = parser.parse_args()

    msgs = read_pcap(args.pcap)
    flap = sum(1 for m in msgs if not m.src_is_hub)
    print(f"{len(msgs)} SureFlap messages ({flap} from flap)", file=sys.stderr)

    if args.sessions:
        for i, sess in enumerate(split_sessions(msgs)):
            enc = [m for m in sess if m.type == 0x01]
            if not enc:
                continue
            start = time.strftime("%H:%M:%S", time.localtime(sess[0].ts))
            end = time.strftime("%H:%M:%S", time.localtime(sess[-1].ts))
            print(f"\nsession {i}: {start}-{end}  {len(enc)} encrypted messages")
            for m in enc:
                hms = time.strftime("%H:%M:%S", time.localtime(m.ts))
                print(f"  {hms} {m.who:4} seq{m.seq:3} {m.payload.hex(' ')}")


if __name__ == "__main__":
    main()
