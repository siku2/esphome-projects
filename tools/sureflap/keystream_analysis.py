"""Test whether the SureFlap session keystream is a weak function of the 0x0e nonce.

For each session (the run between two 0x0e messages) we take the 16 byte nonce
from the 0x0e payload and recover keystream bytes from the FDX-B tag field of flap
movement frames (38 byte flap type 0x01, tag at payload offsets 25-30).

Some flap poll/status frames are also 38 bytes but carry an all-zero tag, so a
plain crib confuses them with movements. With the household tags known (one hex tag
per line in scratch/sureflap-tags.txt, or the SUREFLAP_TAGS env as a comma list) we
classify each frame: for every candidate keystream we require all 38 byte frames to
decode to either a known tag or the all-zero status tag. A session is accepted only
when that resolves to a single keystream at the shared high bytes (offsets 28-30).
Single-cat sessions mixed with status frames stay 1-bit ambiguous and are skipped.

Without a tags file we fall back to the country-756 crib {28:0xd7, 29:0x16, 30:0xbd}
and require every 38 byte frame to agree.

With (nonce, keystream-bit) pairs from many sessions we fit an affine model over
GF(2): keystream_bit = XOR of a subset of nonce bits, plus a constant that absorbs
any fixed key. We fit on a training split and measure accuracy on held-out sessions.
~100% on held-out data means the derivation is linear (a shift-register style
generator) and the cipher is broken without the key. ~50% means it does not look
linear in the nonce.

Usage:
  python3 tools/sureflap/keystream_analysis.py scratch/radio-captures/**/*.pcap
"""

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import decode  # noqa: E402

# payload offset -> known plaintext byte, for 38 byte flap movement frames
MOVEMENT_LEN = 38
TAG_CRIB = {28: 0xD7, 29: 0x16, 30: 0xBD}
TAG_OFF = 25  # FDX-B tag starts here in a movement frame
ZERO_TAG = bytes(6)


def load_tags(path):
    """Return known FDX-B tags as 6 byte values. SUREFLAP_TAGS env wins over the file."""
    env = os.environ.get("SUREFLAP_TAGS")
    if env:
        return [bytes.fromhex(t.strip()) for t in env.split(",") if t.strip()]
    tags = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    tags.append(bytes.fromhex(line))
    except FileNotFoundError:
        pass
    return tags


def sessions_with_nonce(msgs):
    """Yield (nonce_bytes, [type-0x01 messages]) per session."""
    nonce = None
    enc = []
    for m in msgs:
        if m.type == 0x0E:
            if nonce is not None:
                yield nonce, enc
            nonce = bytes(m.payload[2:18]) if len(m.payload) >= 18 else None
            enc = []
        elif m.type == 0x01:
            enc.append(m)
    if nonce is not None:
        yield nonce, enc


def recover_keystream_bytes(enc_msgs, tags=()):
    """Return ({offset: keystream_byte}, status) for this session.

    status is "ok" (accepted), "ambiguous" (dropped, could not resolve) or "none"
    (no usable movement frame)."""
    frames = [
        m for m in enc_msgs if not m.src_is_hub and len(m.payload) == MOVEMENT_LEN
    ]
    if not frames:
        return {}, "none"
    if tags:
        return _recover_with_tags(frames, tags)
    return _recover_with_crib(frames)


def _recover_with_tags(frames, tags):
    known = set(tags) | {ZERO_TAG}
    cands = set()
    for anchor in frames:
        base = anchor.payload[TAG_OFF : TAG_OFF + 6]
        for tag in tags:
            ks6 = bytes(base[j] ^ tag[j] for j in range(6))
            if all(
                bytes(f.payload[TAG_OFF + j] ^ ks6[j] for j in range(6)) in known
                for f in frames
            ):
                cands.add(ks6)
    if not cands:
        return {}, "none"
    highs = {c[3:6] for c in cands}
    if len(highs) != 1:
        return {}, "ambiguous"  # single-cat plus status: keystream off by the tag
    ks6 = next(iter(cands))
    ks = {28: ks6[3], 29: ks6[4], 30: ks6[5]}
    if len({c[0:3] for c in cands}) == 1:  # low tag bytes also pinned
        ks[25], ks[26], ks[27] = ks6[0], ks6[1], ks6[2]
    return ks, "ok"


def _recover_with_crib(frames):
    seen: dict[int, set[int]] = {}
    for f in frames:
        for off, plain in TAG_CRIB.items():
            seen.setdefault(off, set()).add(f.payload[off] ^ plain)
    ks = {}
    status = "ok"
    for off, values in seen.items():
        if len(values) == 1:
            ks[off] = next(iter(values))
        else:
            status = "ambiguous"  # frames disagree: a non-movement 38 byte frame
    return ks, status


def bits_lsb(data: bytes) -> list[int]:
    return [(b >> i) & 1 for b in data for i in range(8)]


def solve_gf2(rows: list[int], targets: list[int], ncols: int):
    """Solve A w = b over GF(2). rows are bitmasks over ncols columns (bit j set
    means column j is 1). Returns a particular solution w (int bitmask) or None if
    inconsistent."""
    aug = [(rows[i] | (targets[i] << ncols)) for i in range(len(rows))]
    pivot_col_of_row = []
    r = 0
    for col in range(ncols):
        piv = next((i for i in range(r, len(aug)) if (aug[i] >> col) & 1), None)
        if piv is None:
            continue
        aug[r], aug[piv] = aug[piv], aug[r]
        for i in range(len(aug)):
            if i != r and (aug[i] >> col) & 1:
                aug[i] ^= aug[r]
        pivot_col_of_row.append(col)
        r += 1
        if r == len(aug):
            break
    for i in range(r, len(aug)):
        if aug[i] == (1 << ncols):  # 0 = 1
            return None
    w = 0
    for i, col in enumerate(pivot_col_of_row):
        if (aug[i] >> ncols) & 1:
            w |= 1 << col
    return w


def predict(w: int, features: int) -> int:
    return bin(w & features).count("1") & 1


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("pcaps", nargs="*")
    parser.add_argument(
        "--holdout",
        type=float,
        default=0.3,
        help="fraction of sessions held out for validation",
    )
    parser.add_argument(
        "--watch",
        metavar="DIR",
        help="scan all pcaps under DIR, append a summary to DIR/keystream-watch.log, "
        "and touch DIR/KEYSTREAM-LINEAR-HIT if any keystream bit looks linear",
    )
    parser.add_argument(
        "--tags",
        default="scratch/sureflap-tags.txt",
        help="file with known FDX-B tags, one hex value per line",
    )
    args = parser.parse_args()

    tags = load_tags(args.tags)

    if args.watch:
        args.pcaps = sorted(
            glob.glob(os.path.join(args.watch, "**", "*.pcap"), recursive=True)
        )
    if not args.pcaps:
        parser.error("no pcaps given (pass files or --watch DIR)")

    linear_hits = 0

    # Read every pcap as one continuous timeline. The recorder rotates files
    # hourly, so a session can start in one file and finish in the next. Parsing
    # files separately would drop those split sessions.
    msgs = []
    for path in args.pcaps:
        try:
            msgs.extend(decode.read_pcap(path))
        except Exception as e:
            print(f"skip {path}: {e}", file=sys.stderr)

    samples = []  # (nonce_bytes, {offset: ks_byte})
    nonce_seen = {}
    ambiguous = 0
    for nonce, enc in sessions_with_nonce(msgs):
        ks, status = recover_keystream_bytes(enc, tags)
        if status == "ambiguous":
            ambiguous += 1
        if ks:
            samples.append((nonce, ks))
            nonce_seen.setdefault(nonce, []).append(ks)

    print(f"tags loaded: {len(tags)} ({'classifier' if tags else 'country crib'})")
    print(f"sessions with recovered keystream bytes: {len(samples)}")
    print(f"sessions dropped as ambiguous: {ambiguous}")
    print(f"distinct nonces: {len(nonce_seen)}")
    dup_conflicts = sum(
        1
        for ks_list in nonce_seen.values()
        if len({tuple(sorted(k.items())) for k in ks_list}) > 1
    )
    print(
        f"repeated nonces giving different keystream: {dup_conflicts} (should be 0 if deterministic)"
    )

    offsets = sorted({o for _, ks in samples for o in ks})
    if len(samples) < 150:
        print(
            "\nnot enough sessions yet for a confident linearity test (want >=150). Keep capturing."
        )
    print(
        f"\naffine-over-GF(2) test of keystream bits vs the 128 nonce bits ({len(samples)} sessions):"
    )
    split = int(len(samples) * (1 - args.holdout))
    train, test = samples[:split], samples[split:]
    NCOLS = 129  # 128 nonce bits + 1 constant

    for off in offsets:
        for bit in range(8):
            tr = [(s, ks) for s, ks in train if off in ks]
            te = [(s, ks) for s, ks in test if off in ks]
            if len(tr) < NCOLS or len(te) < 10:
                continue
            rows = [
                (sum(b << i for i, b in enumerate(bits_lsb(s))) | (1 << 128))
                for s, _ in tr
            ]
            targets = [(ks[off] >> bit) & 1 for _, ks in tr]
            w = solve_gf2(rows, targets, NCOLS)
            if w is None:
                print(
                    f"  offset {off} bit {bit}: NO affine fit on training (inconsistent) -> not linear in nonce"
                )
                continue
            correct = 0
            for s, ks in te:
                feat = sum(b << i for i, b in enumerate(bits_lsb(s))) | (1 << 128)
                correct += predict(w, feat) == ((ks[off] >> bit) & 1)
            acc = correct / len(te)
            if acc > 0.95:
                linear_hits += 1
            flag = (
                "  <== LINEAR" if acc > 0.95 else ("" if acc < 0.7 else "  (partial)")
            )
            print(
                f"  offset {off} bit {bit}: holdout accuracy {acc:.2f} on {len(te)} sessions{flag}"
            )

    summary = f"SUMMARY sessions={len(samples)} distinct_nonces={len(nonce_seen)} linear_bits={linear_hits}"
    print("\n" + summary)

    if args.watch:
        with open(os.path.join(args.watch, "keystream-watch.log"), "a") as f:
            f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} {summary}\n")
        flag_path = os.path.join(args.watch, "KEYSTREAM-LINEAR-HIT")
        if linear_hits:
            with open(flag_path, "w") as f:
                f.write(summary + "\n")
        elif os.path.exists(flag_path):
            os.remove(flag_path)

    if linear_hits:
        sys.exit(10)


if __name__ == "__main__":
    main()
