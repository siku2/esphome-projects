"""Test whether the SureFlap session keystream is a weak function of the 0x0e nonce.

For each session (the run between two 0x0e messages) we take the 16 byte nonce
from the 0x0e payload and recover a few keystream bytes from known plaintext that
needs no ground truth:

- Movement frames (38 byte flap type 0x01): the FDX-B tag high bytes at payload
  offsets 28-30 are fixed (country 756 plus the shared high id bits), so
  keystream[o] = cipher[o] XOR {28:0xd7, 29:0x16, 30:0xbd}.

Offset 30 (0xbd) is fixed for any country-756 chip, so it is the most reliable.
Offsets 28-29 are shared by this household's two cats.

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
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import decode  # noqa: E402

# payload offset -> known plaintext byte, for 38 byte flap movement frames
MOVEMENT_LEN = 38
TAG_CRIB = {28: 0xD7, 29: 0x16, 30: 0xBD}


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


def recover_keystream_bytes(enc_msgs):
    """Return {offset: keystream_byte} recovered in this session, with a flag for
    whether every contributing frame agreed."""
    seen: dict[int, set] = {}
    for m in enc_msgs:
        if not m.src_is_hub and len(m.payload) == MOVEMENT_LEN:
            for off, plain in TAG_CRIB.items():
                seen.setdefault(off, set()).add(m.payload[off] ^ plain)
    ks = {}
    consistent = True
    for off, values in seen.items():
        if len(values) == 1:
            ks[off] = next(iter(values))
        else:
            consistent = False  # frames in one session disagree: crib or split is wrong
    return ks, consistent


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
    parser.add_argument("pcaps", nargs="+")
    parser.add_argument(
        "--holdout",
        type=float,
        default=0.3,
        help="fraction of sessions held out for validation",
    )
    args = parser.parse_args()

    samples = []  # (nonce_bytes, {offset: ks_byte})
    nonce_seen = {}
    inconsistent = 0
    for path in args.pcaps:
        try:
            msgs = decode.read_pcap(path)
        except Exception as e:
            print(f"skip {path}: {e}", file=sys.stderr)
            continue
        for nonce, enc in sessions_with_nonce(msgs):
            ks, ok = recover_keystream_bytes(enc)
            if not ok:
                inconsistent += 1
            if ks:
                samples.append((nonce, ks))
                nonce_seen.setdefault(nonce, []).append(ks)

    print(f"sessions with recovered keystream bytes: {len(samples)}")
    print(f"sessions with within-session crib disagreement: {inconsistent}")
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
            flag = (
                "  <== LINEAR" if acc > 0.95 else ("" if acc < 0.7 else "  (partial)")
            )
            print(
                f"  offset {off} bit {bit}: holdout accuracy {acc:.2f} on {len(te)} sessions{flag}"
            )


if __name__ == "__main__":
    main()
