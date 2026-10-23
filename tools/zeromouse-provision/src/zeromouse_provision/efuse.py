"""Parse espefuse output and check that a chip is safe to reflash."""

import json
import re
from dataclasses import dataclass

from zeromouse_provision.common import ProvisionError

BOARD_IDS = {"v1": 0, "v2": 1}
BOARD_NAMES = {0: "V1 (legacy)", 1: "V2", 2: "KKZM"}
NAMES = [
    "SPI_BOOT_CRYPT_CNT",
    "SECURE_BOOT_EN",
    "DIS_DOWNLOAD_MODE",
    "ENABLE_SECURITY_DOWNLOAD",
    "BLOCK_USR_DATA",
]
HEX_BYTES = re.compile(r"(?:[0-9a-fA-F]{2} ){31}[0-9a-fA-F]{2}")
ACCESS = r"(?:R/W|R/-|-/W|-/-)"


@dataclass(frozen=True)
class EfuseInfo:
    board_id: int
    flash_encryption: str
    secure_boot: bool
    download_mode_disabled: bool
    security_download: bool


def _bool(name: str, raw: object) -> bool:
    if isinstance(raw, bool):
        return raw
    if raw in ("True", "False"):
        return raw == "True"
    raise ProvisionError(f"Cannot read the eFuse {name}, got an unexpected value")


def _board_id(raw: object) -> int:
    if not isinstance(raw, str) or not HEX_BYTES.fullmatch(raw.strip()):
        raise ProvisionError(
            "Cannot read the eFuse BLOCK_USR_DATA, it is hidden or malformed"
        )
    return int(raw.split()[0], 16)


def _build(values: dict[str, object]) -> EfuseInfo:
    missing = [name for name in NAMES if name not in values]
    if missing:
        raise ProvisionError("The eFuse output has no " + ", ".join(missing))
    crypt = values["SPI_BOOT_CRYPT_CNT"]
    if not isinstance(crypt, str):
        raise ProvisionError("Cannot read the eFuse SPI_BOOT_CRYPT_CNT")
    return EfuseInfo(
        board_id=_board_id(values["BLOCK_USR_DATA"]),
        flash_encryption=crypt.strip(),
        secure_boot=_bool("SECURE_BOOT_EN", values["SECURE_BOOT_EN"]),
        download_mode_disabled=_bool("DIS_DOWNLOAD_MODE", values["DIS_DOWNLOAD_MODE"]),
        security_download=_bool(
            "ENABLE_SECURITY_DOWNLOAD", values["ENABLE_SECURITY_DOWNLOAD"]
        ),
    )


def parse_summary_text(text: str) -> EfuseInfo:
    """Parse the human readable `espefuse summary` output."""
    lines = text.splitlines()
    values: dict[str, object] = {}
    for name in NAMES:
        found = [i for i, line in enumerate(lines) if line.startswith(f"{name} (BLOCK")]
        if len(found) != 1:
            raise ProvisionError(
                f"The eFuse summary has {len(found)} entries for {name}"
            )
        line = lines[found[0]]
        if name == "BLOCK_USR_DATA":
            match = re.match(
                r"\s+=\s+(" + HEX_BYTES.pattern + r")\s+" + ACCESS, lines[found[0] + 1]
            )
        else:
            match = re.match(r".* = (.+?)\s+" + ACCESS + r"\b", line)
        if match is None:
            raise ProvisionError(f"Cannot read the eFuse {name} from the summary")
        values[name] = match.group(1)
    return _build(values)


def parse_summary_json(text: str) -> EfuseInfo:
    """Parse the output of `espefuse summary --format json`, which has banner lines first."""
    if text.lstrip().startswith("{"):
        body = text.lstrip()
    else:
        start = text.find("\n{")
        if start < 0:
            raise ProvisionError("The espefuse output has no JSON")
        body = text[start + 1 :]
    try:
        data, _ = json.JSONDecoder().raw_decode(body)
    except json.JSONDecodeError as error:
        raise ProvisionError("The espefuse JSON output is malformed") from error
    if not isinstance(data, dict):
        raise ProvisionError("The espefuse JSON output is malformed")
    values: dict[str, object] = {}
    for name in NAMES:
        entry = data.get(name)
        if not isinstance(entry, dict) or "value" not in entry:
            raise ProvisionError(f"The espefuse JSON has no {name}")
        values[name] = entry["value"]
    return _build(values)


def check_safe_to_flash(info: EfuseInfo) -> None:
    if info.flash_encryption != "Disable":
        raise ProvisionError(
            "Flash encryption is enabled. This tool only supports plain flash."
        )
    if info.secure_boot:
        raise ProvisionError(
            "Secure boot is enabled. This tool only supports unsigned images."
        )
    if info.download_mode_disabled:
        raise ProvisionError("The ROM download mode is disabled.")
    if info.security_download:
        raise ProvisionError(
            "Secure download mode is enabled, flash reads are not possible."
        )


def check_board(info: EfuseInfo, board: str) -> None:
    expected = BOARD_IDS[board]
    if info.board_id != expected:
        found = BOARD_NAMES.get(info.board_id, f"unknown id {info.board_id}")
        raise ProvisionError(f"The board id says {found} but --board {board} was given")
