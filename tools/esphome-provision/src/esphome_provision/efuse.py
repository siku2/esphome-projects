"""Parse espefuse output and check that a chip is safe to reflash."""

import json
import re
from collections.abc import Sequence
from dataclasses import dataclass

from esphome_provision.common import ProvisionError

EfuseValue = bool | str

SAFETY_NAMES = {
    "esp32s3": [
        "SPI_BOOT_CRYPT_CNT",
        "SECURE_BOOT_EN",
        "DIS_DOWNLOAD_MODE",
        "ENABLE_SECURITY_DOWNLOAD",
    ],
}
HEX_BYTES = re.compile(r"(?:[0-9a-fA-F]{2} ){31}[0-9a-fA-F]{2}")
ACCESS = r"(?:R/W|R/-|-/W|-/-)"


@dataclass(frozen=True)
class SafetyInfo:
    flash_encryption: str
    secure_boot: bool
    download_mode_disabled: bool
    security_download: bool


def safety_names(chip: str, label: str) -> list[str]:
    if chip not in SAFETY_NAMES:
        raise ProvisionError(
            f"The eFuse safety check is not implemented for the {label}, "
            + "so this tool refuses to flash it"
        )
    return SAFETY_NAMES[chip]


def _normalize(name: str, raw: object) -> EfuseValue:
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return raw == "True" if raw in ("True", "False") else raw.strip()
    raise ProvisionError(f"Cannot read the eFuse {name}, got an unexpected value")


def _flag(values: dict[str, EfuseValue], name: str) -> bool:
    value = values[name]
    if not isinstance(value, bool):
        raise ProvisionError(f"Cannot read the eFuse {name}, got an unexpected value")
    return value


def safety_info(values: dict[str, EfuseValue], names: Sequence[str]) -> SafetyInfo:
    missing = [name for name in names if name not in values]
    if missing:
        raise ProvisionError("The eFuse output has no " + ", ".join(missing))
    crypt = values["SPI_BOOT_CRYPT_CNT"]
    if not isinstance(crypt, str):
        raise ProvisionError("Cannot read the eFuse SPI_BOOT_CRYPT_CNT")
    return SafetyInfo(
        flash_encryption=crypt,
        secure_boot=_flag(values, "SECURE_BOOT_EN"),
        download_mode_disabled=_flag(values, "DIS_DOWNLOAD_MODE"),
        security_download=_flag(values, "ENABLE_SECURITY_DOWNLOAD"),
    )


def parse_summary_text(text: str, names: Sequence[str]) -> dict[str, EfuseValue]:
    """Parse the human readable `espefuse summary` output."""
    lines = text.splitlines()
    values: dict[str, EfuseValue] = {}
    for name in names:
        found = [i for i, line in enumerate(lines) if line.startswith(f"{name} (BLOCK")]
        if len(found) != 1:
            raise ProvisionError(
                f"The eFuse summary has {len(found)} entries for {name}"
            )
        line = lines[found[0]]
        match = re.match(r".* = (.+?)\s+" + ACCESS + r"\b", line)
        if match is None and found[0] + 1 < len(lines):
            match = re.match(
                r"\s+=\s+(" + HEX_BYTES.pattern + r")\s+" + ACCESS, lines[found[0] + 1]
            )
        if match is None:
            raise ProvisionError(f"Cannot read the eFuse {name} from the summary")
        values[name] = _normalize(name, match.group(1))
    return values


def parse_summary_json(text: str, names: Sequence[str]) -> dict[str, EfuseValue]:
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
    values: dict[str, EfuseValue] = {}
    for name in names:
        entry = data.get(name)
        if not isinstance(entry, dict) or "value" not in entry:
            raise ProvisionError(f"The espefuse JSON has no {name}")
        values[name] = _normalize(name, entry["value"])
    return values


def check_safe_to_flash(info: SafetyInfo) -> None:
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
