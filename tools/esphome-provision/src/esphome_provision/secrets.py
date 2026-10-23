"""Read the secrets file and look up Wi-Fi credentials."""

import getpass
import subprocess
from pathlib import Path

import yaml

from esphome_provision.common import ProvisionError


def repo_root() -> Path:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise ProvisionError(
            "Run this from inside the esphome-projects repository"
        ) from error
    return Path(result.stdout.strip())


def load_secrets(path: Path | None) -> dict[str, str]:
    """Read the secrets file, by default devices/secrets.yaml of the current repository."""
    path = path or repo_root() / "devices/secrets.yaml"
    if not path.exists():
        raise ProvisionError(f"The secrets file {path} does not exist")
    data = yaml.safe_load(path.read_text())
    return {str(k): str(v) for k, v in data.items()} if isinstance(data, dict) else {}


def wifi_credentials(secrets: dict[str, str], ssid: str | None) -> tuple[str, str]:
    """Return the network from --wifi-ssid with a prompted password, or from the secrets."""
    if ssid:
        return ssid, getpass.getpass(f"Wi-Fi password for {ssid}: ")
    if "wifi_ssid" not in secrets or "wifi_password" not in secrets:
        raise ProvisionError(
            "The secrets file needs wifi_ssid and wifi_password, or pass --wifi-ssid"
        )
    return secrets["wifi_ssid"], secrets["wifi_password"]
