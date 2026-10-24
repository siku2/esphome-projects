"""Look up the API key of an ESPHome device in Home Assistant over SSH."""

import argparse
import inspect
import json
import os
import subprocess
from dataclasses import dataclass
from typing import Any

from esphome_provision.common import KEY_ENV, ProvisionError

SSH_ENV = "ESPHOME_PROVISION_HA_SSH"
ENTRIES_FILE = "/config/.storage/core.config_entries"
FROM_HA = "Home Assistant"


def find_entries(
    entries: list[dict[str, Any]],
    name: str | None,
    mac: str | None,
    host: str | None,
) -> list[dict[str, Any]]:
    """Return the ESPHome entries that match the name, the MAC or the host."""
    wanted = {
        "device_name": name,
        "unique_id": mac,
        "host": host,
    }
    found = []
    for entry in entries:
        if entry.get("domain") != "esphome":
            continue
        values = {**entry.get("data", {}), "unique_id": entry.get("unique_id")}
        for field, value in wanted.items():
            current = values.get(field)
            if value and current and str(current).lower() == value.lower():
                found.append(entry)
                break
    return found


REMOTE_TAIL = f"""
import json

with open({ENTRIES_FILE!r}) as handle:
    entries = json.load(handle)["data"]["entries"]
query = json.loads(QUERY)
found = find_entries(entries, query["name"], query["mac"], query["host"])
result = {{"count": len(found), "entry": None}}
if len(found) == 1:
    data = found[0]["data"]
    result["entry"] = {{
        "device_name": data.get("device_name"),
        "host": data.get("host"),
        "noise_psk": data.get("noise_psk"),
    }}
print(json.dumps(result))
"""


def remote_script(name: str | None, mac: str | None, host: str | None) -> str:
    query = json.dumps({"name": name, "mac": mac, "host": host})
    return (
        "from __future__ import annotations\n"
        + "from typing import Any\n"
        + f"QUERY = {json.dumps(query)}\n"
        + inspect.getsource(find_entries)
        + REMOTE_TAIL
    )


@dataclass(frozen=True)
class Credentials:
    key: str
    source: str
    host: str | None = None


def searched(name: str | None, mac: str | None, host: str | None) -> str:
    parts = [
        f"{label} {value}"
        for label, value in (("name", name), ("MAC", mac), ("host", host))
        if value
    ]
    return ", ".join(parts)


def lookup(
    target: str, *, name: str | None, mac: str | None, host: str | None
) -> Credentials:
    """Ask Home Assistant for the key. Only the one matching entry leaves the host."""
    what = searched(name, mac, host)
    if not what:
        raise ProvisionError("Nothing to look up in Home Assistant")
    try:
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", target, "python3", "-"],
            input=remote_script(name, mac, host),
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise ProvisionError(
            f"Could not run ssh to {target}: {type(error).__name__}"
        ) from error
    if result.returncode != 0:
        detail = result.stderr.strip().splitlines()[-1:] or ["no error output"]
        raise ProvisionError(
            f"Looking up {what} on {target} failed (exit {result.returncode}): {detail[0]}"
        )
    try:
        answer = json.loads(result.stdout)
        count = int(answer["count"])
    except (ValueError, KeyError, TypeError) as error:
        raise ProvisionError(
            f"Home Assistant on {target} gave an unexpected answer"
        ) from error
    if count != 1:
        raise ProvisionError(
            f"Found {count} ESPHome devices in Home Assistant for {what}, expected one"
        )
    entry = answer["entry"]
    if not entry.get("noise_psk"):
        raise ProvisionError(
            f"The Home Assistant entry for {what} has no encryption key"
        )
    return Credentials(entry["noise_psk"], FROM_HA, entry.get("host"))


def ssh_target(option: str | None) -> str | None:
    return option or os.environ.get(SSH_ENV)


def resolve_credentials(
    api_key: str | None,
    ssh: str | None,
    *,
    name: str | None = None,
    mac: str | None = None,
    host: str | None = None,
) -> Credentials | None:
    """Pick the key from the option, the environment or Home Assistant, in that order."""
    if api_key:
        return Credentials(api_key, "--api-key")
    if key := os.environ.get(KEY_ENV):
        return Credentials(key, KEY_ENV)
    if ssh:
        return lookup(ssh, name=name, mac=mac, host=host)
    return None


def add_argument(parser: argparse.ArgumentParser) -> None:
    _ = parser.add_argument(
        "--ha-ssh",
        help=f"SSH target of Home Assistant to read the API key from. Defaults to {SSH_ENV}",
    )
