"""Find the API key of an ESPHome device in Home Assistant."""

import argparse
import asyncio
import os
from dataclasses import dataclass
from typing import Any

import aiohttp

from esphome_provision import api, ha_auth, ha_socket
from esphome_provision.common import KEY_ENV, ProvisionError

URL_ENV = "ESPHOME_PROVISION_HA_URL"
FROM_HA = "Home Assistant"


@dataclass(frozen=True)
class Credentials:
    key: str
    source: str
    host: str | None = None


def ha_url(option: str | None) -> str | None:
    return option or os.environ.get(URL_ENV)


def has_mac(device: dict[str, Any], mac: str) -> bool:
    return any(
        len(pair) == 2 and pair[0] == "mac" and str(pair[1]).lower() == mac
        for pair in device.get("connections") or []
    )


def pick_entry(
    entries: list[dict[str, Any]], devices: list[dict[str, Any]], mac: str
) -> str:
    """Return the id of the one ESPHome config entry whose device has this MAC."""
    wanted = mac.lower()
    esphome = {str(e["entry_id"]) for e in entries}
    found = {
        str(entry_id)
        for device in devices
        if has_mac(device, wanted)
        for entry_id in device.get("config_entries") or []
        if str(entry_id) in esphome
    }
    if not found:
        raise ProvisionError(f"Home Assistant has no ESPHome device with MAC {mac}")
    if len(found) > 1:
        raise ProvisionError(
            f"Home Assistant has {len(found)} ESPHome entries for MAC {mac}"
        )
    return found.pop()


def pick_by_name(
    candidates: list[tuple[str, dict[str, Any]]], name: str
) -> tuple[str, str]:
    """Return the entry id and host of the one entry with this device name."""
    wanted = name.lower()
    found = [
        (entry_id, data)
        for entry_id, data in candidates
        if str(data.get("device_name") or "").lower() == wanted
    ]
    if not found:
        raise ProvisionError(f"Home Assistant has no ESPHome device named {name}")
    if len(found) > 1:
        raise ProvisionError(
            f"Home Assistant has {len(found)} ESPHome devices named {name}"
        )
    entry_id, data = found[0]
    host = data.get("host")
    if not host:
        raise ProvisionError(f"Home Assistant gives no host for {name}, pass --host")
    return entry_id, str(host)


async def lookup_key(
    url: str, mac: str | None, host: str | None, name: str | None
) -> tuple[str, str | None]:
    """Return the key and, when found by name, the host."""
    if mac is None and host is not None:
        device, mac = await api.discover(host)
        print(f"The device at {host} is {device} with MAC {mac}")
    if mac is None and name is None:
        raise ProvisionError(
            "Need a MAC, a host or a name to find the device in Home Assistant"
        )
    async with aiohttp.ClientSession() as http:
        token = await ha_auth.access_token(http, url)
        socket = await ha_socket.connect(http, url, token)
        try:
            connection = ha_socket.Connection(socket)
            entries = await ha_socket.esphome_entries(connection)
            found_host = None
            if mac is not None:
                devices = await ha_socket.devices(connection)
                entry_id = pick_entry(entries, devices, mac)
            else:
                assert name is not None
                ids = [str(e["entry_id"]) for e in entries]
                data = await asyncio.gather(
                    *(ha_socket.diagnostics(http, url, token, i) for i in ids)
                )
                entry_id, found_host = pick_by_name(list(zip(ids, data)), name)
                print(f"{name} is at {found_host}")
            key = await ha_socket.encryption_key(connection, entry_id)
        finally:
            _ = await socket.close()
    if not key:
        raise ProvisionError(
            f"The Home Assistant entry for {mac or name} has no encryption key"
        )
    return key, found_host


def resolve_credentials(
    api_key: str | None,
    url: str | None,
    *,
    mac: str | None = None,
    host: str | None = None,
    name: str | None = None,
) -> Credentials | None:
    """Pick the key from the option, the environment or Home Assistant, in that order."""
    if api_key:
        return Credentials(api_key, "--api-key")
    if key := os.environ.get(KEY_ENV):
        return Credentials(key, KEY_ENV)
    if url:
        key, found_host = asyncio.run(lookup_key(url, mac, host, name))
        return Credentials(key, FROM_HA, found_host or host)
    return None


def add_argument(parser: argparse.ArgumentParser) -> None:
    _ = parser.add_argument(
        "--ha-url",
        help=f"Home Assistant address to read the API key from. Defaults to {URL_ENV}",
    )


def require_url(args: argparse.Namespace) -> str:
    url = ha_url(args.ha_url)
    if not url:
        raise ProvisionError(f"Give --ha-url or set {URL_ENV}")
    return url


async def run_login(url: str) -> None:
    async with aiohttp.ClientSession() as http:
        _ = await ha_auth.login(http, url)


async def run_logout(url: str) -> None:
    async with aiohttp.ClientSession() as http:
        await ha_auth.logout(http, url)


def cmd_login(args: argparse.Namespace) -> None:
    asyncio.run(run_login(require_url(args)))


def cmd_logout(args: argparse.Namespace) -> None:
    asyncio.run(run_logout(require_url(args)))


def add_command(
    subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]",
) -> None:
    group = subparsers.add_parser("ha", help="Manage the Home Assistant login")
    commands = group.add_subparsers(dest="ha_command", required=True)
    login = commands.add_parser(
        "login", help="Log in to Home Assistant in the browser and remember the login"
    )
    add_argument(login)
    login.set_defaults(handler=cmd_login)
    logout = commands.add_parser("logout", help="Revoke the stored login and delete it")
    add_argument(logout)
    logout.set_defaults(handler=cmd_logout)
