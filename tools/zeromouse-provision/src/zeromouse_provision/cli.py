"""Install and provision ZeroMOUSE cat flaps (ESP32-S3).

  backup   Check the chip and save a verified full flash backup.
  install  Backup, then replace the stock firmware and provision the device.
  restore  Write a backup (or a raw 16 MiB image) back to the chip.
  cloud    Send the stock cloud credentials over the API (from an nvs.bin).
  s3       Send the S3 keys over the API.
  clear    Remove all stored credentials over the API.

The API key for cloud, s3 and clear comes from <device>_api_encryption_key
in the secrets file, or from ZM_API_KEY.
"""

import argparse
import asyncio
import getpass
import os
import re
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path

import yaml
from aioesphomeapi.client import ExecuteServiceDataType

from zeromouse_provision import api, flasher
from zeromouse_provision.common import ProvisionError
from zeromouse_provision.improv import ImprovClient
from zeromouse_provision.nvs import (
    StockIdentity,
    identity_from_image,
    parse_nvs,
    stock_identity,
)

MAC = re.compile(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", re.IGNORECASE)
APP = "nix run .#zeromouse-provision --"


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


def entity_names(key: str) -> set[str]:
    translations = repo_root() / "projects/zeromouse/translations"
    names: set[str] = set()
    for language in ("de", "en"):
        data = yaml.safe_load((translations / f"{language}.yaml").read_text())
        if not isinstance(data, dict) or not isinstance(data.get(key), str):
            raise ProvisionError(f"The {language} translation has no {key}")
        names.add(data[key])
    return names


def cloud_values(
    identity: StockIdentity, *, keep_counters: bool = False
) -> ExecuteServiceDataType:
    return {
        "thing_name": identity.thing_name,
        "certificate": identity.certificate,
        "private_key": identity.private_key,
        "owner_id": identity.owner_id,
        "identity_id": identity.identity_id,
        "event_count": -1 if keep_counters else identity.event_count,
        "boot_count": -1 if keep_counters else identity.boot_count,
    }


def s3_values(secrets: dict[str, str], device: str) -> ExecuteServiceDataType | None:
    key_id, secret = f"{device}_s3_access_key_id", f"{device}_s3_secret_access_key"
    if key_id in secrets and secret in secrets:
        return {"access_key_id": secrets[key_id], "secret_access_key": secrets[secret]}
    return None


def wait_for_port(port: str, mac: str, timeout: float) -> str:
    import serial
    from serial.tools import list_ports

    time.sleep(2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(port):
            try:
                serial.Serial(port).close()
                return port
            except (OSError, serial.SerialException):
                pass
        else:
            same = [
                p.device
                for p in list_ports.comports()
                if p.vid == 0x303A
                and (p.serial_number or "").lower().replace("-", ":") == mac.lower()
            ]
            if len(same) == 1:
                print(f"The serial port is now {same[0]}")
                return same[0]
        time.sleep(0.5)
    raise ProvisionError(
        f"The serial port {port} did not come back within {timeout:.0f} s"
    )


def improv_host(port: str, ssid: str, password: str, fallback: str | None) -> str:
    import serial

    link = serial.Serial()
    link.port = port
    link.baudrate = 115200
    link.timeout = 0.2
    link.dtr = False
    link.rts = False
    link.open()
    try:
        client = ImprovClient(link)
        print("Waiting for the Improv serial port")
        client.wait_authorized(90)
        print("Sending the Wi-Fi settings")
        result = client.send_wifi(ssid, password, 60)
        host = result.host()
        if host is None:
            name = client.device_name()
            if name:
                try:
                    host = socket.gethostbyname(f"{name}.local")
                except OSError:
                    host = None
    finally:
        link.close()
    host = host or fallback
    if host is None:
        raise ProvisionError(
            "The device joined Wi-Fi but its address is unknown. Pass --host with the IP address."
        )
    return host


async def api_phase(
    host: str,
    mac: str,
    identity: StockIdentity,
    s3: ExecuteServiceDataType | None,
    args: argparse.Namespace,
) -> None:
    api.quiet_api_logging()
    connected_names = entity_names("cloud_connected")
    print(f"Connecting to {host} with the default API key")
    session = await api.connect_first(host, mac)
    if args.observe_only:
        names = entity_names("block_enabled")
        print("Turning blocking off")
        _ = await api.set_switch(session, names, False)
        await session.close()
        if await api.read_switch(host, names):
            raise ProvisionError("The blocking switch is still on")
        print("  blocking is off")
        session = await api.connect_first(host, mac)
    print("Provisioning the cloud credentials")
    await api.run_action(session, "provision_cloud", cloud_values(identity))
    print("  device restarting")
    session = await api.wait_ready(
        host, cloud=True, s3=False, connected=True, connected_names=connected_names
    )
    print("  cloud credentials stored and AWS IoT connected")
    if s3 is not None:
        print("Provisioning the S3 credentials")
        await api.run_action(session, "provision_s3", s3)
        print("  device restarting")
        session = await api.wait_ready(
            host, cloud=True, s3=True, connected=True, connected_names=connected_names
        )
        print("  S3 credentials stored")
    else:
        print(
            "No S3 keys in the secrets file for this device, skipping S3 provisioning"
        )
    if args.s3_endpoint:
        for key, value in (
            ("s3_endpoint", args.s3_endpoint),
            ("s3_bucket", args.s3_bucket),
        ):
            names = entity_names(key)
            _ = await api.set_text(session, names, value)
            await session.close()
            if await api.read_text(host, names) != value:
                raise ProvisionError(f"The {key} setting was not stored")
            print(f"  {key} set")
            session = await api.connect_first(host, mac)
    await session.close()


def restore_command(port: str, directory: Path) -> str:
    return f"{APP} restore --port {shlex.quote(port)} --backup-dir {shlex.quote(str(directory))}"


def cmd_backup(args: argparse.Namespace) -> None:
    flasher.require_tools()
    directory = flasher.check_backup_dir(args.backup_dir)
    pre = flasher.preflight(args.port, args.board)
    _ = flasher.make_backup(args.port, directory, pre)


def cmd_install(args: argparse.Namespace, secrets: dict[str, str]) -> None:
    flasher.require_tools()
    if bool(args.s3_endpoint) != bool(args.s3_bucket):
        raise ProvisionError("Give both --s3-endpoint and --s3-bucket or neither")
    _ = flasher.check_firmware(args.firmware, args.board)
    print(
        f"Firmware {args.firmware} is an ESP32-S3 factory image of siku2.zeromouse for {args.board}"
    )
    if args.wifi_ssid:
        ssid, password = (
            args.wifi_ssid,
            getpass.getpass(f"Wi-Fi password for {args.wifi_ssid}: "),
        )
    else:
        if "wifi_ssid" not in secrets or "wifi_password" not in secrets:
            raise ProvisionError(
                "The secrets file needs wifi_ssid and wifi_password, or pass --wifi-ssid"
            )
        ssid, password = secrets["wifi_ssid"], secrets["wifi_password"]
    s3 = s3_values(secrets, args.device) if args.device else None
    directory = flasher.check_backup_dir(args.backup_dir)

    pre = flasher.preflight(args.port, args.board)
    image = flasher.make_backup(args.port, directory, pre)
    identity, table, nvs = identity_from_image(image.read_bytes())
    print(
        f"Stock firmware found: partition table {table:#x}, nvs {nvs.offset:#x}, "
        + f"thing name {identity.thing_name}, event count {identity.event_count}, boot count {identity.boot_count}"
    )
    restore = restore_command(args.port, directory)
    print(f"To undo everything later run: {restore}\n")

    port = args.port
    try:
        print("Erasing the flash")
        _ = flasher.esptool(
            port,
            "erase-flash",
            what="Erasing the flash",
            timeout=flasher.LONG_TIMEOUT,
            show=True,
        )
        print("Writing the firmware")
        flasher.write_image(port, 0, args.firmware, what="Writing the firmware")
        port = wait_for_port(port, pre.chip.mac, 40)
        host = improv_host(port, ssid, password, args.host)
        print(f"The device is on Wi-Fi at {host}")
        asyncio.run(api_phase(host, pre.chip.mac, identity, s3, args))
    except BaseException:
        print(
            "\nThe install did not finish and the device may be in a half state.\n"
            + f"The stock firmware is saved. Restore it with:\n  {restore}",
            file=sys.stderr,
        )
        raise

    print(
        "\nDone.\n"
        + f"  MAC:      {pre.chip.mac}\n"
        + f"  Backup:   {directory}\n"
        + f"  Restore:  {restore}\n"
        + "Next: adopt the device in Home Assistant now, which sets the API key."
    )


def cmd_restore(args: argparse.Namespace) -> None:
    flasher.require_tools()
    if args.backup_dir:
        manifest, image = flasher.load_backup(args.backup_dir)
        board_id = manifest["board_id"]
        assert isinstance(board_id, int)
        mac = str(manifest["mac"])
        allow = args.allow_other_device
    else:
        if not args.image or not args.expect_mac:
            raise ProvisionError(
                "Use --backup-dir, or --image together with --expect-mac"
            )
        if not MAC.fullmatch(args.expect_mac):
            raise ProvisionError("--expect-mac must look like aa:bb:cc:dd:ee:ff")
        image, mac, board_id, allow = args.image, args.expect_mac, None, False
    flasher.restore(args.port, image, mac, allow_other=allow, board_id=board_id)


def load_identity(args: argparse.Namespace) -> StockIdentity:
    try:
        content = parse_nvs(args.nvs.read_bytes())
    except OSError as error:
        raise ProvisionError(f"Cannot read {args.nvs}") from error
    identity = stock_identity(content)
    print(
        f"Thing name {identity.thing_name}, event count {identity.event_count}, boot count {identity.boot_count}"
    )
    return identity


async def api_action(
    host: str, api_key: str, action: str, values: ExecuteServiceDataType
) -> None:
    api.quiet_api_logging()
    session = api.Session(host, api_key)
    await session.connect(30)
    await api.run_action(session, action, values)
    print("Done. The device restarted.")


def cmd_api(args: argparse.Namespace, secrets: dict[str, str]) -> None:
    if not args.host or not args.device:
        raise ProvisionError(f"{args.command} needs --host and --device")
    api_key = os.environ.get("ZM_API_KEY") or secrets.get(
        f"{args.device}_api_encryption_key"
    )
    if not api_key:
        raise ProvisionError(
            f"No API key, set ZM_API_KEY or {args.device}_api_encryption_key"
        )
    if args.command == "cloud":
        values = cloud_values(load_identity(args), keep_counters=args.keep_counters)
        if args.dry_run:
            return
        asyncio.run(api_action(args.host, api_key, "provision_cloud", values))
    elif args.command == "s3":
        s3 = s3_values(secrets, args.device)
        if s3 is None:
            raise ProvisionError(
                f"The secrets file needs {args.device}_s3_access_key_id and {args.device}_s3_secret_access_key"
            )
        asyncio.run(api_action(args.host, api_key, "provision_s3", s3))
    else:
        asyncio.run(api_action(args.host, api_key, "clear_provisioning", {}))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--host",
        help="Device address for cloud, s3 and clear. For install, used if Improv gives none",
    )
    parser.add_argument("--device", help="Secret name prefix, for example kopi")
    parser.add_argument("--secrets", type=Path, help="Defaults to devices/secrets.yaml")
    commands = parser.add_subparsers(dest="command", required=True)

    def add(name: str, help_text: str) -> argparse.ArgumentParser:
        sub = commands.add_parser(name, help=help_text)
        _ = sub.add_argument(
            "--host", default=argparse.SUPPRESS, help=argparse.SUPPRESS
        )
        _ = sub.add_argument(
            "--secrets", type=Path, default=argparse.SUPPRESS, help=argparse.SUPPRESS
        )
        return sub

    def serial_args(sub: argparse.ArgumentParser) -> None:
        _ = sub.add_argument(
            "--port", required=True, help="Serial port of the USB connection"
        )

    backup = add("backup", "Check the chip and save a verified flash backup")
    serial_args(backup)
    _ = backup.add_argument("--backup-dir", type=Path, required=True)
    _ = backup.add_argument("--board", choices=["v1", "v2"])

    install = add("install", "Replace the stock firmware and provision the device")
    serial_args(install)
    _ = install.add_argument("--backup-dir", type=Path, required=True)
    _ = install.add_argument(
        "--firmware", type=Path, required=True, help="firmware.factory.bin"
    )
    _ = install.add_argument("--board", choices=["v1", "v2"], required=True)
    _ = install.add_argument(
        "--device",
        default=argparse.SUPPRESS,
        help="Secret name prefix, for example kopi",
    )
    _ = install.add_argument(
        "--wifi-ssid", help="Use this network instead of wifi_ssid from the secrets"
    )
    _ = install.add_argument(
        "--observe-only",
        action="store_true",
        help="Turn blocking off before the cloud starts",
    )
    _ = install.add_argument("--s3-endpoint")
    _ = install.add_argument("--s3-bucket")

    restore = add("restore", "Write a backup back to the chip")
    serial_args(restore)
    source = restore.add_mutually_exclusive_group(required=True)
    _ = source.add_argument("--backup-dir", type=Path)
    _ = source.add_argument(
        "--image", type=Path, help="Raw 16 MiB image without a manifest"
    )
    _ = restore.add_argument("--expect-mac", help="Required with --image")
    _ = restore.add_argument("--allow-other-device", action="store_true")

    cloud = add("cloud", "Send the stock cloud credentials over the API")
    _ = cloud.add_argument("--device", default=argparse.SUPPRESS)
    _ = cloud.add_argument(
        "--nvs", type=Path, required=True, help="nvs.bin split from a stock flash dump"
    )
    _ = cloud.add_argument(
        "--keep-counters",
        action="store_true",
        help="Keep the event and boot counters on the device",
    )
    _ = cloud.add_argument("--dry-run", action="store_true")
    for name, text in (
        ("s3", "Send the S3 keys from the secrets file over the API"),
        ("clear", "Remove all stored credentials over the API"),
    ):
        sub = add(name, text)
        _ = sub.add_argument("--device", default=argparse.SUPPRESS)
    return parser


def main() -> int:
    os.umask(0o077)
    args = build_parser().parse_args()
    try:
        secrets = (
            load_secrets(args.secrets)
            if args.command in ("install", "cloud", "s3", "clear")
            else {}
        )
        if args.command == "backup":
            cmd_backup(args)
        elif args.command == "install":
            cmd_install(args, secrets)
        elif args.command == "restore":
            cmd_restore(args)
        else:
            cmd_api(args, secrets)
    except ProvisionError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
