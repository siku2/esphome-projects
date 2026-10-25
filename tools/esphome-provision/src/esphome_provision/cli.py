"""Install ESPHome firmware on ESP32 devices with a verified backup, then provision them.

  backup   Check the chip and save a verified full flash backup.
  install  Backup, then erase and write the firmware, set up Wi-Fi and provision.
  restore  Write a backup (or a raw image) back to the chip.
  logs     Stream the log of an installed device.
  upload   Upload a firmware image to an installed device over the air.
  ha       Log in to or out of Home Assistant.

Profiles add project specific checks, steps and commands, for example
"zeromouse". The profile is picked from --profile or from the project name
in the firmware. Without a profile, install stops after the Wi-Fi setup and
a basic check over the API.

Backups hold device keys. Without --backup-dir they go to
backups/<name>-<date> in the current directory, where <name> is the device
name or the MAC address. A backup inside a git work tree must be ignored
there, or the work tree must not be the esphome-projects repository.

The API key defaults to all zeros. If the device already has a key, for
example from Home Assistant, the commands for installed devices look for it in
this order: --api-key, ESPHOME_API_KEY, then Home Assistant. Give its address
with --ha-url or ESPHOME_PROVISION_HA_URL. Log in once with "ha login", or set
ESPHOME_PROVISION_HA_TOKEN to a long-lived token. Only "install --resume",
"logs", "upload" and the profile commands read the key from Home Assistant.
"""

import argparse
import asyncio
import os
import shlex
import sys
from pathlib import Path

from esphome_provision import api, flasher, ha, logs, upload
from esphome_provision.common import KEY_ENV, MAC, ProvisionError
from esphome_provision.firmware import (
    Firmware,
    app_name,
    check_project,
    check_target,
    read_firmware,
)
from esphome_provision.profiles import Profile, all_profiles, detect_profile
from esphome_provision.secrets import load_secrets, wifi_credentials
from esphome_provision.wifi import improv_host, wait_for_port

APP = "nix run .#esphome-provision --"
PORT_PLACEHOLDER = "<port>"
HOST_PLACEHOLDER = "<device address>"
PROFILE_OPTIONS = ("board", "device", "observe_only", "s3_endpoint", "s3_bucket")


def select_profile(name: str | None, firmware: Firmware | None) -> Profile | None:
    profile: Profile | None
    if name:
        profile = all_profiles()[name]
        if firmware is not None:
            check_project(firmware, profile.project_name)
    else:
        profile = detect_profile(firmware.data) if firmware else None
    if profile:
        print(f"Using the {profile.name} profile")
    else:
        print("No profile applies, using the generic install")
    return profile


def require_profile_for_options(
    args: argparse.Namespace, profile: Profile | None
) -> None:
    if profile is not None:
        return
    given = [name for name in PROFILE_OPTIONS if getattr(args, name, None)]
    if given:
        options = ", ".join("--" + name.replace("_", "-") for name in given)
        raise ProvisionError(
            f"{options} only apply with a profile, but none was selected"
        )


def restore_command(port: str | None, directory: Path) -> str:
    where = shlex.quote(port) if port else PORT_PLACEHOLDER
    return f"{APP} restore --port {where} --backup-dir {shlex.quote(str(directory))}"


def resume_command(
    args: argparse.Namespace, profile: Profile | None, host: str | None
) -> str:
    parts = [
        "install",
        "--resume",
        "--backup-dir",
        shlex.quote(str(args.backup_dir)),
        "--host",
        shlex.quote(host) if host else HOST_PLACEHOLDER,
    ]
    if profile:
        parts += ["--profile", profile.name]
        parts += [shlex.quote(part) for part in profile.resume_arguments(args)]
    if args.secrets:
        parts += ["--secrets", shlex.quote(str(args.secrets))]
    if args.api_key:
        parts += ["--api-key", "<api key>"]
    if args.ha_url:
        parts += ["--ha-url", shlex.quote(args.ha_url)]
    return f"{APP} {' '.join(parts)}"


async def api_phase(
    host: str,
    mac: str,
    profile: Profile | None,
    context: object,
    args: argparse.Namespace,
    firmware: Firmware | None,
    credentials: ha.Credentials | None,
) -> None:
    api.quiet_api_logging()
    source = f"API key from {credentials.source}" if credentials else "default API key"
    print(f"Connecting to {host} with the {source}")
    factory = api.session_factory(
        host,
        credentials.key if credentials else api.ZERO_PSK,
        mac=mac,
        project_name=profile.project_name.decode() if profile else None,
        firmware=firmware.data if firmware and not profile else None,
        node_name=app_name(firmware) if firmware and not profile else None,
    )
    if profile:
        await profile.provision(factory, context, args)
    else:
        session = await api.retry_dropped(factory)
        await session.close()
        print("  the device answers over the API and matches the firmware")


def fail_hint(
    args: argparse.Namespace, profile: Profile | None, host: str | None, flashed: bool
) -> None:
    state = "may be in a half state" if flashed else "was not changed beyond the backup"
    print(
        f"\nThe install did not finish and the device {state}.\n"
        + "The original firmware is saved. Restore it with:\n"
        + f"  {restore_command(args.port, args.backup_dir)}\n"
        + "Once the new firmware runs on the device, continue with:\n"
        + f"  {resume_command(args, profile, host)}",
        file=sys.stderr,
    )


def profile_efuses(profile: Profile | None) -> tuple[str, ...]:
    return profile.efuse_names if profile else ()


def explicit_backup_dir(args: argparse.Namespace) -> Path | None:
    return flasher.check_backup_dir(args.backup_dir) if args.backup_dir else None


def choose_backup_dir(
    args: argparse.Namespace, directory: Path | None, pre: flasher.Preflight
) -> Path:
    """Use the checked --backup-dir or pick the default, and remember it for the hints."""
    if directory is None:
        name = getattr(args, "device", None) or pre.chip.mac.replace(":", "")
        directory = flasher.check_backup_dir(flasher.default_backup_dir(name))
    args.backup_dir = directory
    print(f"Backup directory: {directory}")
    return directory


def cmd_backup(args: argparse.Namespace) -> None:
    flasher.require_tools()
    directory = explicit_backup_dir(args)
    profile = select_profile(args.profile, None)
    pre = flasher.preflight(
        args.port,
        extra_efuses=profile_efuses(profile),
        check_chip=profile.check_chip if profile else None,
    )
    if profile:
        profile.check_device(pre, args)
    directory = choose_backup_dir(args, directory, pre)
    _ = flasher.make_backup(args.port, directory, pre)


def cmd_install(args: argparse.Namespace) -> None:
    if args.resume:
        cmd_resume(args)
        return
    if not args.port or not args.firmware:
        raise ProvisionError("install needs --port and --firmware")
    flasher.require_tools()
    firmware = read_firmware(args.firmware)
    profile = select_profile(args.profile, firmware)
    require_profile_for_options(args, profile)
    if profile:
        profile.check_firmware(firmware.data, args)
    secrets = load_secrets(args.secrets)
    ssid, password = wifi_credentials(secrets, args.wifi_ssid)
    directory = explicit_backup_dir(args)

    def check_chip(chip: flasher.ChipInfo) -> None:
        check_target(firmware, chip.chip, chip.flash_size)
        if profile:
            profile.check_chip(chip)

    pre = flasher.preflight(
        args.port, extra_efuses=profile_efuses(profile), check_chip=check_chip
    )
    if profile:
        profile.check_device(pre, args)
    directory = choose_backup_dir(args, directory, pre)
    print(f"Firmware {args.firmware} is a factory image for the {pre.chip.chip.label}")
    image = flasher.make_backup(args.port, directory, pre)
    context = profile.prepare(image.read_bytes(), args, secrets) if profile else None
    restore = restore_command(args.port, directory)
    print(f"To undo everything later run: {restore}\n")

    port, chip = args.port, pre.chip.chip.esptool_id
    host: str | None = args.host
    flashed = False
    try:
        print("Erasing the flash")
        flasher.erase_flash(port, chip)
        flashed = True
        print("Writing the firmware")
        flasher.write_image(port, chip, 0, args.firmware, what="Writing the firmware")
        port = wait_for_port(port, pre.chip.mac, 40)
        host = improv_host(port, ssid, password, args.host)
        print(f"The device is on Wi-Fi at {host}")
        credentials = ha.resolve_credentials(args.api_key, None)
        asyncio.run(
            api_phase(host, pre.chip.mac, profile, context, args, firmware, credentials)
        )
    except BaseException:
        fail_hint(args, profile, host, flashed)
        raise
    print_done(pre.chip.mac, directory, restore)


async def device_profile(
    args: argparse.Namespace, credentials: ha.Credentials | None
) -> str | None:
    """Name of the profile whose project name the device reports."""
    api.quiet_api_logging()
    session = api.Session(args.host, credentials.key if credentials else api.ZERO_PSK)
    await session.connect(30)
    try:
        project = (await session.device_info()).project_name
    finally:
        await session.close()
    for profile in all_profiles().values():
        if project == profile.project_name.decode():
            return profile.name
    return None


def cmd_resume(args: argparse.Namespace) -> None:
    if not args.host:
        raise ProvisionError("install --resume needs --host")
    if not args.backup_dir:
        raise ProvisionError("install --resume needs --backup-dir")
    backup = flasher.load_backup(args.backup_dir)
    credentials = ha.resolve_credentials(
        args.api_key, ha.ha_url(args.ha_url), mac=backup.mac
    )
    firmware = read_firmware(args.firmware) if args.firmware else None
    profile_name = args.profile
    if profile_name is None and firmware is None:
        profile_name = asyncio.run(device_profile(args, credentials))
    profile = select_profile(profile_name, firmware)
    require_profile_for_options(args, profile)
    if firmware is not None:
        check_target(firmware, backup.chip, backup.flash_size)
        if profile:
            profile.check_firmware(firmware.data, args)
    secrets = load_secrets(args.secrets) if profile else {}
    context = (
        profile.prepare(backup.image.read_bytes(), args, secrets) if profile else None
    )
    try:
        asyncio.run(
            api_phase(
                args.host, backup.mac, profile, context, args, firmware, credentials
            )
        )
    except BaseException:
        fail_hint(args, profile, args.host, True)
        raise
    print_done(backup.mac, args.backup_dir, restore_command(args.port, args.backup_dir))


def print_done(mac: str, directory: Path, restore: str) -> None:
    print(
        "\nDone.\n"
        + f"  MAC:      {mac}\n"
        + f"  Backup:   {directory}\n"
        + f"  Restore:  {restore}\n"
        + "Next: adopt the device in Home Assistant now, which sets the API key."
    )


def cmd_restore(args: argparse.Namespace) -> None:
    flasher.require_tools()
    backup = None
    if args.backup_dir:
        backup = flasher.load_backup(args.backup_dir)
        image, mac, allow = backup.image, backup.mac, args.allow_other_device
    else:
        if not args.image or not args.expect_mac:
            raise ProvisionError(
                "Use --backup-dir, or --image together with --expect-mac"
            )
        if not MAC.fullmatch(args.expect_mac):
            raise ProvisionError("--expect-mac must look like aa:bb:cc:dd:ee:ff")
        image, mac, allow = args.image, args.expect_mac, False
    flasher.restore(args.port, image, mac, allow_other=allow, backup=backup)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="esphome-provision",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True)
    profiles = all_profiles()

    def add_profile_options(sub: argparse.ArgumentParser) -> None:
        _ = sub.add_argument(
            "--profile",
            choices=sorted(profiles),
            help="Defaults to the profile whose project name is in the firmware",
        )
        for profile in profiles.values():
            profile.add_install_arguments(
                sub.add_argument_group(f"{profile.name} profile options")
            )

    backup = commands.add_parser(
        "backup", help="Check the chip and save a verified flash backup"
    )
    _ = backup.add_argument("--port", required=True, help="Serial port of the device")
    _ = backup.add_argument(
        "--backup-dir",
        type=Path,
        help="New directory for the backup. Defaults to backups/<name>-<date>",
    )
    add_profile_options(backup)
    backup.set_defaults(handler=cmd_backup)

    install = commands.add_parser(
        "install", help="Replace the firmware and provision the device"
    )
    _ = install.add_argument("--port", help="Serial port of the device")
    _ = install.add_argument(
        "--backup-dir",
        type=Path,
        help="New directory for the backup. Defaults to backups/<name>-<date>. "
        + "Required with --resume, where it is the existing backup",
    )
    _ = install.add_argument("--firmware", type=Path, help="firmware.factory.bin")
    _ = install.add_argument(
        "--resume",
        action="store_true",
        help="Skip the backup and flashing and continue with the provisioning over the API",
    )
    _ = install.add_argument(
        "--host",
        help="Device address. Used if Improv gives none, required with --resume",
    )
    _ = install.add_argument(
        "--api-key",
        help=f"Noise key of the device. Defaults to {KEY_ENV}, then with --resume "
        + "Home Assistant, then zeros",
    )
    ha.add_argument(install)
    _ = install.add_argument(
        "--wifi-ssid", help="Use this network instead of wifi_ssid from the secrets"
    )
    _ = install.add_argument(
        "--secrets", type=Path, help="Defaults to devices/secrets.yaml"
    )
    add_profile_options(install)
    install.set_defaults(handler=cmd_install)

    restore = commands.add_parser("restore", help="Write a backup back to the chip")
    _ = restore.add_argument("--port", required=True, help="Serial port of the device")
    source = restore.add_mutually_exclusive_group(required=True)
    _ = source.add_argument("--backup-dir", type=Path)
    _ = source.add_argument(
        "--image", type=Path, help="Raw image of the flash size, without a manifest"
    )
    _ = restore.add_argument("--expect-mac", help="Required with --image")
    _ = restore.add_argument("--allow-other-device", action="store_true")
    restore.set_defaults(handler=cmd_restore)

    logs.add_command(commands)
    upload.add_command(commands)
    ha.add_command(commands)
    for profile in profiles.values():
        profile.add_subcommands(commands)
    return parser


def main() -> int:
    os.umask(0o077)
    args = build_parser().parse_args()
    try:
        args.handler(args)
    except ProvisionError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
