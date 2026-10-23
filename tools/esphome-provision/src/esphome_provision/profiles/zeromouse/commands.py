"""The zeromouse subcommands that talk to a device over the API."""

import argparse
import asyncio
import os
from pathlib import Path

from aioesphomeapi.client import ExecuteServiceDataType

from esphome_provision import api
from esphome_provision.common import KEY_ENV, ProvisionError
from esphome_provision.flasher import load_backup
from esphome_provision.profiles.zeromouse.nvs import (
    StockIdentity,
    identity_from_image,
    parse_nvs,
    stock_identity,
)
from esphome_provision.profiles.zeromouse.steps import (
    LOG_TAG,
    PROJECT_NAME,
    cloud_values,
    s3_values,
)
from esphome_provision.secrets import load_secrets


def load_identity(args: argparse.Namespace) -> StockIdentity:
    if args.nvs:
        try:
            identity = stock_identity(parse_nvs(args.nvs.read_bytes()))
        except OSError as error:
            raise ProvisionError(f"Cannot read {args.nvs}") from error
    else:
        backup = load_backup(args.backup_dir)
        identity, _, _ = identity_from_image(backup.image.read_bytes())
    print(
        f"Thing name {identity.thing_name}, event count {identity.event_count}, boot count {identity.boot_count}"
    )
    return identity


def api_key(args: argparse.Namespace, secrets: dict[str, str] | None) -> str:
    key = args.api_key or os.environ.get(KEY_ENV)
    if key:
        return key
    if args.device:
        secrets = secrets if secrets is not None else load_secrets(args.secrets)
        key = secrets.get(f"{args.device}_api_encryption_key")
    if not key:
        raise ProvisionError(
            f"No API key, pass --api-key, set {KEY_ENV} or give --device "
            + "with <device>_api_encryption_key in the secrets"
        )
    return key


async def run(
    args: argparse.Namespace,
    secrets: dict[str, str] | None,
    action: str,
    values: ExecuteServiceDataType,
) -> None:
    api.quiet_api_logging()
    factory = api.session_factory(
        args.host,
        api_key(args, secrets),
        mac=None,
        project_name=PROJECT_NAME,
        default_timeout=30,
    )
    await api.run_action(factory, action, values, log_tag=LOG_TAG)
    print("Done. The device restarted.")


def cmd_cloud(args: argparse.Namespace) -> None:
    values = cloud_values(load_identity(args), keep_counters=args.keep_counters)
    if args.dry_run:
        return
    asyncio.run(run(args, None, "provision_cloud", values))


def cmd_s3(args: argparse.Namespace) -> None:
    if not args.device:
        raise ProvisionError("s3 needs --device")
    secrets = load_secrets(args.secrets)
    values = s3_values(secrets, args.device)
    if values is None:
        raise ProvisionError(
            f"The secrets file needs {args.device}_s3_access_key_id and {args.device}_s3_secret_access_key"
        )
    asyncio.run(run(args, secrets, "provision_s3", values))


def cmd_clear(args: argparse.Namespace) -> None:
    asyncio.run(run(args, None, "clear_provisioning", {}))


def add_subcommands(
    subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]",
) -> None:
    group = subparsers.add_parser(
        "zeromouse",
        help="ZeroMOUSE commands that use the API of an installed device",
        description="The API key comes from --api-key, from ESPHOME_API_KEY, or from <device>_api_encryption_key in the secrets file.",
    )
    commands = group.add_subparsers(dest="zeromouse_command", required=True)

    def add(name: str, help_text: str) -> argparse.ArgumentParser:
        sub = commands.add_parser(name, help=help_text)
        _ = sub.add_argument("--host", required=True, help="Address of the device")
        _ = sub.add_argument("--device", help="Secret name prefix, for example kopi")
        _ = sub.add_argument("--api-key", help=f"Defaults to {KEY_ENV}")
        _ = sub.add_argument(
            "--secrets", type=Path, help="Defaults to devices/secrets.yaml"
        )
        return sub

    cloud = add("cloud", "Send the stock cloud credentials over the API")
    source = cloud.add_mutually_exclusive_group(required=True)
    _ = source.add_argument(
        "--nvs", type=Path, help="nvs.bin split from a stock flash dump"
    )
    _ = source.add_argument(
        "--backup-dir", type=Path, help="Backup that holds the stock flash"
    )
    _ = cloud.add_argument(
        "--keep-counters",
        action="store_true",
        help="Keep the event and boot counters on the device",
    )
    _ = cloud.add_argument("--dry-run", action="store_true")
    cloud.set_defaults(handler=cmd_cloud)
    s3 = add("s3", "Send the S3 keys from the secrets file over the API")
    s3.set_defaults(handler=cmd_s3)
    clear = add("clear", "Remove all stored credentials over the API")
    clear.set_defaults(handler=cmd_clear)
