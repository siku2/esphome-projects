"""Stream the log of a device over the API."""

import argparse
import asyncio

from aioesphomeapi import LogLevel

from esphome_provision import api, ha
from esphome_provision.common import KEY_ENV, ProvisionError

LEVELS = ("none", "error", "warn", "info", "config", "debug", "verbose", "very_verbose")
FIRST_CONNECT_TIMEOUT = 30
RECONNECT_TIMEOUT = 60


def print_line(line: str) -> None:
    print(line, flush=True)


async def stream(host: str, psk: str, level: LogLevel) -> None:
    """Print log lines until cancelled. Reconnect when the device restarts or drops."""
    first = True
    while True:
        session = api.Session(host, psk, on_line=print_line)
        timeout = FIRST_CONNECT_TIMEOUT if first else RECONNECT_TIMEOUT
        try:
            await session.connect(timeout)
        except api.ConnectTimeout:
            if first:
                raise
            continue
        try:
            if not first:
                print_line(f"--- Reconnected to {host}")
            session.subscribe_logs(dump_config=first, level=level)
            first = False
            await session.wait_stopped()
        finally:
            await session.close()
        print_line(f"--- Connection to {host} lost, reconnecting")


def cmd_logs(args: argparse.Namespace) -> None:
    credentials = ha.resolve_credentials(
        args.api_key, ha.ssh_target(args.ha_ssh), name=args.device
    )
    if credentials is None:
        raise ProvisionError(
            f"No API key, pass --api-key, set {KEY_ENV} or give --ha-ssh "
            + f"(or set {ha.SSH_ENV}) to read it from Home Assistant"
        )
    host = args.host or credentials.host
    if not host:
        raise ProvisionError(
            "logs needs --host unless the key comes from Home Assistant"
        )
    print(f"API key from {credentials.source}")
    api.quiet_api_logging()
    level = LogLevel[f"LOG_LEVEL_{args.level.upper()}"]
    asyncio.run(stream(host, credentials.key, level))


def add_command(
    subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]",
) -> None:
    logs = subparsers.add_parser("logs", help="Stream the log of an installed device")
    _ = logs.add_argument("device", help="Device name, for example bean-kopi")
    _ = logs.add_argument(
        "--host", help="Device address. Defaults to the host in Home Assistant"
    )
    _ = logs.add_argument(
        "--api-key", help=f"Noise key of the device. Defaults to {KEY_ENV}"
    )
    _ = logs.add_argument(
        "--level",
        choices=LEVELS,
        default="very_verbose",
        help="The device limits this to its own maximum level",
    )
    ha.add_argument(logs)
    logs.set_defaults(handler=cmd_logs)
