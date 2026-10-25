"""Upload a firmware image over the air with the API key from Home Assistant."""

import argparse
import asyncio
import base64
import time
from pathlib import Path

from aioesphomeapi.core import APIConnectionError
from esphome import espota2  # type: ignore[import-untyped]

from esphome_provision import api, ha
from esphome_provision.common import KEY_ENV, ProvisionError

OTA_PORT = 3232
BOOT_TIMEOUT = 120
POLL_INTERVAL = 2


def ota_password(key: str) -> str:
    """Return the OTA password the device derives from its API key."""
    return base64.b64decode(key).hex()


def default_firmware(device: str) -> Path:
    return Path("devices/.esphome/build") / device / "build" / "firmware.ota.bin"


def check_booted(before: str, after: str) -> None:
    if before == after:
        raise ProvisionError(
            f"The device still reports the compilation time {after}, "
            + "so the new image did not boot. It may have stayed on the old image."
        )


async def read_compilation_time(host: str, key: str, timeout: float) -> str:
    session = api.Session(host, key)
    await session.connect(timeout)
    try:
        return (await session.device_info()).compilation_time
    finally:
        await session.close()


async def wait_for_new_image(host: str, key: str, before: str) -> str:
    """Poll until the device reports another compilation time or the timeout passes."""
    deadline = time.monotonic() + BOOT_TIMEOUT
    after = before
    while time.monotonic() < deadline and after == before:
        await asyncio.sleep(POLL_INTERVAL)
        try:
            after = await read_compilation_time(host, key, POLL_INTERVAL * 5)
        except (api.ConnectTimeout, APIConnectionError):
            continue
    return after


def send(host: str, key: str, firmware: Path) -> None:
    rc, _ = espota2.run_ota(host, OTA_PORT, ota_password(key), firmware, noise_psk=key)
    if rc != 0:
        raise ProvisionError(f"The upload to {host} failed")


async def upload(host: str, key: str, firmware: Path) -> None:
    before = await read_compilation_time(host, key, 30)
    print(f"Uploading {firmware} to {host}, running image built {before}", flush=True)
    await asyncio.to_thread(send, host, key, firmware)
    print("Upload finished, waiting for the device to restart", flush=True)
    after = await wait_for_new_image(host, key, before)
    check_booted(before, after)
    print(f"The device runs the new image, built {after}")


def cmd_upload(args: argparse.Namespace) -> None:
    firmware: Path = args.firmware or default_firmware(args.device)
    if not firmware.is_file():
        raise ProvisionError(
            f"Firmware not found: {firmware}. Build it or pass --firmware"
        )
    credentials = ha.resolve_credentials(
        args.api_key, ha.ha_url(args.ha_url), host=args.host, name=args.device
    )
    if credentials is None:
        raise ProvisionError(
            f"No API key, pass --api-key, set {KEY_ENV} or give --ha-url "
            + f"(or set {ha.URL_ENV}) to read it from Home Assistant"
        )
    host = credentials.host or args.host
    if not host:
        raise ProvisionError(
            "upload needs --host unless the key comes from Home Assistant"
        )
    print(f"API key from {credentials.source}")
    api.quiet_api_logging()
    asyncio.run(upload(host, credentials.key, firmware))


def add_command(
    subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]",
) -> None:
    upload = subparsers.add_parser(
        "upload", help="Upload a firmware image to an installed device over the air"
    )
    _ = upload.add_argument("device", help="Device name, for example bean-kopi")
    _ = upload.add_argument(
        "--firmware",
        type=Path,
        help="OTA image. Defaults to devices/.esphome/build/<device>/build/firmware.ota.bin",
    )
    _ = upload.add_argument(
        "--host", help="Device address. Defaults to the host in Home Assistant"
    )
    _ = upload.add_argument(
        "--api-key", help=f"Noise key of the device. Defaults to {KEY_ENV}"
    )
    ha.add_argument(upload)
    upload.set_defaults(handler=cmd_upload)
