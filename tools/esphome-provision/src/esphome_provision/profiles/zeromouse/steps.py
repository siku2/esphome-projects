"""The API steps that provision a ZeroMOUSE device."""

import argparse
import asyncio
import re
from dataclasses import dataclass

from aioesphomeapi import BinarySensorInfo, BinarySensorState
from aioesphomeapi.client import ExecuteServiceDataType

from esphome_provision import api
from esphome_provision.api import Session, SessionFactory
from esphome_provision.common import ProvisionError
from esphome_provision.profiles.zeromouse.names import entity_names
from esphome_provision.profiles.zeromouse.nvs import StockIdentity

PROJECT_NAME = "siku2.zeromouse"
LOG_TAG = "zeromouse_provisioning"
CLOUD_CONNECTED_LINE = re.compile(r"\[aws_iot[^\]]*\]:\s*Connected\b")
# Time for the shadow delta to arrive after the cloud connects.
OBSERVE_SETTLE_S = 15
OBSERVE_SWITCHES = (
    ("accept_cloud_settings", "accepting cloud settings"),
    ("block_enabled", "blocking"),
)


@dataclass(frozen=True)
class Context:
    identity: StockIdentity
    s3: ExecuteServiceDataType | None


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


def dump_value(session: Session, label: str) -> str | None:
    pattern = re.compile(re.escape(label) + r":\s*(YES|NO)\b")
    for line in reversed(session.lines):
        match = pattern.search(line)
        if match:
            return match.group(1)
    return None


def cloud_connected_logged(session: Session) -> bool:
    return any(CLOUD_CONNECTED_LINE.search(line) for line in session.lines)


async def wait_ready(
    factory: SessionFactory,
    *,
    cloud: bool,
    s3: bool,
    connected: bool,
    connected_names: set[str],
    timeout: float = 150,
) -> None:
    """Reconnect after a restart and check the provisioning state from dump_config."""

    async def check(session: Session) -> None:
        entities, _ = await session.entities()
        sensor_key: int | None = None
        if connected:
            sensor_key = api.find_entity(
                entities, BinarySensorInfo, connected_names, "binary sensor"
            ).key
        session.subscribe(dump_config=True)
        for label, wanted in (("Cloud credentials", cloud), ("S3 credentials", s3)):
            if wanted:
                await session.wait_until(
                    lambda label=label: dump_value(session, label) is not None,
                    30,
                    label,
                )
                if dump_value(session, label) != "YES":
                    raise ProvisionError(
                        f"The device reports {label}: NO after provisioning"
                    )

        def is_connected() -> bool:
            state = session.states.get(sensor_key) if sensor_key is not None else None
            return cloud_connected_logged(session) or (
                isinstance(state, BinarySensorState) and state.state is True
            )

        if connected:
            await session.wait_until(is_connected, timeout, "the AWS IoT connection")

    await api.with_session(factory, check, timeout)


async def turn_observe_switches_off(factory: SessionFactory) -> None:
    # The cloud shadow would turn blocking back on unless its settings are ignored.
    for key, label in OBSERVE_SWITCHES:
        names = entity_names(key)
        print(f"Turning {label} off")
        await api.set_switch(factory, names, False)
        if await api.read_switch(factory, names):
            raise ProvisionError(f"The {key} switch is still on")
        print(f"  {label} is off")


async def check_observe_switches_stay_off(factory: SessionFactory) -> None:
    await asyncio.sleep(OBSERVE_SETTLE_S)
    for key, _ in OBSERVE_SWITCHES:
        if await api.read_switch(factory, entity_names(key)):
            raise ProvisionError(
                f"The {key} switch turned on again after the cloud connected"
            )
    print(
        "Blocking and accepting cloud settings are still off after the cloud connected"
    )


async def provision(
    factory: SessionFactory, context: Context, args: argparse.Namespace
) -> None:
    connected_names = entity_names("cloud_connected")
    if args.observe_only:
        await turn_observe_switches_off(factory)
    print("Provisioning the cloud credentials")
    await api.run_action(
        factory, "provision_cloud", cloud_values(context.identity), log_tag=LOG_TAG
    )
    print("  device restarting")
    await wait_ready(
        factory, cloud=True, s3=False, connected=True, connected_names=connected_names
    )
    print("  cloud credentials stored and AWS IoT connected")
    if context.s3 is not None:
        print("Provisioning the S3 credentials")
        await api.run_action(factory, "provision_s3", context.s3, log_tag=LOG_TAG)
        print("  device restarting")
        await wait_ready(
            factory,
            cloud=True,
            s3=True,
            connected=True,
            connected_names=connected_names,
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
            await api.set_text(factory, names, value)
            if await api.read_text(factory, names) != value:
                raise ProvisionError(f"The {key} setting was not stored")
            print(f"  {key} set")
    if args.observe_only:
        await check_observe_switches_stay_off(factory)
