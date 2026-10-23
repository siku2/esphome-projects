"""ESPHome native API steps for the ZeroMOUSE installer."""

import asyncio
import base64
import logging
import re
import time
from collections.abc import Callable
from typing import TypeVar

from aioesphomeapi import (
    APIClient,
    BinarySensorInfo,
    BinarySensorState,
    DeviceInfo,
    EntityInfo,
    EntityState,
    InvalidEncryptionKeyAPIError,
    LogLevel,
    RequiresEncryptionAPIError,
    SwitchInfo,
    SwitchState,
    TextInfo,
    TextState,
    UserService,
)
from aioesphomeapi.api_pb2 import SubscribeLogsResponse  # pyright: ignore[reportAttributeAccessIssue]
from aioesphomeapi.client import ExecuteServiceDataType
from aioesphomeapi.core import APIConnectionError

from zeromouse_provision.common import ProvisionError, strip_ansi

PORT = 6053
ZERO_PSK = base64.b64encode(bytes(32)).decode()
PROJECT_NAME = "siku2.zeromouse"
CLOUD_CONNECTED_LINE = re.compile(r"\[aws_iot[^\]]*\]:\s*Connected\b")

Info = TypeVar("Info", bound=EntityInfo)


def quiet_api_logging() -> None:
    logging.getLogger("aioesphomeapi").setLevel(logging.CRITICAL)


class Session:
    def __init__(self, host: str, psk: str) -> None:
        self.host = host
        self.psk = psk
        self.lines: list[str] = []
        self.states: dict[int, EntityState] = {}
        self.client: APIClient | None = None
        self._changed = asyncio.Event()
        self._stopped = asyncio.Event()

    async def _on_stop(self, expected: bool) -> None:
        del expected
        self._stopped.set()
        self._changed.set()

    def _on_log(self, message: SubscribeLogsResponse) -> None:
        text = strip_ansi(bytes(message.message).decode("utf-8", errors="replace"))
        self.lines.extend(text.splitlines())
        self._changed.set()

    def _on_state(self, state: EntityState) -> None:
        self.states[state.key] = state
        self._changed.set()

    async def connect(self, timeout: float) -> None:
        """Connect, retrying while the device boots. Encryption errors are final."""
        deadline = time.monotonic() + timeout
        last: Exception | None = None
        while time.monotonic() < deadline:
            client = APIClient(self.host, PORT, None, noise_psk=self.psk)
            self._stopped.clear()
            try:
                await asyncio.wait_for(
                    client.connect(on_stop=self._on_stop, login=True, log_errors=False),
                    15,
                )
            except (InvalidEncryptionKeyAPIError, RequiresEncryptionAPIError) as error:
                raise ProvisionError(
                    f"The device at {self.host} rejects the all-zeros API key. It may already have an API key."
                ) from error
            except (APIConnectionError, TimeoutError, OSError) as error:
                last = error
                await asyncio.sleep(2)
                continue
            self.client = client
            return
        reason = f": {last}" if last else ""
        raise ProvisionError(
            f"Could not connect to {self.host}:{PORT} within {timeout:.0f} s{reason}"
        )

    def stopped(self) -> bool:
        return self._stopped.is_set()

    def require_client(self) -> APIClient:
        if self.client is None:
            raise ProvisionError("Not connected to the device")
        return self.client

    async def close(self) -> None:
        if self.client is not None:
            try:
                await self.client.disconnect()
            except APIConnectionError:
                pass

    async def device_info(self) -> DeviceInfo:
        return await self.require_client().device_info()

    async def entities(self) -> tuple[list[EntityInfo], list[UserService]]:
        return await self.require_client().list_entities_services()

    def subscribe(self, *, dump_config: bool) -> None:
        client = self.require_client()
        _ = client.subscribe_logs(self._on_log, LogLevel.LOG_LEVEL_DEBUG, dump_config)
        client.subscribe_states(self._on_state)

    async def wait_until(
        self, condition: Callable[[], bool], timeout: float, what: str
    ) -> None:
        deadline = time.monotonic() + timeout
        while not condition():
            if self._stopped.is_set():
                raise ProvisionError(f"The connection dropped while waiting for {what}")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProvisionError(
                    f"Timed out after {timeout:.0f} s waiting for {what}"
                )
            self._changed.clear()
            try:
                await asyncio.wait_for(self._changed.wait(), remaining)
            except TimeoutError:
                continue

    async def wait_for_restart(self, timeout: float, what: str) -> None:
        try:
            await asyncio.wait_for(self._stopped.wait(), timeout)
        except TimeoutError:
            errors = [
                line
                for line in self.lines
                if "zeromouse_provisioning" in line and line.startswith(("[E]", "[W]"))
            ]
            hint = " Device log: " + " | ".join(errors) if errors else ""
            raise ProvisionError(
                f"The device did not restart after {what}.{hint}"
            ) from None

    def dump_value(self, label: str) -> str | None:
        pattern = re.compile(re.escape(label) + r":\s*(YES|NO)\b")
        for line in reversed(self.lines):
            match = pattern.search(line)
            if match:
                return match.group(1)
        return None

    def cloud_connected_logged(self) -> bool:
        return any(CLOUD_CONNECTED_LINE.search(line) for line in self.lines)


def find_entity(
    entities: list[EntityInfo], kind: type[Info], names: set[str], what: str
) -> Info:
    found = [e for e in entities if isinstance(e, kind) and e.name in names]
    if len(found) != 1:
        raise ProvisionError(
            f"Expected one {what} named {' or '.join(sorted(names))}, found {len(found)}"
        )
    return found[0]


def find_service(services: list[UserService], name: str) -> UserService:
    found = [s for s in services if s.name == name]
    if len(found) != 1:
        raise ProvisionError(f"The device has no {name} action")
    return found[0]


async def connect_first(
    host: str, expected_mac: str | None, timeout: float = 90
) -> Session:
    session = Session(host, ZERO_PSK)
    await session.connect(timeout)
    info = await session.device_info()
    if info.project_name != PROJECT_NAME:
        await session.close()
        raise ProvisionError(
            f"The device at {host} is not running {PROJECT_NAME} (project {info.project_name!r})"
        )
    if expected_mac is not None and info.mac_address.lower() != expected_mac.lower():
        await session.close()
        raise ProvisionError(
            f"The device at {host} has a different MAC address than the flashed chip"
        )
    return session


async def run_action(
    session: Session, action: str, values: ExecuteServiceDataType
) -> None:
    """Call an action and wait until the device restarts."""
    session.subscribe(dump_config=False)
    _, services = await session.entities()
    service = find_service(services, action)
    try:
        _ = await session.require_client().execute_service(service, values)
    except APIConnectionError:
        if not session.stopped():
            raise
    await session.wait_for_restart(30, action)
    await session.close()


async def wait_ready(
    host: str,
    *,
    cloud: bool,
    s3: bool,
    connected: bool,
    connected_names: set[str],
    timeout: float = 150,
) -> Session:
    """Reconnect after a restart and check the provisioning state from dump_config."""
    session = Session(host, ZERO_PSK)
    await session.connect(timeout)
    entities, _ = await session.entities()
    sensor_key: int | None = None
    if connected:
        sensor_key = find_entity(
            entities, BinarySensorInfo, connected_names, "binary sensor"
        ).key
    session.subscribe(dump_config=True)
    for label, wanted in (("Cloud credentials", cloud), ("S3 credentials", s3)):
        if wanted:
            await session.wait_until(
                lambda label=label: session.dump_value(label) is not None, 30, label
            )
            if session.dump_value(label) != "YES":
                raise ProvisionError(
                    f"The device reports {label}: NO after provisioning"
                )

    def is_connected() -> bool:
        state = session.states.get(sensor_key) if sensor_key is not None else None
        return session.cloud_connected_logged() or (
            isinstance(state, BinarySensorState) and state.state is True
        )

    if connected:
        await session.wait_until(is_connected, timeout, "the AWS IoT connection")
    return session


async def set_switch(session: Session, names: set[str], value: bool) -> int:
    entities, _ = await session.entities()
    key = find_entity(entities, SwitchInfo, names, "switch").key
    session.subscribe(dump_config=False)
    session.require_client().switch_command(key, value)
    await asyncio.sleep(1)
    return key


async def set_text(session: Session, names: set[str], value: str) -> int:
    entities, _ = await session.entities()
    key = find_entity(entities, TextInfo, names, "text entity").key
    session.subscribe(dump_config=False)
    session.require_client().text_command(key, value)
    await asyncio.sleep(1)
    return key


async def read_switch(host: str, names: set[str]) -> bool:
    session = Session(host, ZERO_PSK)
    await session.connect(60)
    try:
        entities, _ = await session.entities()
        key = find_entity(entities, SwitchInfo, names, "switch").key
        session.subscribe(dump_config=False)
        await session.wait_until(
            lambda: isinstance(session.states.get(key), SwitchState),
            15,
            "the switch state",
        )
        state = session.states[key]
        assert isinstance(state, SwitchState)
        return state.state
    finally:
        await session.close()


async def read_text(host: str, names: set[str]) -> str:
    session = Session(host, ZERO_PSK)
    await session.connect(60)
    try:
        entities, _ = await session.entities()
        key = find_entity(entities, TextInfo, names, "text entity").key
        session.subscribe(dump_config=False)
        await session.wait_until(
            lambda: isinstance(session.states.get(key), TextState), 15, "the text state"
        )
        state = session.states[key]
        assert isinstance(state, TextState)
        return state.state
    finally:
        await session.close()
