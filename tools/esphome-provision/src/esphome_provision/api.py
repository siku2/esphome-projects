"""ESPHome native API sessions and helpers."""

import asyncio
import base64
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Protocol, TypeVar

from aioesphomeapi import (
    APIClient,
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

from esphome_provision.common import ProvisionError, strip_ansi

PORT = 6053
ZERO_PSK = base64.b64encode(bytes(32)).decode()
NAME_LENGTH = 31

Info = TypeVar("Info", bound=EntityInfo)
Result = TypeVar("Result")


class ConnectionDropped(ProvisionError):
    """The device closed the connection when no restart was expected."""


class ConnectTimeout(ProvisionError):
    """The device did not accept a connection in time."""


def invalid_key_message(host: str, psk: str) -> str:
    if psk == ZERO_PSK:
        return (
            f"The device at {host} already has an API key, most likely set by Home Assistant. "
            + "Pass the key with --api-key. "
            + "Or remove the device from Home Assistant, erase the device by running install again "
            + "(or restore, then install), and adopt it in Home Assistant only after the tool finished."
        )
    return (
        f"The device at {host} rejects the API key. "
        + "Check --api-key. The device may have a different key, for example one set by Home Assistant."
    )


def quiet_api_logging() -> None:
    logging.getLogger("aioesphomeapi").setLevel(logging.CRITICAL)


class Session:
    def __init__(
        self, host: str, psk: str, on_line: Callable[[str], None] | None = None
    ) -> None:
        self.host = host
        self.psk = psk
        self.on_line = on_line
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
        for line in text.splitlines():
            if self.on_line:
                self.on_line(line)
            else:
                self.lines.append(line)
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
                    invalid_key_message(self.host, self.psk)
                ) from error
            except (APIConnectionError, TimeoutError, OSError) as error:
                last = error
                await asyncio.sleep(2)
                continue
            self.client = client
            return
        reason = f": {last}" if last else ""
        raise ConnectTimeout(
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

    def subscribe_logs(
        self, *, dump_config: bool, level: LogLevel = LogLevel.LOG_LEVEL_DEBUG
    ) -> None:
        _ = self.require_client().subscribe_logs(self._on_log, level, dump_config)

    def subscribe(self, *, dump_config: bool) -> None:
        self.subscribe_logs(dump_config=dump_config)
        self.require_client().subscribe_states(self._on_state)

    async def wait_stopped(self) -> None:
        await self._stopped.wait()

    async def wait_until(
        self, condition: Callable[[], bool], timeout: float, what: str
    ) -> None:
        deadline = time.monotonic() + timeout
        while not condition():
            if self._stopped.is_set():
                raise ConnectionDropped(
                    f"The connection dropped while waiting for {what}"
                )
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

    async def wait_for_restart(
        self, timeout: float, what: str, log_tag: str | None = None
    ) -> None:
        try:
            await asyncio.wait_for(self._stopped.wait(), timeout)
        except TimeoutError:
            errors = [
                line
                for line in self.lines
                if log_tag and log_tag in line and line.startswith(("[E]", "[W]"))
            ]
            hint = " Device log: " + " | ".join(errors) if errors else ""
            raise ProvisionError(
                f"The device did not restart after {what}.{hint}"
            ) from None


class SessionFactory(Protocol):
    def __call__(self, timeout: float | None = None) -> Awaitable[Session]: ...


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


async def retry_dropped(operation: Callable[[], Awaitable[Result]]) -> Result:
    """Run a self-contained step. If the connection drops, reconnect and run it once more."""
    try:
        return await operation()
    except (APIConnectionError, ConnectionDropped):
        await asyncio.sleep(2)
    try:
        return await operation()
    except (APIConnectionError, ConnectionDropped) as error:
        raise ProvisionError(
            f"The connection to the device dropped twice: {error}"
        ) from error


def session_factory(
    host: str,
    psk: str,
    *,
    mac: str | None,
    project_name: str | None,
    firmware: bytes | None = None,
    node_name: str | None = None,
    default_timeout: float = 90,
) -> SessionFactory:
    """Return a function that connects and checks that this is the expected device."""

    async def connect(timeout: float | None = None) -> Session:
        session = Session(host, psk)
        await session.connect(timeout or default_timeout)
        try:
            info = await session.device_info()
        except BaseException:
            await session.close()
            raise
        problem = None
        if project_name is not None and info.project_name != project_name:
            problem = f"is not running {project_name} (project {info.project_name!r})"
        elif (
            firmware is not None
            and info.project_name
            and info.project_name.encode() not in firmware
        ):
            problem = (
                f"runs the project {info.project_name!r}, which is not in the firmware"
            )
        elif mac is not None and info.mac_address.lower() != mac.lower():
            problem = "has a different MAC address than the flashed chip"
        elif node_name is not None and not (
            info.name == node_name
            or (len(node_name) == NAME_LENGTH and info.name.startswith(node_name))
        ):
            problem = f"is named {info.name!r}, the firmware is named {node_name!r}"
        if problem:
            await session.close()
            raise ProvisionError(f"The device at {host} {problem}")
        return session

    return connect


async def run_action(
    factory: SessionFactory,
    action: str,
    values: ExecuteServiceDataType,
    *,
    log_tag: str | None = None,
) -> None:
    """Call an action and wait until the device restarts."""

    async def prepare() -> tuple[Session, UserService]:
        session = await factory()
        try:
            session.subscribe(dump_config=False)
            _, services = await session.entities()
            service = find_service(services, action)
            if session.stopped():
                raise ConnectionDropped("The connection dropped before the action ran")
        except BaseException:
            await session.close()
            raise
        return session, service

    session, service = await retry_dropped(prepare)
    try:
        _ = await session.require_client().execute_service(service, values)
    except APIConnectionError:
        if not session.stopped():
            raise
    await session.wait_for_restart(30, action, log_tag)
    await session.close()


async def with_session(
    factory: SessionFactory,
    operation: Callable[[Session], Awaitable[Result]],
    timeout: float | None = None,
) -> Result:
    """Connect, run a step and close. A dropped connection is retried once."""

    async def step() -> Result:
        session = await factory(timeout)
        try:
            return await operation(session)
        finally:
            await session.close()

    return await retry_dropped(step)


async def set_switch(factory: SessionFactory, names: set[str], value: bool) -> None:
    async def operation(session: Session) -> None:
        entities, _ = await session.entities()
        key = find_entity(entities, SwitchInfo, names, "switch").key
        session.subscribe(dump_config=False)
        session.require_client().switch_command(key, value)
        await asyncio.sleep(1)

    await with_session(factory, operation)


async def set_text(factory: SessionFactory, names: set[str], value: str) -> None:
    async def operation(session: Session) -> None:
        entities, _ = await session.entities()
        key = find_entity(entities, TextInfo, names, "text entity").key
        session.subscribe(dump_config=False)
        session.require_client().text_command(key, value)
        await asyncio.sleep(1)

    await with_session(factory, operation)


async def read_switch(factory: SessionFactory, names: set[str]) -> bool:
    async def operation(session: Session) -> bool:
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

    return await with_session(factory, operation)


async def read_text(factory: SessionFactory, names: set[str]) -> str:
    async def operation(session: Session) -> str:
        entities, _ = await session.entities()
        key = find_entity(entities, TextInfo, names, "text entity").key
        session.subscribe(dump_config=False)
        await session.wait_until(
            lambda: isinstance(session.states.get(key), TextState), 15, "the text state"
        )
        state = session.states[key]
        assert isinstance(state, TextState)
        return state.state

    return await with_session(factory, operation)
