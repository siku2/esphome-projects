"""A typed client for a running simulation, built on aioesphomeapi."""

import asyncio
import math
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import replace

import aioesphomeapi.model as api
from aioesphomeapi.api_pb2 import SubscribeLogsResponse  # type: ignore[attr-defined]
from aioesphomeapi.client import APIClient
from aioesphomeapi.core import (
    APIConnectionError,
    InvalidEncryptionKeyAPIError,
    RequiresEncryptionAPIError,
)

from esphome_sim.errors import ConnectError, SimError
from esphome_sim.model import (
    ArgType,
    ArgValue,
    ClimateReading,
    Entity,
    EntityKind,
    EntityState,
    LogLevel,
    NumberRange,
    Service,
    ServiceArg,
)
from esphome_sim.project import Project
from esphome_sim.validate import check_number, check_option, check_service_args

RETRY_INTERVAL = 2.0
FATAL_ERRORS = (InvalidEncryptionKeyAPIError, RequiresEncryptionAPIError)
ARG_TYPES = {
    api.UserServiceArgType.BOOL: ArgType.BOOL,
    api.UserServiceArgType.INT: ArgType.INT,
    api.UserServiceArgType.FLOAT: ArgType.FLOAT,
    api.UserServiceArgType.STRING: ArgType.STRING,
    api.UserServiceArgType.BOOL_ARRAY: ArgType.BOOL_ARRAY,
    api.UserServiceArgType.INT_ARRAY: ArgType.INT_ARRAY,
    api.UserServiceArgType.FLOAT_ARRAY: ArgType.FLOAT_ARRAY,
    api.UserServiceArgType.STRING_ARRAY: ArgType.STRING_ARRAY,
}


class SimClient:
    """A connection to the simulation of one project."""

    def __init__(self, project: Project) -> None:
        self._project = project
        self._api: APIClient | None = None
        self._closed = asyncio.Event()
        self._entities: dict[str, Entity] = {}
        self._by_key: dict[int, Entity] = {}
        self._services: dict[str, Service] = {}
        self._api_services: dict[str, api.UserService] = {}

    @property
    def address(self) -> str:
        """Return `host:port` of the simulation."""
        return f"{self._project.host}:{self._project.port}"

    @property
    def entities(self) -> Mapping[str, Entity]:
        """Return the entities of the device, indexed by object id."""
        return self._entities

    @property
    def services(self) -> Mapping[str, Service]:
        """Return the user services of the device, indexed by name."""
        return self._services

    async def connect(self, timeout: float | None = None) -> None:
        """Connect, retrying every two seconds, and fetch entities and services."""
        loop = asyncio.get_running_loop()
        self._closed.clear()
        deadline = None if timeout is None else loop.time() + timeout
        while True:
            self._api = self._new_api()
            try:
                await self._api.connect(
                    on_stop=self._on_stop, login=True, log_errors=False
                )
                break
            except FATAL_ERRORS as err:
                raise ConnectError(f"{self.address}: {err}") from err
            except APIConnectionError as err:
                if deadline is not None and loop.time() + RETRY_INTERVAL > deadline:
                    raise ConnectError(
                        f"cannot connect to {self.address}, is the simulation running?"
                    ) from err
                await asyncio.sleep(RETRY_INTERVAL)
        with _connection_errors(self.address):
            infos, services = await self._api.list_entities_services()
        for info in infos:
            entity = _to_entity(info)
            self._entities[entity.object_id] = entity
            self._by_key[entity.key] = entity
        for user_service in services:
            self._services[user_service.name] = _to_service(user_service)
            self._api_services[user_service.name] = user_service

    async def disconnect(self) -> None:
        """Close the connection."""
        if self._api is None:
            return
        try:
            await self._api.disconnect()
        except APIConnectionError:
            pass

    async def wait_closed(self) -> None:
        """Wait until the connection drops."""
        await self._closed.wait()

    def entity(self, object_id: str, kind: EntityKind | None = None) -> Entity:
        """Return the entity `object_id`, which must be of `kind` if given."""
        entity = self._entities.get(object_id)
        if entity is None or (kind is not None and entity.kind is not kind):
            candidates = sorted(
                e.object_id
                for e in self._entities.values()
                if kind is None or e.kind is kind
            )
            what = f"{kind} entity" if kind else "entity"
            valid = ", ".join(candidates) or "none"
            raise SimError(f"no {what} {object_id!r}, valid: {valid}")
        return entity

    def service(self, name: str) -> Service:
        """Return the user service `name`."""
        service = self._services.get(name)
        if service is None:
            valid = ", ".join(sorted(self._services)) or "none"
            raise SimError(f"no service {name!r}, valid: {valid}")
        return service

    def press_button(self, object_id: str) -> None:
        """Press a button entity."""
        entity = self.entity(object_id, EntityKind.BUTTON)
        with _connection_errors(self.address):
            self._connected().button_command(entity.key)

    def set_number(self, object_id: str, value: float) -> None:
        """Set a number entity after checking its range and step."""
        entity = self.entity(object_id, EntityKind.NUMBER)
        if entity.number_range is None:
            raise SimError(f"{object_id}: number entity without a range")
        checked = check_number(entity, entity.number_range, value)
        with _connection_errors(self.address):
            self._connected().number_command(entity.key, checked)

    def set_switch(self, object_id: str, on: bool) -> None:
        """Turn a switch entity on or off."""
        entity = self.entity(object_id, EntityKind.SWITCH)
        with _connection_errors(self.address):
            self._connected().switch_command(entity.key, on)

    def set_select(self, object_id: str, option: str) -> None:
        """Choose an option of a select entity."""
        entity = self.entity(object_id, EntityKind.SELECT)
        checked = check_option(entity, option)
        with _connection_errors(self.address):
            self._connected().select_command(entity.key, checked)

    async def call_service(self, name: str, **args: ArgValue) -> None:
        """Call a user service after checking argument names and types."""
        checked = check_service_args(self.service(name), args)
        with _connection_errors(self.address):
            await self._connected().execute_service(self._api_services[name], checked)

    def subscribe(self, callback: Callable[[EntityState], None]) -> None:
        """Call `callback` for every state update of a known entity."""

        def on_state(state: api.EntityState) -> None:
            entity = self._by_key.get(state.key)
            if entity is not None:
                callback(_to_state(entity, state))

        with _connection_errors(self.address):
            self._connected().subscribe_states(on_state)

    def subscribe_logs(
        self, callback: Callable[[str], None], level: LogLevel
    ) -> Callable[[], None]:
        """Call `callback` with every log line, return a function to stop."""

        def on_log(message: SubscribeLogsResponse) -> None:
            callback(bytes(message.message).decode(errors="replace"))

        with _connection_errors(self.address):
            return self._connected().subscribe_logs(
                on_log, log_level=api.LogLevel(level)
            )

    def _connected(self) -> APIClient:
        if self._api is None:
            raise ConnectError(f"not connected to {self.address}")
        return self._api

    def _new_api(self) -> APIClient:
        return APIClient(
            self._project.host,
            self._project.port,
            None,
            noise_psk=self._project.api_key,
        )

    async def _on_stop(self, expected_disconnect: bool) -> None:
        self._closed.set()


@contextmanager
def _connection_errors(address: str) -> Iterator[None]:
    try:
        yield
    except APIConnectionError as err:
        raise ConnectError(f"{address}: {err}") from err


def _to_entity(info: api.EntityInfo) -> Entity:
    entity = Entity(
        key=info.key, object_id=info.object_id, name=info.name, kind=EntityKind.OTHER
    )
    match info:
        case api.ButtonInfo():
            return replace(entity, kind=EntityKind.BUTTON)
        case api.NumberInfo():
            return replace(
                entity,
                kind=EntityKind.NUMBER,
                number_range=NumberRange(info.min_value, info.max_value, info.step),
                unit=info.unit_of_measurement,
                decimals=_step_decimals(info.step),
            )
        case api.SwitchInfo():
            return replace(entity, kind=EntityKind.SWITCH)
        case api.SelectInfo():
            return replace(entity, kind=EntityKind.SELECT, options=tuple(info.options))
        case api.SensorInfo():
            return replace(
                entity,
                kind=EntityKind.SENSOR,
                unit=info.unit_of_measurement,
                decimals=info.accuracy_decimals,
            )
        case api.TextSensorInfo():
            return replace(entity, kind=EntityKind.TEXT_SENSOR)
        case api.BinarySensorInfo():
            return replace(entity, kind=EntityKind.BINARY_SENSOR)
        case api.ClimateInfo():
            return replace(entity, kind=EntityKind.CLIMATE)
    return entity


def _to_service(service: api.UserService) -> Service:
    args = []
    for arg in service.args:
        if arg.type is None:
            raise SimError(f"service {service.name}: argument {arg.name} has no type")
        args.append(ServiceArg(name=arg.name, type=ARG_TYPES[arg.type]))
    return Service(key=service.key, name=service.name, args=tuple(args))


def _to_state(entity: Entity, state: api.EntityState) -> EntityState:
    match state:
        case api.SensorState() | api.NumberState():
            if state.missing_state or not math.isfinite(state.state):
                return EntityState(entity, "unknown", None)
            return EntityState(entity, _format_number(entity, state.state), state.state)
        case api.BinarySensorState() if state.missing_state:
            return EntityState(entity, "unknown", None)
        case api.SwitchState() | api.BinarySensorState():
            return EntityState(entity, "on" if state.state else "off", state.state)
        case api.TextSensorState() | api.SelectState():
            if state.missing_state:
                return EntityState(entity, "unknown", None)
            return EntityState(entity, state.state, state.state)
        case api.ClimateState():
            reading = ClimateReading(
                mode=state.mode.name if state.mode is not None else "UNKNOWN",
                current=state.current_temperature,
                target=state.target_temperature,
            )
            value = (
                f"{reading.mode} current {reading.current:.1f}"
                f" target {reading.target:.1f}"
            )
            return EntityState(entity, value, reading)
    return EntityState(entity, "unknown", None)


def _format_number(entity: Entity, value: float) -> str:
    text = f"{value:.{max(entity.decimals, 0)}f}"
    return f"{text} {entity.unit}" if entity.unit else text


def _step_decimals(step: float) -> int:
    text = f"{step:g}"
    if "e" in text:
        return 3
    return len(text.partition(".")[2])
