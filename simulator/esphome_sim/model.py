"""Records that describe entities, states and user services of a device."""

import enum
from dataclasses import dataclass


class EntityKind(enum.StrEnum):
    """The entity types that the simulator knows how to show or control."""

    BUTTON = "button"
    NUMBER = "number"
    SWITCH = "switch"
    SELECT = "select"
    SENSOR = "sensor"
    TEXT_SENSOR = "text_sensor"
    BINARY_SENSOR = "binary_sensor"
    CLIMATE = "climate"
    OTHER = "other"


@dataclass(slots=True, frozen=True)
class NumberRange:
    """The allowed values of a number entity."""

    min_value: float
    max_value: float
    step: float


@dataclass(slots=True, frozen=True)
class Entity:
    """An entity of the connected device."""

    key: int
    object_id: str
    name: str
    kind: EntityKind
    number_range: NumberRange | None = None
    options: tuple[str, ...] = ()
    unit: str = ""
    decimals: int = 1


@dataclass(slots=True, frozen=True)
class ClimateReading:
    """The state of a climate entity."""

    mode: str
    current: float
    target: float


type RawState = bool | float | str | ClimateReading | None


@dataclass(slots=True, frozen=True)
class EntityState:
    """A state update with a human readable value."""

    entity: Entity
    value: str
    raw: RawState


class ArgType(enum.StrEnum):
    """The argument types of an ESPHome user service."""

    BOOL = "bool"
    INT = "int"
    FLOAT = "float"
    STRING = "string"
    BOOL_ARRAY = "bool[]"
    INT_ARRAY = "int[]"
    FLOAT_ARRAY = "float[]"
    STRING_ARRAY = "string[]"


type ArgValue = (
    bool | int | float | str | list[bool] | list[int] | list[float] | list[str]
)


@dataclass(slots=True, frozen=True)
class ServiceArg:
    """One argument of a user service."""

    name: str
    type: ArgType


@dataclass(slots=True, frozen=True)
class Service:
    """A user service (API action) of the device."""

    key: int
    name: str
    args: tuple[ServiceArg, ...]


class LogLevel(enum.IntEnum):
    """Device log levels, with the values of the ESPHome API."""

    NONE = 0
    ERROR = 1
    WARN = 2
    INFO = 3
    CONFIG = 4
    DEBUG = 5
    VERBOSE = 6
    VERY_VERBOSE = 7
