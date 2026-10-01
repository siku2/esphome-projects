"""Check and convert values before they are sent to the device."""

import math
from collections.abc import Mapping, Sequence
from typing import TypeIs

from esphome_sim.errors import SimError
from esphome_sim.model import ArgType, ArgValue, Entity, NumberRange, Service

STEP_TOLERANCE = 1e-6
TRUE_WORDS = frozenset({"on", "true", "yes", "1"})
FALSE_WORDS = frozenset({"off", "false", "no", "0"})


def check_number(entity: Entity, number_range: NumberRange, value: float) -> float:
    """Return `value` if it lies in the range and on a step of the entity."""
    low, high, step = number_range.min_value, number_range.max_value, number_range.step
    if not math.isfinite(value):
        raise SimError(f"{entity.object_id}: {value} is not a finite number")
    if not low - STEP_TOLERANCE <= value <= high + STEP_TOLERANCE:
        raise SimError(f"{entity.object_id}: {value:g} is outside {low:g}..{high:g}")
    if step > 0:
        steps = (value - low) / step
        if abs(steps - round(steps)) > STEP_TOLERANCE:
            raise SimError(
                f"{entity.object_id}: {value:g} is not {low:g} plus a multiple"
                f" of the step {step:g}"
            )
    return value


def check_option(entity: Entity, option: str) -> str:
    """Return `option` if it is one of the options of a select entity."""
    if option not in entity.options:
        valid = ", ".join(entity.options) or "none"
        raise SimError(f"{entity.object_id}: unknown option {option!r}, valid: {valid}")
    return option


def parse_float(text: str, what: str) -> float:
    """Parse a finite float."""
    try:
        value = float(text)
    except ValueError:
        raise SimError(f"{what}: {text!r} is not a number") from None
    if not math.isfinite(value):
        raise SimError(f"{what}: {text!r} is not a finite number")
    return value


def parse_int(text: str, what: str) -> int:
    """Parse an integer."""
    try:
        return int(text)
    except ValueError:
        raise SimError(f"{what}: {text!r} is not an integer") from None


def parse_bool(text: str, what: str) -> bool:
    """Parse on/off, true/false, yes/no or 1/0."""
    word = text.strip().lower()
    if word in TRUE_WORDS:
        return True
    if word in FALSE_WORDS:
        return False
    raise SimError(f"{what}: {text!r} is not on or off")


def parse_service_args(
    service: Service, pairs: Sequence[tuple[str, str]]
) -> dict[str, ArgValue]:
    """Convert `name=value` strings to the argument types of `service`."""
    types = {arg.name: arg.type for arg in service.args}
    values: dict[str, ArgValue] = {}
    for name, text in pairs:
        if name in values:
            raise SimError(f"{service.name}: argument {name!r} given twice")
        arg_type = types.get(name)
        if arg_type is None:
            raise SimError(_unknown_arg(service, name))
        values[name] = _parse_arg(arg_type, text, f"{service.name}.{name}")
    return check_service_args(service, values)


def check_service_args(
    service: Service, args: Mapping[str, ArgValue]
) -> dict[str, ArgValue]:
    """Return `args` if their names and types match the service definition."""
    for name in args:
        if name not in {arg.name for arg in service.args}:
            raise SimError(_unknown_arg(service, name))
    checked: dict[str, ArgValue] = {}
    for arg in service.args:
        if arg.name not in args:
            raise SimError(f"{service.name}: missing argument {arg.name!r}")
        checked[arg.name] = _check_arg(
            arg.type, args[arg.name], f"{service.name}.{arg.name}"
        )
    return checked


def describe_service(service: Service) -> str:
    """Return a signature like `press(input: string, hold_ms: int)`."""
    args = ", ".join(f"{arg.name}: {arg.type}" for arg in service.args)
    return f"{service.name}({args})"


def _unknown_arg(service: Service, name: str) -> str:
    return f"{service.name}: unknown argument {name!r}, expected {describe_service(service)}"


def _parse_arg(arg_type: ArgType, text: str, what: str) -> ArgValue:
    items = [item.strip() for item in text.split(",")] if text.strip() else []
    match arg_type:
        case ArgType.BOOL:
            return parse_bool(text, what)
        case ArgType.INT:
            return parse_int(text, what)
        case ArgType.FLOAT:
            return parse_float(text, what)
        case ArgType.STRING:
            return text
        case ArgType.BOOL_ARRAY:
            return [parse_bool(item, what) for item in items]
        case ArgType.INT_ARRAY:
            return [parse_int(item, what) for item in items]
        case ArgType.FLOAT_ARRAY:
            return [parse_float(item, what) for item in items]
        case ArgType.STRING_ARRAY:
            return items


def _check_arg(arg_type: ArgType, value: ArgValue, what: str) -> ArgValue:
    match arg_type:
        case ArgType.BOOL if isinstance(value, bool):
            return value
        case ArgType.INT if _is_int(value):
            return value
        case ArgType.FLOAT if _is_real(value):
            return float(value)
        case ArgType.STRING if isinstance(value, str):
            return value
        case ArgType.BOOL_ARRAY if isinstance(value, list):
            if all(isinstance(item, bool) for item in value):
                return value
        case ArgType.INT_ARRAY if isinstance(value, list):
            if all(_is_int(item) for item in value):
                return value
        case ArgType.FLOAT_ARRAY if isinstance(value, list):
            if all(_is_real(item) for item in value):
                return [float(item) for item in value]
        case ArgType.STRING_ARRAY if isinstance(value, list):
            if all(isinstance(item, str) for item in value):
                return value
    raise SimError(f"{what}: expected {arg_type}, got {value!r}")


def _is_int(value: object) -> TypeIs[int]:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_real(value: object) -> TypeIs[int | float]:
    return isinstance(value, int | float) and not isinstance(value, bool)
