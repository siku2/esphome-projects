"""Tests for number, option and service argument validation."""

import pytest

from esphome_sim.errors import SimError
from esphome_sim.model import (
    ArgType,
    ArgValue,
    Entity,
    EntityKind,
    NumberRange,
    Service,
    ServiceArg,
)
from esphome_sim.validate import (
    check_number,
    check_option,
    check_service_args,
    parse_bool,
    parse_service_args,
)

BEARING = Entity(
    key=1, object_id="sim_wind_bearing", name="Bearing", kind=EntityKind.NUMBER
)
BEARING_RANGE = NumberRange(0, 360, 5)
OUTDOOR_RANGE = NumberRange(-20, 45, 0.5)
MODE = Entity(
    key=2, object_id="mode", name="Mode", kind=EntityKind.SELECT, options=("a", "b")
)
PRESS = Service(
    key=3,
    name="press",
    args=(ServiceArg("input", ArgType.STRING), ServiceArg("hold_ms", ArgType.INT)),
)
ALL_TYPES = Service(
    key=4,
    name="all",
    args=(
        ServiceArg("flag", ArgType.BOOL),
        ServiceArg("ratio", ArgType.FLOAT),
        ServiceArg("ids", ArgType.INT_ARRAY),
        ServiceArg("names", ArgType.STRING_ARRAY),
    ),
)


@pytest.mark.parametrize("value", [0, 90, 360, 235])
def test_number_on_step(value: float) -> None:
    assert check_number(BEARING, BEARING_RANGE, value) == value


@pytest.mark.parametrize("value", [-20, 18.5, 44.5, 45])
def test_number_on_fractional_step(value: float) -> None:
    assert check_number(BEARING, OUTDOOR_RANGE, value) == value


@pytest.mark.parametrize("value", [-5, 365, float("nan"), float("inf")])
def test_number_out_of_range(value: float) -> None:
    with pytest.raises(SimError, match="sim_wind_bearing"):
        check_number(BEARING, BEARING_RANGE, value)


def test_number_off_step() -> None:
    with pytest.raises(SimError, match="step 5"):
        check_number(BEARING, BEARING_RANGE, 91)


def test_number_without_step_accepts_any() -> None:
    assert check_number(BEARING, NumberRange(0, 1, 0), 0.123) == 0.123


def test_option() -> None:
    assert check_option(MODE, "b") == "b"
    with pytest.raises(SimError, match="valid: a, b"):
        check_option(MODE, "c")


@pytest.mark.parametrize(
    ("text", "expected"), [("on", True), ("OFF", False), ("1", True)]
)
def test_parse_bool(text: str, expected: bool) -> None:
    assert parse_bool(text, "x") is expected


def test_parse_bool_rejects_other_words() -> None:
    with pytest.raises(SimError, match="not on or off"):
        parse_bool("maybe", "x")


def test_parse_service_args() -> None:
    args = parse_service_args(PRESS, [("input", "exit"), ("hold_ms", "1200")])
    assert args == {"input": "exit", "hold_ms": 1200}


def test_parse_service_args_all_types() -> None:
    pairs = [("flag", "on"), ("ratio", "2"), ("ids", "1, 2"), ("names", "")]
    args = parse_service_args(ALL_TYPES, pairs)
    assert args == {"flag": True, "ratio": 2.0, "ids": [1, 2], "names": []}
    assert isinstance(args["ratio"], float)


def test_parse_service_args_unknown_name() -> None:
    with pytest.raises(
        SimError, match=r"expected press\(input: string, hold_ms: int\)"
    ):
        parse_service_args(PRESS, [("key", "x")])


def test_parse_service_args_bad_int() -> None:
    with pytest.raises(SimError, match="press.hold_ms"):
        parse_service_args(PRESS, [("input", "exit"), ("hold_ms", "long")])


def test_parse_service_args_duplicate() -> None:
    with pytest.raises(SimError, match="given twice"):
        parse_service_args(PRESS, [("input", "a"), ("input", "b")])


def test_check_service_args_missing() -> None:
    with pytest.raises(SimError, match="missing argument 'hold_ms'"):
        check_service_args(PRESS, {"input": "exit"})


def test_check_service_args_rejects_bool_for_int() -> None:
    with pytest.raises(SimError, match="expected int"):
        check_service_args(PRESS, {"input": "exit", "hold_ms": True})


def test_check_service_args_rejects_wrong_array_items() -> None:
    args: dict[str, ArgValue] = {"flag": False, "ratio": 1, "ids": ["1"], "names": []}
    with pytest.raises(SimError, match="expected int"):
        check_service_args(ALL_TYPES, args)
