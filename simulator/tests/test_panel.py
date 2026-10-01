"""Tests for the panel entity ordering."""

from esphome_sim.model import Entity, EntityKind
from esphome_sim.panel import button_rows, order_entities


def entity(key: int, name: str, kind: EntityKind) -> Entity:
    return Entity(key=key, object_id=name.lower(), name=name, kind=kind)


def test_order_groups_by_kind_and_keeps_reported_order() -> None:
    entities = [
        entity(1, "Zed", EntityKind.SENSOR),
        entity(2, "Beta", EntityKind.BUTTON),
        entity(3, "Alpha", EntityKind.BUTTON),
        entity(4, "Gamma", EntityKind.NUMBER),
        entity(5, "Other", EntityKind.OTHER),
        entity(6, "Aaa", EntityKind.SENSOR),
    ]
    assert [e.name for e in order_entities(entities)] == [
        "Beta",
        "Alpha",
        "Gamma",
        "Zed",
        "Aaa",
    ]


def button(key: int, name: str) -> Entity:
    return entity(key, name, EntityKind.BUTTON)


def names(rows: list[tuple[Entity, Entity | None]]) -> list[tuple[str, str | None]]:
    return [(b.name, h.name if h else None) for b, h in rows]


def test_button_rows_pair_base_with_hold() -> None:
    rows = button_rows([button(1, "Menu"), button(2, "Menu Hold")])
    assert names(rows) == [("Menu", "Menu Hold")]


def test_button_rows_keep_hold_without_base_alone() -> None:
    rows = button_rows([button(1, "Menu Hold")])
    assert names(rows) == [("Menu Hold", None)]


def test_button_rows_keep_base_without_hold_alone() -> None:
    rows = button_rows([button(1, "Menu")])
    assert names(rows) == [("Menu", None)]


def test_button_rows_keep_base_position() -> None:
    buttons = [
        button(1, "Up"),
        button(2, "Menu Hold"),
        button(3, "Down"),
        button(4, "Menu"),
        button(5, "Up Hold"),
    ]
    assert names(button_rows(buttons)) == [
        ("Up", "Up Hold"),
        ("Down", None),
        ("Menu", "Menu Hold"),
    ]
