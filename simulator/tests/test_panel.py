"""Tests for the panel entity ordering."""

from esphome_sim.model import Entity, EntityKind
from esphome_sim.panel import order_entities


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
