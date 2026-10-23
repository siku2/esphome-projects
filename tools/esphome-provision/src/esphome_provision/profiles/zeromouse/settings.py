"""Carry the settings of the stock firmware over to the ESPHome entities."""

import asyncio
import math
from dataclasses import dataclass
from typing import Literal

from aioesphomeapi import (
    EntityInfo,
    NumberInfo,
    NumberState,
    SelectInfo,
    SelectState,
    SwitchInfo,
    SwitchState,
)

from esphome_provision import api
from esphome_provision.api import Session, SessionFactory
from esphome_provision.common import ProvisionError
from esphome_provision.profiles.zeromouse.names import entity_names
from esphome_provision.profiles.zeromouse.nvs import NvsContent

Kind = Literal["number", "switch", "select"]

# Brightness comes first because it rewrites the exposure level and gain ceiling.
STOCK_SETTINGS: tuple[tuple[str, str, str, Kind], ...] = (
    ("camera", "brightness", "brightness", "number"),
    ("camera", "autoExpLevel", "auto_exp_level", "number"),
    ("camera", "gainCeiling", "gain_ceiling", "number"),
    ("camera", "autoGainCtrlMin", "auto_gain_ctrl_min", "number"),
    ("camera", "autoGainCtrlMax", "auto_gain_ctrl_max", "number"),
    ("camera", "autoGainStblMin", "auto_gain_stbl_min", "number"),
    ("camera", "autoGainStblMax", "auto_gain_stbl_max", "number"),
    ("camera", "camShtsBfrT0", "cam_shots_before_t0", "number"),
    ("system", "cleanET", "clean_event_time", "number"),
    ("system", "undecidableMode", "undecidable_mode", "select"),
    ("system", "finaleInMC", "finale_interval_max_count", "number"),
    ("system", "finaleInD", "finale_interval_delta", "number"),
    ("system", "uploadEmpty", "upload_empty", "switch"),
    ("system", "logEnabled", "log_enabled", "switch"),
    ("prox", "irRangeIgnore", "ir_range_ignore_event", "number"),
    ("prox", "irPreyTime", "ir_prey_event_time", "number"),
    ("prox", "irMedianCount", "ir_median_count", "number"),
    ("prox", "irAmbient", "ir_ambient", "number"),
    ("prox", "irSlopeThresh", "ir_slope_thresh_value", "number"),
    ("prox", "irMotionThrsh", "ir_motion_thresh", "number"),
    ("prox", "irPresenceThrsh", "ir_presence_thresh", "number"),
    ("rfid", "blockEnabled", "block_enabled", "switch"),
    ("rfid", "responseTimeout", "response_timeout", "number"),
    ("rfid", "lREMCount", "last_ref_event_max_count", "number"),
    ("rfid", "lREMTimeSec", "last_ref_event_max_time_sec", "number"),
)
APPLY_TIMEOUT_S = 15


@dataclass(frozen=True)
class StockSetting:
    key: str
    kind: Kind
    value: int


def stock_settings(content: NvsContent, *, skip: set[str]) -> list[StockSetting]:
    """The stock values in carry-over order. Keys missing from the NVS are left out."""
    found: list[StockSetting] = []
    for namespace, nvs_key, key, kind in STOCK_SETTINGS:
        value = content.get((namespace, nvs_key))
        if key in skip or value is None:
            continue
        if not isinstance(value, int):
            raise ProvisionError(
                f"The stock setting {namespace}/{nvs_key} is not a number"
            )
        found.append(StockSetting(key, kind, value))
    return found


@dataclass(frozen=True)
class Target:
    setting: StockSetting
    entity: EntityInfo
    wanted: float | bool | str


def resolve(entities: list[EntityInfo], settings: list[StockSetting]) -> list[Target]:
    targets: list[Target] = []
    errors: list[str] = []
    for setting in settings:
        names = entity_names(setting.key)
        kind = {"number": NumberInfo, "switch": SwitchInfo, "select": SelectInfo}[
            setting.kind
        ]
        matches = [e for e in entities if isinstance(e, kind) and e.name in names]
        if not matches:
            print(f"  {setting.key}: this board has no such setting, skipped")
            continue
        if len(matches) > 1:
            raise ProvisionError(f"More than one entity for {setting.key}")
        entity = matches[0]
        wanted: float | bool | str
        if isinstance(entity, NumberInfo):
            wanted = float(setting.value)
            if not entity.min_value <= wanted <= entity.max_value:
                errors.append(
                    f"{setting.key} = {setting.value} is outside {entity.min_value:g} to {entity.max_value:g}"
                )
                continue
        elif isinstance(entity, SwitchInfo):
            wanted = setting.value != 0
        else:
            assert isinstance(entity, SelectInfo)
            if not 0 <= setting.value < len(entity.options):
                errors.append(f"{setting.key} = {setting.value} has no option")
                continue
            wanted = entity.options[setting.value]
        targets.append(Target(setting, entity, wanted))
    if errors:
        raise ProvisionError("Stock settings that do not fit: " + ", ".join(errors))
    return targets


def matches(session: Session, target: Target) -> bool:
    state = session.states.get(target.entity.key)
    if isinstance(state, NumberState):
        return isinstance(target.wanted, float) and math.isclose(
            state.state, target.wanted, abs_tol=1e-3
        )
    if isinstance(state, (SwitchState, SelectState)):
        return state.state == target.wanted
    return False


async def apply_stock_settings(
    factory: SessionFactory, settings: list[StockSetting]
) -> None:
    """Set every stock value and require all of them to read back."""

    async def operation(session: Session) -> None:
        entities, _ = await session.entities()
        targets = resolve(entities, settings)
        session.subscribe(dump_config=False)
        client = session.require_client()
        for index, target in enumerate(targets):
            if isinstance(target.wanted, bool):
                client.switch_command(target.entity.key, target.wanted)
            elif isinstance(target.wanted, float):
                client.number_command(target.entity.key, target.wanted)
            else:
                client.select_command(target.entity.key, str(target.wanted))
            if index == 0 and target.setting.key == "brightness":
                await asyncio.sleep(1)
        try:
            await session.wait_until(
                lambda: all(matches(session, t) for t in targets),
                APPLY_TIMEOUT_S,
                "the stock settings",
            )
        except ProvisionError:
            wrong = [t.setting.key for t in targets if not matches(session, t)]
            raise ProvisionError(
                "These stock settings did not stick: " + ", ".join(wrong)
            ) from None
        for target in targets:
            print(f"  {target.setting.key} = {target.wanted}")

    await api.with_session(factory, operation)
