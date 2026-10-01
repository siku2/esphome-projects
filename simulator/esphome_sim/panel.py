"""A Tk control panel built from the entities of a running simulation."""

import asyncio
import queue
import threading
import tkinter as tk
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass

from esphome_sim.client import RETRY_INTERVAL, SimClient
from esphome_sim.errors import ConnectError, SimError
from esphome_sim.model import Entity, EntityKind, EntityState, NumberRange
from esphome_sim.project import Project

SIM_PREFIX = "sim_"
POLL_MS = 100
SCALE_LENGTH = 320
HOLD_SUFFIX = " Hold"
KIND_ORDER = (
    EntityKind.BUTTON,
    EntityKind.NUMBER,
    EntityKind.SWITCH,
    EntityKind.SELECT,
    EntityKind.SENSOR,
    EntityKind.BINARY_SENSOR,
    EntityKind.TEXT_SENSOR,
    EntityKind.CLIMATE,
)


@dataclass(slots=True, frozen=True)
class Connected:
    """The client connected and fetched these entities."""

    entities: tuple[Entity, ...]


@dataclass(slots=True, frozen=True)
class Disconnected:
    """The client is not connected."""

    message: str


@dataclass(slots=True, frozen=True)
class StateChanged:
    """An entity reported a new state."""

    state: EntityState


@dataclass(slots=True, frozen=True)
class CommandFailed:
    """A command from the panel was rejected."""

    message: str


type PanelEvent = Connected | Disconnected | StateChanged | CommandFailed
type Updater = Callable[[EntityState], None]
type Command = Callable[[SimClient], None]


def order_entities(entities: Iterable[Entity]) -> list[Entity]:
    """Group the shown entities by kind and keep the reported order within a kind."""
    shown = [e for e in entities if e.kind in KIND_ORDER]
    return sorted(shown, key=lambda e: KIND_ORDER.index(e.kind))


def button_rows(buttons: Sequence[Entity]) -> list[tuple[Entity, Entity | None]]:
    """Pair each button with its " Hold" button and keep the order of the base."""
    by_name = {b.name: b for b in buttons}
    holds = {f"{b.name}{HOLD_SUFFIX}" for b in buttons} & by_name.keys()
    return [
        (b, by_name.get(f"{b.name}{HOLD_SUFFIX}"))
        for b in buttons
        if b.name not in holds
    ]


class Panel:
    """A window with one widget per entity, split into simulation and device."""

    def __init__(self, project: Project, stop: threading.Event | None = None) -> None:
        self._project = project
        self._stop = stop if stop is not None else threading.Event()
        self._events: queue.Queue[PanelEvent] = queue.Queue()
        self._loop = asyncio.new_event_loop()
        self._client: SimClient | None = None
        self._updaters: dict[str, Updater] = {}
        try:
            self._root = tk.Tk()
        except tk.TclError as err:
            raise SimError(f"cannot open the panel window: {err}") from err
        self._root.title(f"{project.name} simulator")
        self._status = tk.StringVar(master=self._root, value="Connecting")
        tk.Label(self._root, textvariable=self._status, anchor="w").pack(
            fill=tk.X, padx=6, pady=4
        )
        groups = tk.Frame(self._root)
        groups.pack(fill=tk.BOTH, expand=True)
        self._sim_group = tk.LabelFrame(groups, text="Simulation")
        self._device_group = tk.LabelFrame(groups, text="Device")
        for group in (self._sim_group, self._device_group):
            group.pack(
                side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=4, anchor="n"
            )
            group.columnconfigure(1, weight=1)

    def run(self) -> None:
        """Show the window until it is closed or the stop event is set."""
        session = self._loop.create_task(self._session())
        thread = threading.Thread(target=self._run_loop, args=(session,), daemon=True)
        thread.start()
        self._root.after(POLL_MS, self._drain)
        try:
            self._root.mainloop()
        finally:
            self._loop.call_soon_threadsafe(session.cancel)
            thread.join()

    def _run_loop(self, session: "asyncio.Task[None]") -> None:
        try:
            self._loop.run_until_complete(session)
        except asyncio.CancelledError:
            pass
        finally:
            self._loop.close()

    async def _session(self) -> None:
        while True:
            client = SimClient(self._project)
            self._events.put(Disconnected(f"Waiting for {client.address}"))
            try:
                await client.connect()
            except ConnectError as err:
                self._events.put(Disconnected(str(err)))
                await asyncio.sleep(RETRY_INTERVAL)
                continue
            self._client = client
            try:
                self._events.put(Connected(tuple(client.entities.values())))
                client.subscribe(lambda state: self._events.put(StateChanged(state)))
                await client.wait_closed()
            finally:
                self._client = None
                await client.disconnect()

    def _send(self, command: Command) -> None:
        def run() -> None:
            client = self._client
            if client is None:
                return
            try:
                command(client)
            except SimError as err:
                self._events.put(CommandFailed(str(err)))

        self._loop.call_soon_threadsafe(run)

    def _drain(self) -> None:
        if self._stop.is_set():
            self._root.destroy()
            return
        while True:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            match event:
                case Connected(entities):
                    self._build(entities)
                    self._status.set(
                        f"Connected to {self._project.host}:{self._project.port}"
                    )
                case Disconnected(message):
                    self._clear()
                    self._status.set(message)
                case StateChanged(state):
                    updater = self._updaters.get(state.entity.object_id)
                    if updater is not None:
                        updater(state)
                case CommandFailed(message):
                    self._status.set(message)
        self._root.after(POLL_MS, self._drain)

    def _clear(self) -> None:
        for group in (self._sim_group, self._device_group):
            for child in group.winfo_children():
                child.destroy()
        self._updaters.clear()

    def _build(self, entities: Iterable[Entity]) -> None:
        self._clear()
        ordered = order_entities(entities)
        buttons = [e for e in ordered if e.kind is EntityKind.BUTTON]
        for base, hold in button_rows(buttons):
            group = self._group_for(base)
            self._add_button_row(group, group.grid_size()[1], base, hold)
        for entity in ordered:
            if entity.kind is EntityKind.BUTTON:
                continue
            group = self._group_for(entity)
            updater = self._add_row(group, group.grid_size()[1], entity)
            if updater is not None:
                self._updaters[entity.object_id] = updater

    def _group_for(self, entity: Entity) -> tk.LabelFrame:
        sim = entity.object_id.startswith(SIM_PREFIX)
        return self._sim_group if sim else self._device_group

    def _add_button_row(
        self, group: tk.LabelFrame, row: int, base: Entity, hold: Entity | None
    ) -> None:
        span = 2 if hold is None else 1
        for column, entity in enumerate((base, hold)):
            if entity is None:
                continue
            tk.Button(
                group,
                text=entity.name,
                command=self._press(entity.object_id),
            ).grid(row=row, column=column, columnspan=span, sticky="ew", padx=4, pady=1)

    def _press(self, object_id: str) -> Callable[[], None]:
        return lambda: self._send(lambda c: c.press_button(object_id))

    def _add_row(
        self, group: tk.LabelFrame, row: int, entity: Entity
    ) -> Updater | None:
        object_id = entity.object_id
        label = f"{entity.name} ({entity.unit})" if entity.unit else entity.name
        tk.Label(group, text=label, anchor="w").grid(
            row=row, column=0, sticky="w", padx=4
        )
        match entity.kind:
            case EntityKind.NUMBER if entity.number_range is not None:
                return self._add_scale(group, row, object_id, entity.number_range)
            case EntityKind.SWITCH:
                return self._add_switch(group, row, object_id)
            case EntityKind.SELECT if entity.options:
                return self._add_select(group, row, object_id, entity.options)
        value = tk.StringVar(master=self._root, value="--")
        tk.Label(group, textvariable=value, anchor="w").grid(
            row=row, column=1, sticky="w", padx=4
        )
        return lambda state: value.set(state.value)

    def _add_scale(
        self, group: tk.LabelFrame, row: int, object_id: str, number_range: NumberRange
    ) -> Updater:
        scale = tk.Scale(
            group,
            orient=tk.HORIZONTAL,
            from_=number_range.min_value,
            to=number_range.max_value,
            resolution=number_range.step,
            length=SCALE_LENGTH,
        )
        scale.grid(row=row, column=1, sticky="ew", padx=4)

        def on_release(_event: "tk.Event[tk.Scale]") -> None:
            value = float(scale.get())
            self._send(lambda c: c.set_number(object_id, value))

        scale.bind("<ButtonRelease-1>", on_release)

        def update(state: EntityState) -> None:
            if isinstance(state.raw, float):
                scale.set(state.raw)

        return update

    def _add_switch(self, group: tk.LabelFrame, row: int, object_id: str) -> Updater:
        checked = tk.BooleanVar(master=self._root)

        def on_toggle() -> None:
            on = checked.get()
            self._send(lambda c: c.set_switch(object_id, on))

        tk.Checkbutton(group, variable=checked, command=on_toggle).grid(
            row=row, column=1, sticky="w", padx=4
        )

        def update(state: EntityState) -> None:
            if isinstance(state.raw, bool):
                checked.set(state.raw)

        return update

    def _add_select(
        self, group: tk.LabelFrame, row: int, object_id: str, options: tuple[str, ...]
    ) -> Updater:
        first, *rest = options
        selected = tk.StringVar(master=self._root, value=first)

        def on_select(_value: tk.StringVar) -> None:
            option = selected.get()
            self._send(lambda c: c.set_select(object_id, option))

        tk.OptionMenu(group, selected, first, *rest, command=on_select).grid(
            row=row, column=1, sticky="w", padx=4
        )

        def update(state: EntityState) -> None:
            if isinstance(state.raw, str):
                selected.set(state.raw)

        return update
