"""Tk control panel for the grill display host simulation."""

import asyncio
import queue
import threading
import tkinter as tk

import aioesphomeapi
from aioesphomeapi import (
    ClimateInfo,
    ClimateState,
    NumberInfo,
    SensorInfo,
    SwitchInfo,
    TextSensorInfo,
)

HOST = "127.0.0.1"
PORT = 6053
KEY = "j/+/ENFUx0rHq87Yz1/Cll3lOqfVXsQ3TC+zXp/A9CI="
RETRY_SECONDS = 2

BUTTONS = [
    ("Up", "rotary_up", 100),
    ("Down", "rotary_down", 100),
    ("Press", "rotary_press", 100),
    ("Menu", "menu", 100),
    ("Exit", "exit", 100),
    ("Exit (hold)", "exit", 1200),
]
WEATHER_PREFIXES = ("sim_wind", "sim_outdoor", "sim_solar")
GRILL_PREFIXES = ("sim_west", "sim_east", "sim_meat_temperature")
DEVICE_ROWS = [
    ("Phase", TextSensorInfo, "garphase"),
    ("Meat rate", SensorInfo, "fleischrate"),
    ("ETA", TextSensorInfo, "fertig_um"),
    ("Remaining minutes", SensorInfo, "restminuten"),
    ("Meat target", NumberInfo, "zieltemperatur_fleisch"),
    ("West heater", ClimateInfo, "heizstab_west"),
    ("East heater", ClimateInfo, "heizstab_ost"),
]

loop = asyncio.new_event_loop()
events = queue.Queue()
link = {"client": None, "press": None}


def send(method, *args):
    client = link["client"]

    async def call():
        result = getattr(client, method)(*args)
        if result is not None:
            await result

    if client is not None:
        asyncio.run_coroutine_threadsafe(call(), loop)


async def session():
    while True:
        stopped = asyncio.Event()

        async def on_stop(_expected):
            stopped.set()

        client = aioesphomeapi.APIClient(HOST, PORT, None, noise_psk=KEY)
        try:
            await client.connect(on_stop=on_stop, login=True)
            entities, services = await client.list_entities_services()
            events.put(("entities", client, entities, services))
            client.subscribe_states(lambda state: events.put(("state", state)))
            await stopped.wait()
        except aioesphomeapi.APIConnectionError:
            pass
        events.put(("status", f"Waiting for {HOST}:{PORT}"))
        await asyncio.sleep(RETRY_SECONDS)


def describe(state):
    if isinstance(state, ClimateState):
        return (
            f"{state.mode.name}  current {state.current_temperature:.1f}"
            f"  target {state.target_temperature:.1f}"
        )
    value = state.state
    return f"{value:.1f}" if isinstance(value, float) else str(value)


def make_scale(parent, info):
    scale = tk.Scale(
        parent,
        orient=tk.HORIZONTAL,
        label=info.name,
        from_=info.min_value,
        to=info.max_value,
        resolution=info.step,
        length=420,
    )
    scale.bind(
        "<ButtonRelease-1>",
        lambda _e: send("number_command", info.key, scale.get()),
    )
    scale.pack(fill=tk.X, padx=6)
    return scale


def main():
    root = tk.Tk()
    root.title("Grill Display Sim Panel")
    status = tk.StringVar(value="Connecting")
    tk.Label(root, textvariable=status, anchor="w").pack(fill=tk.X, padx=6)

    buttons = tk.Frame(root)
    buttons.pack(fill=tk.X, padx=6, pady=4)
    for text, name, hold_ms in BUTTONS:
        tk.Button(
            buttons,
            text=text,
            command=lambda n=name, h=hold_ms: send(
                "execute_service", link["press"], {"input": n, "hold_ms": h}
            ),
        ).pack(side=tk.LEFT, expand=True, fill=tk.X)

    weather = tk.LabelFrame(root, text="Weather and solar")
    grill = tk.LabelFrame(root, text="Simulated grill")
    device = tk.LabelFrame(root, text="Device")
    for frame in (weather, grill, device):
        frame.pack(fill=tk.X, padx=6, pady=4)

    scales = {}
    meat_probe = tk.BooleanVar()
    readouts = {}

    def build(client, entities, services):
        for frame in (weather, grill, device):
            for child in frame.winfo_children():
                child.destroy()
        scales.clear()
        readouts.clear()
        link["client"] = client
        link["press"] = next(s for s in services if s.name == "press")
        by_id = {(type(e), e.object_id): e for e in entities}
        for info in entities:
            if not isinstance(info, NumberInfo):
                continue
            if info.object_id.startswith(WEATHER_PREFIXES):
                scales[info.key] = make_scale(weather, info)
            elif info.object_id.startswith(GRILL_PREFIXES):
                scales[info.key] = make_scale(grill, info)
        probe = by_id.get((SwitchInfo, "sim_meat_probe"))
        if probe is not None:
            readouts[probe.key] = meat_probe
            tk.Checkbutton(
                grill,
                text=probe.name,
                variable=meat_probe,
                command=lambda: send("switch_command", probe.key, meat_probe.get()),
            ).pack(anchor="w", padx=6)
        for label, kind, object_id in DEVICE_ROWS:
            info = by_id.get((kind, object_id))
            if info is None:
                print("missing", label, object_id)
                print("object_ids:", sorted(by_id, key=str))
                continue
            var = tk.StringVar(value="--")
            readouts[info.key] = var
            row = tk.Frame(device)
            row.pack(fill=tk.X, padx=6)
            tk.Label(row, text=label, width=18, anchor="w").pack(side=tk.LEFT)
            tk.Label(row, textvariable=var, anchor="w").pack(side=tk.LEFT)
        status.set(f"Connected to {HOST}:{PORT}")

    def apply(state):
        if getattr(state, "missing_state", False):
            return
        if state.key in scales:
            scales[state.key].set(state.state)
        elif state.key in readouts:
            target = readouts[state.key]
            if isinstance(target, tk.BooleanVar):
                target.set(state.state)
            else:
                target.set(describe(state))

    def drain():
        while not events.empty():
            kind, *payload = events.get()
            if kind == "entities":
                build(*payload)
            elif kind == "state":
                apply(*payload)
            else:
                link["client"] = None
                status.set(*payload)
        root.after(100, drain)

    drain()
    threading.Thread(target=loop.run_forever, daemon=True).start()
    asyncio.run_coroutine_threadsafe(session(), loop)
    root.mainloop()


main()
