"""Drive the grill display host simulation through its `press` API action."""

import asyncio
import sys

import aioesphomeapi

HOST = "127.0.0.1"
PORT = 6053
KEY = "j/+/ENFUx0rHq87Yz1/Cll3lOqfVXsQ3TC+zXp/A9CI="
DEFAULT_HOLD_MS = 100

USAGE = """usage: grill-display-sim-press [-v] STEP...

Steps:
  NAME[:HOLD_MS]  press an input, NAME is one of
                  rotary_up, rotary_down, rotary_press, menu, exit
  sleep:SECONDS   wait
  state           print entity states

Options:
  -v              print the device log
"""


def describe(state):
    if hasattr(state, "mode"):
        return (
            f"mode={state.mode} current={state.current_temperature} "
            f"target={state.target_temperature}"
        )
    return getattr(state, "state", None)


async def main(args):
    verbose = "-v" in args
    steps = [a for a in args if a != "-v"]
    client = aioesphomeapi.APIClient(HOST, PORT, None, noise_psk=KEY)
    await client.connect(login=True)
    if verbose:
        client.subscribe_logs(
            lambda m: print(m.message.decode(errors="replace")),
            log_level=aioesphomeapi.LogLevel.LOG_LEVEL_VERBOSE,
        )
    entities, services = await client.list_entities_services()
    press = next(s for s in services if s.name == "press")
    names = {e.key: e.name for e in entities}

    def on_state(state):
        print(f"{names.get(state.key, state.key)}: {describe(state)}")

    for step in steps:
        if step == "state":
            client.subscribe_states(on_state)
            await asyncio.sleep(2)
        elif step.startswith("sleep:"):
            await asyncio.sleep(float(step.removeprefix("sleep:")))
        else:
            name, _, hold = step.partition(":")
            hold_ms = int(hold) if hold else DEFAULT_HOLD_MS
            await client.execute_service(press, {"input": name, "hold_ms": hold_ms})
            await asyncio.sleep(hold_ms / 1000 + 0.3)
    if verbose:
        await asyncio.sleep(3)
    await client.disconnect()


if len(sys.argv) < 2:
    sys.exit(USAGE)
asyncio.run(main(sys.argv[1:]))
