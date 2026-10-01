"""Run `esphome run` for a project with the panel alongside."""

import os
import signal
import subprocess
import threading
from collections.abc import Sequence
from pathlib import Path
from types import FrameType

from esphome_sim.errors import SimError
from esphome_sim.panel import Panel
from esphome_sim.project import Project

FORWARDED_SIGNALS = (signal.SIGINT, signal.SIGTERM)


def run(repo_root: Path, project: Project, esphome_args: Sequence[str]) -> int:
    """Build and start the simulation, show the panel, return the esphome exit code."""
    command = [
        "esphome",
        "run",
        str(project.path.relative_to(repo_root)),
        *esphome_args,
    ]
    # The SDL display never maps its window on native Wayland, X11 works.
    env = {"SDL_VIDEODRIVER": "x11", **os.environ} if "DISPLAY" in os.environ else None
    try:
        # A separate process group, so that a terminal Ctrl-C arrives only once.
        process = subprocess.Popen(
            command, cwd=repo_root, stdin=subprocess.DEVNULL, process_group=0, env=env
        )
    except FileNotFoundError as err:
        raise SimError("esphome is not on PATH") from err

    def forward(signum: int, _frame: FrameType | None) -> None:
        if process.poll() is None:
            os.killpg(process.pid, signum)

    previous = {sig: signal.signal(sig, forward) for sig in FORWARDED_SIGNALS}
    stop = threading.Event()

    def wait_for_esphome() -> None:
        process.wait()
        stop.set()

    threading.Thread(target=wait_for_esphome, daemon=True).start()
    try:
        Panel(project, stop).run()
        return process.wait()
    except BaseException:
        forward(signal.SIGINT, None)
        process.wait()
        raise
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
