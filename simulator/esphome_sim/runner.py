"""Compile a project, run the host program and show the panel alongside."""

import json
import os
import shutil
import signal
import subprocess
import threading
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import FrameType

from esphome_sim.errors import SimError
from esphome_sim.panel import Panel
from esphome_sim.project import Project

FORWARDED_SIGNALS = (signal.SIGINT, signal.SIGTERM)
GL_WRAPPER = "nixGLIntel"
DATA_DIR_ENV = "ESPHOME_DATA_DIR"


def storage_path(config: Path, env: Mapping[str, str]) -> Path:
    """Return the storage json that esphome writes for the config file."""
    data_dir = Path(env[DATA_DIR_ENV]) if env.get(DATA_DIR_ENV) else None
    if data_dir is None:
        data_dir = config.parent / ".esphome"
    return data_dir / "storage" / f"{config.name}.json"


def read_program_path(storage: Path) -> Path:
    """Read `firmware_bin_path` from a storage json and check that it is runnable."""
    try:
        document: object = json.loads(storage.read_text(encoding="utf-8"))
    except (OSError, ValueError) as err:
        raise SimError(f"{storage}: cannot read: {err}") from err
    if not isinstance(document, dict):
        raise SimError(f"{storage}: the document root must be an object")
    value = document.get("firmware_bin_path")
    if not isinstance(value, str) or not value:
        raise SimError(f"{storage}: firmware_bin_path must be a non-empty string")
    program = Path(value)
    if not program.is_file():
        raise SimError(f"{storage}: program {program} does not exist")
    if not os.access(program, os.X_OK):
        raise SimError(f"{storage}: program {program} is not executable")
    return program


def program_command(program: Path, wrapper: str | None) -> list[str]:
    """Return the command that runs `program`, through `wrapper` when given."""
    if wrapper is None:
        return [str(program)]
    return [wrapper, str(program)]


def compile_command(config: Path, esphome_args: Sequence[str]) -> list[str]:
    """Return the `esphome compile` command for `config`."""
    return ["esphome", "compile", str(config), *esphome_args]


def run(repo_root: Path, project: Project, esphome_args: Sequence[str]) -> int:
    """Compile and start the simulation, show the panel, return the program exit code."""
    config = project.path.relative_to(repo_root)
    lock = threading.Lock()
    current: list[subprocess.Popen[bytes]] = []
    interrupted = threading.Event()
    stop = threading.Event()
    result: list[int] = []
    failure: list[SimError] = []

    def spawn(command: Sequence[str]) -> int:
        try:
            # A separate process group, so that a terminal Ctrl-C arrives only once.
            process = subprocess.Popen(
                command, cwd=repo_root, stdin=subprocess.DEVNULL, process_group=0
            )
        except FileNotFoundError as err:
            raise SimError(f"{command[0]} is not on PATH") from err
        with lock:
            current.append(process)
            if interrupted.is_set():
                os.killpg(process.pid, signal.SIGINT)
        return process.wait()

    def work() -> None:
        try:
            code = spawn(compile_command(config, esphome_args))
            if code == 0 and not interrupted.is_set():
                program = read_program_path(storage_path(project.path, os.environ))
                wrapper = shutil.which(GL_WRAPPER)
                if wrapper is None:
                    print(f"esphome-sim: {GL_WRAPPER} not found, running without it")
                else:
                    print(f"esphome-sim: running through {GL_WRAPPER}")
                code = spawn(program_command(program, wrapper))
            result.append(code)
        except SimError as err:
            failure.append(err)
        finally:
            stop.set()

    def forward(signum: int, _frame: FrameType | None) -> None:
        interrupted.set()
        with lock:
            for process in current:
                if process.poll() is None:
                    os.killpg(process.pid, signum)

    previous = {sig: signal.signal(sig, forward) for sig in FORWARDED_SIGNALS}
    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    try:
        Panel(project, stop).run()
        worker.join()
    except BaseException:
        forward(signal.SIGINT, None)
        worker.join()
        raise
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if failure:
        raise failure[0]
    return result[0]
