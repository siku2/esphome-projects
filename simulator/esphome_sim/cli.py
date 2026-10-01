"""The `esphome-sim` command line interface."""

import argparse
import asyncio
import enum
import sys
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

from esphome_sim import runner
from esphome_sim.client import SimClient
from esphome_sim.errors import ConnectError, SimError
from esphome_sim.model import EntityKind, EntityState, LogLevel
from esphome_sim.panel import Panel
from esphome_sim.project import (
    find_repo_root,
    list_projects,
    load_project,
    project_path,
)
from esphome_sim.validate import parse_bool, parse_float, parse_service_args

PROG = "esphome-sim"
USAGE_ERROR = 2
CONNECT_ERROR = 1
DEFAULT_TIMEOUT = 10.0
STATE_TIMEOUT = 3.0
LOG_LEVELS = [level.name.lower() for level in LogLevel if level is not LogLevel.NONE]


class CommandName(enum.StrEnum):
    """The subcommands."""

    RUN = "run"
    PANEL = "panel"
    PRESS = "press"
    SET = "set"
    CALL = "call"
    STATE = "state"
    LOG = "log"
    PROJECTS = "projects"


@dataclass(slots=True, frozen=True)
class Options:
    """Validated command line arguments."""

    command: CommandName
    root: Path
    timeout: float
    project: str = ""
    object_ids: tuple[str, ...] = ()
    value: str = ""
    service: str = ""
    pairs: tuple[tuple[str, str], ...] = ()
    follow: bool = False
    level: LogLevel = LogLevel.DEBUG
    esphome_args: tuple[str, ...] = ()


type ClientAction = Callable[[SimClient], Awaitable[None]]


def main(argv: Sequence[str] | None = None) -> None:
    """Parse arguments, run the command and exit."""
    try:
        options = parse_args(sys.argv[1:] if argv is None else argv)
        code = execute(options)
    except ConnectError as err:
        print(f"{PROG}: error: {err}", file=sys.stderr)
        sys.exit(CONNECT_ERROR)
    except SimError as err:
        print(f"{PROG}: error: {err}", file=sys.stderr)
        sys.exit(USAGE_ERROR)
    except KeyboardInterrupt:
        sys.exit(130)
    sys.exit(code)


def parse_args(argv: Sequence[str]) -> Options:
    """Parse and validate `argv`, which may contain `-- esphome args` for run."""
    own, extra = _split_extra(argv)
    parser = _parser()
    namespace = parser.parse_args(own)
    command = CommandName(_attr(namespace, "command", str))
    if extra and command is not CommandName.RUN:
        parser.error("arguments after -- are only allowed for run")
    root_arg = getattr(namespace, "root", None)
    if root_arg is None:
        root = find_repo_root(Path.cwd())
    elif isinstance(root_arg, Path):
        root = root_arg.resolve()
    else:
        raise TypeError("--root must be a path")
    if not (root / "tests").is_dir():
        raise SimError(f"{root} has no tests directory")

    options = Options(
        command=command, root=root, timeout=_attr(namespace, "timeout", float)
    )
    if command is CommandName.PROJECTS:
        return options
    project = _attr(namespace, "project", str)
    match command:
        case CommandName.RUN:
            return replace(options, project=project, esphome_args=tuple(extra))
        case CommandName.PANEL:
            return replace(options, project=project)
        case CommandName.PRESS:
            object_id = _attr(namespace, "object_id", str)
            return replace(options, project=project, object_ids=(object_id,))
        case CommandName.SET:
            object_id = _attr(namespace, "object_id", str)
            value = _attr(namespace, "value", str)
            return replace(
                options, project=project, object_ids=(object_id,), value=value
            )
        case CommandName.CALL:
            pairs = tuple(_pair(item) for item in _attr(namespace, "args", list))
            service = _attr(namespace, "service", str)
            return replace(options, project=project, service=service, pairs=pairs)
        case CommandName.STATE:
            object_ids = tuple(
                _str(item) for item in _attr(namespace, "object_ids", list)
            )
            follow = _attr(namespace, "follow", bool)
            return replace(
                options, project=project, object_ids=object_ids, follow=follow
            )
        case CommandName.LOG:
            level = LogLevel[_attr(namespace, "level", str).upper()]
            return replace(options, project=project, level=level)


def execute(options: Options) -> int:
    """Run the command described by `options` and return the exit code."""
    if options.command is CommandName.PROJECTS:
        for name in list_projects(options.root):
            path = project_path(options.root, name).relative_to(options.root)
            print(f"{name}\t{path}")
        return 0
    project = load_project(options.root, options.project)
    match options.command:
        case CommandName.RUN:
            return runner.run(options.root, project, options.esphome_args)
        case CommandName.PANEL:
            Panel(project).run()
            return 0
    action = _client_action(options)
    asyncio.run(_with_client(SimClient(project), options.timeout, action))
    return 0


def _client_action(options: Options) -> ClientAction:
    match options.command:
        case CommandName.PRESS:
            return lambda client: _press(client, options.object_ids[0])
        case CommandName.SET:
            return lambda client: _set(client, options.object_ids[0], options.value)
        case CommandName.CALL:
            return lambda client: _call(client, options.service, options.pairs)
        case CommandName.STATE:
            return lambda client: _state(client, options.object_ids, options.follow)
        case CommandName.LOG:
            return lambda client: _log(client, options.level)
    raise ValueError(f"{options.command} needs no client")


async def _with_client(client: SimClient, timeout: float, action: ClientAction) -> None:
    await client.connect(timeout)
    try:
        await action(client)
    finally:
        await client.disconnect()


async def _press(client: SimClient, object_id: str) -> None:
    client.press_button(object_id)


async def _set(client: SimClient, object_id: str, value: str) -> None:
    entity = client.entity(object_id)
    match entity.kind:
        case EntityKind.NUMBER:
            client.set_number(object_id, parse_float(value, object_id))
        case EntityKind.SWITCH:
            client.set_switch(object_id, parse_bool(value, object_id))
        case EntityKind.SELECT:
            client.set_select(object_id, value)
        case _:
            raise SimError(
                f"{object_id} is a {entity.kind}, set supports number, switch and select"
            )


async def _call(client: SimClient, name: str, pairs: Sequence[tuple[str, str]]) -> None:
    args = parse_service_args(client.service(name), pairs)
    await client.call_service(name, **args)


async def _state(client: SimClient, object_ids: Sequence[str], follow: bool) -> None:
    if object_ids:
        wanted = {client.entity(object_id).object_id for object_id in object_ids}
    else:
        wanted = {
            entity.object_id
            for entity in client.entities.values()
            if entity.kind not in (EntityKind.BUTTON, EntityKind.OTHER)
        }
    seen: dict[str, EntityState] = {}
    complete = asyncio.Event()

    def on_state(state: EntityState) -> None:
        object_id = state.entity.object_id
        if object_id not in wanted:
            return
        if follow:
            print(f"{object_id}: {state.value}", flush=True)
            return
        seen[object_id] = state
        if wanted <= seen.keys():
            complete.set()

    client.subscribe(on_state)
    if follow:
        await client.wait_closed()
        raise ConnectError("the simulation closed the connection")
    try:
        await asyncio.wait_for(complete.wait(), STATE_TIMEOUT)
    except TimeoutError:
        pass
    for object_id in sorted(wanted):
        state = seen.get(object_id)
        print(f"{object_id}: {state.value if state else 'no state'}")


async def _log(client: SimClient, level: LogLevel) -> None:
    client.subscribe_logs(lambda line: print(line, flush=True), level)
    await client.wait_closed()
    raise ConnectError("the simulation closed the connection")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=PROG, description="Drive ESPHome host simulations."
    )
    parser.add_argument(
        "--root", type=Path, help="repository root (default: closest flake.nix)"
    )
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        default=DEFAULT_TIMEOUT,
        help=f"seconds to wait for the simulation (default: {DEFAULT_TIMEOUT:g})",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    run = commands.add_parser(
        "run",
        help="build and run the simulation with the panel, pass esphome args after --",
    )
    run.add_argument("project")
    panel = commands.add_parser("panel", help="show the panel for a running simulation")
    panel.add_argument("project")
    press = commands.add_parser("press", help="press a button entity")
    press.add_argument("project")
    press.add_argument("object_id")
    set_ = commands.add_parser(
        "set", help="set a number, switch (on/off) or select (option) entity"
    )
    set_.add_argument("project")
    set_.add_argument("object_id")
    set_.add_argument("value")
    call = commands.add_parser("call", help="call a user service (API action)")
    call.add_argument("project")
    call.add_argument("service")
    call.add_argument("args", nargs="*", metavar="key=value", type=_key_value)
    state = commands.add_parser("state", help="print entity states")
    state.add_argument("project")
    state.add_argument("object_ids", nargs="*", metavar="object_id")
    state.add_argument("--follow", action="store_true", help="keep printing updates")
    log = commands.add_parser("log", help="print the device log")
    log.add_argument("project")
    log.add_argument("--level", choices=LOG_LEVELS, default="debug")
    commands.add_parser("projects", help="list the simulator projects")
    return parser


def _split_extra(argv: Sequence[str]) -> tuple[list[str], list[str]]:
    args = list(argv)
    if "--" not in args:
        return args, []
    index = args.index("--")
    return args[:index], args[index + 1 :]


def _positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number") from None
    if not value > 0:
        raise argparse.ArgumentTypeError(f"{text!r} is not a positive number")
    return value


def _key_value(text: str) -> tuple[str, str]:
    key, sep, value = text.partition("=")
    if not sep or not key:
        raise argparse.ArgumentTypeError(f"{text!r} is not key=value")
    return key, value


def _attr[T](namespace: argparse.Namespace, name: str, kind: type[T]) -> T:
    value: object = getattr(namespace, name)
    if not isinstance(value, kind):
        raise TypeError(f"argument {name} has type {type(value).__name__}")
    return value


def _str(value: object) -> str:
    if not isinstance(value, str):
        raise TypeError(f"expected str, got {type(value).__name__}")
    return value


def _pair(value: object) -> tuple[str, str]:
    match value:
        case (str() as key, str() as text):
            return key, text
    raise TypeError(f"expected a key=value pair, got {value!r}")
