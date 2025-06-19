#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = []
# ///

from pathlib import Path
import subprocess
from typing import Protocol, cast
import argparse

_PROJECT_ROOT = Path(__file__).parent
_PROJECTS_DIR = _PROJECT_ROOT / "projects"


class Args(Protocol):
    pass


def parse_args() -> Args:
    parser = argparse.ArgumentParser("x.py")
    _subparsers = parser.add_subparsers()

    args = parser.parse_args()
    return cast(Args, args)


def main() -> None:
    _args = parse_args()

    generate_interactive_bom(
        _PROJECTS_DIR / "warema-cover/hardware/warema-cover.kicad_pcb",
        _PROJECT_ROOT / "public",
    )


def generate_interactive_bom(
    pcb_file: Path,
    dest_dir: Path,
    *,
    dark_mode: bool = True,
) -> None:
    args: list[str] = [
        "xvfb-run",
        "--auto-servernum",
        "--server-args",
        "-screen 0 1024x768x24",
        "generate_interactive_bom",
        "--no-browser",
        "--dest-dir",
        str(dest_dir),
    ]
    if dark_mode:
        args.append("--dark-mode")
    args.append(str(pcb_file))
    subprocess.run(
        args,
        check=True,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        timeout=60,
    )


if __name__ == "__main__":
    main()
