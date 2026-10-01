"""Tests for command line parsing."""

from pathlib import Path

import pytest

from esphome_sim.cli import CommandName, parse_args
from esphome_sim.model import LogLevel


@pytest.fixture
def root(tmp_path: Path) -> Path:
    (tmp_path / "tests").mkdir()
    return tmp_path


def test_run_passes_esphome_args(root: Path) -> None:
    options = parse_args(["--root", str(root), "run", "demo", "--", "--no-logs"])
    assert options.command is CommandName.RUN
    assert options.project == "demo"
    assert options.esphome_args == ("--no-logs",)


def test_call_pairs(root: Path) -> None:
    options = parse_args(["--root", str(root), "call", "demo", "press", "input=exit"])
    assert options.service == "press"
    assert options.pairs == (("input", "exit"),)


def test_log_level(root: Path) -> None:
    options = parse_args(["--root", str(root), "log", "demo", "--level", "verbose"])
    assert options.level is LogLevel.VERBOSE


@pytest.mark.parametrize(
    "argv",
    [
        ["call", "demo", "press", "input"],
        ["log", "demo", "--level", "loud"],
        ["state", "demo", "--", "x"],
        ["--timeout", "-1", "projects"],
        ["frobnicate"],
    ],
)
def test_usage_errors_exit_2(root: Path, argv: list[str]) -> None:
    with pytest.raises(SystemExit) as exit_info:
        parse_args(["--root", str(root), *argv])
    assert exit_info.value.code == 2
