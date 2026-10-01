"""Tests for storage json parsing and command assembly."""

import json
import stat
from pathlib import Path

import pytest

from esphome_sim.errors import SimError
from esphome_sim.runner import (
    compile_command,
    program_command,
    read_program_path,
    storage_path,
)


def write_storage(path: Path, document: object) -> Path:
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def make_program(path: Path, mode: int = 0o755) -> Path:
    path.write_text("#!/bin/sh\n")
    path.chmod(mode)
    return path


def test_storage_path_default() -> None:
    config = Path("/repo/tests/demo-host.yaml")
    assert storage_path(config, {}) == Path(
        "/repo/tests/.esphome/storage/demo-host.yaml.json"
    )


def test_storage_path_data_dir_env() -> None:
    config = Path("/repo/tests/demo-host.yaml")
    env = {"ESPHOME_DATA_DIR": "/data"}
    assert storage_path(config, env) == Path("/data/storage/demo-host.yaml.json")


def test_read_program_path(tmp_path: Path) -> None:
    program = make_program(tmp_path / "program")
    storage = write_storage(tmp_path / "s.json", {"firmware_bin_path": str(program)})
    assert read_program_path(storage) == program


@pytest.mark.parametrize(
    "document",
    [
        [],
        {},
        {"firmware_bin_path": None},
        {"firmware_bin_path": ""},
        {"firmware_bin_path": 3},
    ],
)
def test_read_program_path_rejects_bad_document(
    tmp_path: Path, document: object
) -> None:
    storage = write_storage(tmp_path / "s.json", document)
    with pytest.raises(SimError):
        read_program_path(storage)


def test_read_program_path_missing_storage(tmp_path: Path) -> None:
    with pytest.raises(SimError, match="cannot read"):
        read_program_path(tmp_path / "missing.json")


def test_read_program_path_invalid_json(tmp_path: Path) -> None:
    storage = tmp_path / "s.json"
    storage.write_text("{")
    with pytest.raises(SimError, match="cannot read"):
        read_program_path(storage)


def test_read_program_path_missing_program(tmp_path: Path) -> None:
    storage = write_storage(
        tmp_path / "s.json", {"firmware_bin_path": str(tmp_path / "nope")}
    )
    with pytest.raises(SimError, match="does not exist"):
        read_program_path(storage)


def test_read_program_path_not_executable(tmp_path: Path) -> None:
    program = make_program(tmp_path / "program", stat.S_IRUSR | stat.S_IWUSR)
    storage = write_storage(tmp_path / "s.json", {"firmware_bin_path": str(program)})
    with pytest.raises(SimError, match="not executable"):
        read_program_path(storage)


def test_program_command() -> None:
    program = Path("/build/program")
    assert program_command(program, None) == ["/build/program"]
    assert program_command(program, "/bin/nixGLIntel") == [
        "/bin/nixGLIntel",
        "/build/program",
    ]


def test_compile_command_forwards_args() -> None:
    command = compile_command(Path("tests/demo-host.yaml"), ["--only-generate"])
    assert command == ["esphome", "compile", "tests/demo-host.yaml", "--only-generate"]
