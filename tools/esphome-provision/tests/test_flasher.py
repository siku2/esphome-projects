import json
import os
import shutil
import subprocess
from datetime import date
from pathlib import Path

import pytest

from esphome_provision.common import ProvisionError
from esphome_provision.firmware import CHIPS
from esphome_provision.flasher import (
    ChipInfo,
    Preflight,
    check_backup_dir,
    default_backup_dir,
    load_backup,
    make_manifest,
    parse_flash_id,
    parse_flash_size,
    sha256_file,
    write_manifest,
)

MIB = 1024 * 1024


def flash_id(chip: str, size: str, extra: str = "") -> str:
    return (
        "esptool v5\n"
        + f"Chip type:          {chip}{extra}\n"
        + "Features:           Wi-Fi\n"
        + "MAC:                aa:bb:cc:dd:ee:ff\n"
        + f"Detected flash size: {size}\n"
    )


def test_flash_id_for_each_chip() -> None:
    for chip in CHIPS:
        info = parse_flash_id(flash_id(chip.label, "4MB", " (revision v1.0)"))
        assert info.chip == chip
        assert info.flash_size == 4 * MIB
        assert info.mac == "aa:bb:cc:dd:ee:ff"


def test_flash_id_with_a_variant_name() -> None:
    info = parse_flash_id(flash_id("ESP32-D0WD-V3", "4MB"))
    assert info.chip.label == "ESP32"


def test_flash_id_refusals() -> None:
    with pytest.raises(ProvisionError, match="download mode"):
        _ = parse_flash_id(flash_id("ESP32-S3 (Secure Download Mode)", "16MB"))
    with pytest.raises(ProvisionError, match="not supported"):
        _ = parse_flash_id(flash_id("ESP8266", "4MB"))
    with pytest.raises(ProvisionError):
        _ = parse_flash_id(flash_id("ESP32-S3", "huge"))
    with pytest.raises(ProvisionError):
        _ = parse_flash_id("Chip type: ESP32-S3\nDetected flash size: 4MB\n")


def test_flash_sizes() -> None:
    assert parse_flash_size("16MB") == 16 * MIB
    assert parse_flash_size("512KB") == 512 * 1024
    with pytest.raises(ProvisionError):
        _ = parse_flash_size("16 MB")


def make_backup_dir(path: Path, size: int = 4096) -> Preflight:
    chip = next(c for c in CHIPS if c.label == "ESP32-S3")
    pre = Preflight(
        ChipInfo(chip, "aa:bb:cc:dd:ee:ff", size), "text", {"BLOCK_USR_DATA": "01 00"}
    )
    path.mkdir()
    _ = (path / "flash.bin").write_bytes(b"\x01" * size)
    write_manifest(path, make_manifest(pre, sha256_file(path / "flash.bin")))
    return pre


def test_manifest_round_trip(tmp_path: Path) -> None:
    pre = make_backup_dir(tmp_path / "backup")
    backup = load_backup(tmp_path / "backup")
    assert backup.mac == pre.chip.mac
    assert backup.chip == pre.chip.chip
    assert backup.flash_size == 4096
    assert backup.efuses == {"BLOCK_USR_DATA": "01 00"}


def test_manifest_detects_changes(tmp_path: Path) -> None:
    _ = make_backup_dir(tmp_path / "backup")
    image = tmp_path / "backup/flash.bin"
    _ = image.write_bytes(b"\x02" * 4096)
    with pytest.raises(ProvisionError, match="sha256"):
        _ = load_backup(tmp_path / "backup")
    _ = image.write_bytes(b"\x01" * 100)
    with pytest.raises(ProvisionError, match="flash size"):
        _ = load_backup(tmp_path / "backup")


def test_manifest_errors(tmp_path: Path) -> None:
    with pytest.raises(ProvisionError):
        _ = load_backup(tmp_path)
    _ = make_backup_dir(tmp_path / "backup")
    manifest = tmp_path / "backup/manifest.json"
    data = json.loads(manifest.read_text())
    data["chip"] = "esp8266"
    _ = manifest.write_text(json.dumps(data))
    with pytest.raises(ProvisionError, match="chip"):
        _ = load_backup(tmp_path / "backup")


def test_legacy_manifest_is_accepted(tmp_path: Path) -> None:
    _ = make_backup_dir(tmp_path / "backup")
    manifest = tmp_path / "backup/manifest.json"
    data = json.loads(manifest.read_text())
    del data["efuses"]
    data["board_id"] = 1
    _ = manifest.write_text(json.dumps(data))
    assert load_backup(tmp_path / "backup").efuses == {"BLOCK_USR_DATA": "01"}


def test_backup_dir_must_be_empty(tmp_path: Path) -> None:
    assert check_backup_dir(tmp_path / "new") == tmp_path / "new"
    (tmp_path / "full").mkdir()
    _ = (tmp_path / "full/file").write_bytes(b"")
    with pytest.raises(ProvisionError, match="empty"):
        _ = check_backup_dir(tmp_path / "full")


def git_repo(path: Path, ignore: str = "") -> Path:
    git = shutil.which("git")
    if git is None:
        pytest.skip("git is not available")
    env = {"GIT_CONFIG_GLOBAL": "/dev/null", "GIT_CONFIG_SYSTEM": "/dev/null"}
    _ = subprocess.run(
        [git, "-c", "init.defaultBranch=main", "init", "-q", str(path)],
        check=True,
        env={**os.environ, **env},
    )
    _ = (path / ".gitignore").write_text(ignore)
    return path


@pytest.fixture(autouse=True)
def no_global_git_config(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", "/dev/null")
    monkeypatch.setenv("GIT_CONFIG_SYSTEM", "/dev/null")


def test_default_backup_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)
    day = date(2026, 10, 24)
    first = default_backup_dir("kopi", day)
    assert first == Path("backups/kopi-2026-10-24")
    first.mkdir(parents=True)
    assert default_backup_dir("kopi", day) == first
    _ = (first / "file").write_bytes(b"")
    second = default_backup_dir("kopi", day)
    assert second == Path("backups/kopi-2026-10-24-2")
    second.mkdir()
    _ = (second / "file").write_bytes(b"")
    assert default_backup_dir("kopi", day) == Path("backups/kopi-2026-10-24-3")


def test_backup_dir_ignored_in_own_repo_is_allowed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = git_repo(tmp_path / "repo", "/backups/\n")
    monkeypatch.chdir(repo)
    assert check_backup_dir(repo / "backups/x") == repo / "backups/x"


def test_backup_dir_not_ignored_in_own_repo_is_refused(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = git_repo(tmp_path / "repo", "/backups/\n")
    monkeypatch.chdir(repo)
    with pytest.raises(ProvisionError, match="not ignored"):
        _ = check_backup_dir(repo / "other/x")


def test_backup_dir_in_other_repo_is_allowed_with_a_note(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    own = git_repo(tmp_path / "own")
    other = git_repo(tmp_path / "other")
    monkeypatch.chdir(own)
    _ = check_backup_dir(other / "backups/x")
    output = capsys.readouterr().out
    assert f"git work tree {other.resolve()}" in output
    assert "private repository" in output


def test_backup_dir_outside_any_repo_is_allowed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    own = git_repo(tmp_path / "own")
    monkeypatch.chdir(own)
    _ = check_backup_dir(tmp_path / "plain/x")
    assert capsys.readouterr().out == ""
