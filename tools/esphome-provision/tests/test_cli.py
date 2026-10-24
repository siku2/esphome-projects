import argparse
from pathlib import Path

import pytest

from esphome_provision import cli
from esphome_provision.profiles import all_profiles, detect_profile


def parse(*argv: str) -> argparse.Namespace:
    return cli.build_parser().parse_args(list(argv))


def test_install_options() -> None:
    args = parse(
        "install", "--port", "/dev/ttyACM0", "--backup-dir", "b",
        "--firmware", "f.bin", "--profile", "zeromouse", "--board", "v2",
        "--device", "kopi", "--observe-only", "--s3-endpoint", "e", "--s3-bucket", "k",
    )  # fmt: skip
    assert args.handler is cli.cmd_install
    assert args.board == "v2"
    assert args.observe_only
    assert not args.resume


def test_resume_options() -> None:
    args = parse(
        "install", "--resume", "--backup-dir", "b", "--host", "10.0.0.5",
        "--api-key", "k", "--device", "kopi",
    )  # fmt: skip
    assert args.resume
    assert args.firmware is None


def test_profile_commands() -> None:
    args = parse("zeromouse", "cloud", "--host", "h", "--backup-dir", "b")
    assert args.nvs is None
    assert args.backup_dir == Path("b")
    for command in ("s3", "clear"):
        assert parse("zeromouse", command, "--host", "h").zeromouse_command == command
    with pytest.raises(SystemExit):
        parse("zeromouse", "cloud", "--host", "h")


def test_restore_needs_a_source() -> None:
    with pytest.raises(SystemExit):
        parse("restore", "--port", "p")
    assert parse("restore", "--port", "p", "--backup-dir", "b").backup_dir == Path("b")


def test_failure_hint_has_restore_and_resume_commands() -> None:
    args = parse(
        "install", "--port", "/dev/ttyACM0", "--backup-dir", "b dir",
        "--firmware", "f.bin", "--board", "v2", "--device", "kopi",
        "--observe-only", "--api-key", "secret",
    )  # fmt: skip
    profile = all_profiles()["zeromouse"]
    resume = cli.resume_command(args, profile, "10.0.0.5")
    assert resume.startswith("nix run .#esphome-provision -- install --resume")
    assert "--backup-dir 'b dir' --host 10.0.0.5 --profile zeromouse" in resume
    assert "--device kopi --observe-only" in resume
    assert "secret" not in resume
    restore = cli.restore_command(args.port, args.backup_dir)
    assert restore.startswith(
        "nix run .#esphome-provision -- restore --port /dev/ttyACM0"
    )
    assert "<device address>" in cli.resume_command(args, profile, None)


def test_profile_detection() -> None:
    profile = all_profiles()["zeromouse"]
    assert detect_profile(b"xx" + profile.project_name + b"yy") is profile
    assert detect_profile(b"nothing here") is None


def test_logs_options() -> None:
    args = parse("logs", "bean-kopi", "--host", "h")
    assert args.handler is cli.logs.cmd_logs
    assert args.device == "bean-kopi"
    assert args.level == "very_verbose"
    assert args.ha_ssh is None
