import base64
from pathlib import Path

import pytest

from esphome_provision import cli, upload
from esphome_provision.common import ProvisionError


def test_password_is_the_key_bytes_as_lowercase_hex() -> None:
    key = base64.b64encode(bytes(range(32))).decode()
    assert upload.ota_password(key) == bytes(range(32)).hex()
    assert upload.ota_password(api_zero()) == "0" * 64


def api_zero() -> str:
    return base64.b64encode(bytes(32)).decode()


def test_default_firmware_path() -> None:
    assert upload.default_firmware("bean-kopi") == Path(
        "devices/.esphome/build/bean-kopi/build/firmware.ota.bin"
    )


def test_unchanged_compilation_time_means_the_image_did_not_boot() -> None:
    with pytest.raises(ProvisionError, match="did not boot"):
        upload.check_booted("Oct 1 2026, 10:00:00", "Oct 1 2026, 10:00:00")
    upload.check_booted("Oct 1 2026, 10:00:00", "Oct 2 2026, 11:00:00")


def test_upload_options() -> None:
    args = cli.build_parser().parse_args(
        ["upload", "bean-kopi", "--firmware", "f.bin", "--host", "h", "--api-key", "k"]
    )
    assert args.handler is upload.cmd_upload
    assert args.firmware == Path("f.bin")
    assert args.device == "bean-kopi"
    defaults = cli.build_parser().parse_args(["upload", "bean-kopi"])
    assert defaults.firmware is None
    assert defaults.host is None


def test_missing_firmware_is_an_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    args = cli.build_parser().parse_args(["upload", "bean-kopi", "--api-key", "k"])
    with pytest.raises(ProvisionError, match="Firmware not found"):
        upload.cmd_upload(args)
