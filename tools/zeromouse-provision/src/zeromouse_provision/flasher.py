"""Chip checks, backups, firmware checks and flashing through the esptool CLIs."""

import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import threading
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from zeromouse_provision.common import ProvisionError, strip_ansi
from zeromouse_provision.efuse import (
    NAMES,
    EfuseInfo,
    check_board,
    check_safe_to_flash,
    parse_summary_json,
    parse_summary_text,
)
from zeromouse_provision.partitions import parse_partition_table

CHIP = "esp32s3"
CHIP_ID_ESP32S3 = 9
FLASH_SIZE = 16 * 1024 * 1024
PROJECT_NAME = b"siku2.zeromouse"
# A driver tag that only the build for each board contains.
BOARD_MARKERS = {"v1": b"pt2046b", "v2": b"sths34pf80"}
TOOL = "zeromouse_provision 1"
SHORT_TIMEOUT = 120
LONG_TIMEOUT = 1800
KEEP = ["--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]


def run(cmd: list[str], *, what: str, timeout: float, show: bool = False) -> str:
    """Run a command with a timeout and return its combined output."""
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
        )
    except FileNotFoundError as error:
        raise ProvisionError(f"{cmd[0]} was not found on PATH") from error
    timed_out = threading.Event()

    def kill() -> None:
        timed_out.set()
        process.kill()

    timer = threading.Timer(timeout, kill)
    timer.start()
    lines: list[str] = []
    try:
        assert process.stdout is not None
        for line in process.stdout:
            clean = strip_ansi(line)
            lines.append(clean)
            if show:
                print(clean, end="", flush=True)
        _ = process.wait()
    finally:
        timer.cancel()
        if process.poll() is None:
            process.kill()
            _ = process.wait()
    output = "".join(lines)
    if timed_out.is_set():
        raise ProvisionError(f"{what} timed out after {timeout:.0f} s")
    if process.returncode != 0:
        tail = "".join(lines[-8:]).strip()
        raise ProvisionError(
            f"{what} failed with exit code {process.returncode}\n{tail}"
        )
    return output


def esptool(
    port: str,
    *args: str,
    what: str,
    timeout: float = SHORT_TIMEOUT,
    after: str = "no-reset",
    show: bool = False,
) -> str:
    cmd = ["esptool", "--chip", CHIP, "--port", port, "--after", after, *args]
    return run(cmd, what=what, timeout=timeout, show=show)


@dataclass(frozen=True)
class ChipInfo:
    mac: str
    flash_size: int


def parse_flash_id(output: str) -> ChipInfo:
    chip = re.search(r"^Chip type:\s+(.+)$", output, re.MULTILINE)
    if (
        chip is None
        or not chip.group(1).startswith("ESP32-S3")
        or "Secure Download" in chip.group(1)
    ):
        raise ProvisionError("The chip is not an ESP32-S3 in normal download mode")
    mac = re.search(
        r"^MAC:\s+((?:[0-9a-f]{2}:){5}[0-9a-f]{2})\s*$", output, re.MULTILINE
    )
    if mac is None:
        raise ProvisionError("Could not read the MAC address from esptool")
    size = re.search(r"^Detected flash size:\s+(\S+)\s*$", output, re.MULTILINE)
    if size is None or size.group(1) != "16MB":
        found = size.group(1) if size else "unknown"
        raise ProvisionError(f"The flash size is {found}, expected 16MB")
    return ChipInfo(mac.group(1), FLASH_SIZE)


@dataclass(frozen=True)
class Preflight:
    chip: ChipInfo
    efuses: EfuseInfo
    efuse_text: str


def read_efuses(port: str) -> tuple[EfuseInfo, str]:
    base = ["espefuse", "--chip", CHIP, "--port", port]
    as_json = run(
        [*base, "summary", "--format", "json", *NAMES],
        what="Reading eFuses",
        timeout=SHORT_TIMEOUT,
    )
    as_text = run(
        [*base, "summary"], what="Reading the eFuse summary", timeout=SHORT_TIMEOUT
    )
    info = parse_summary_json(as_json)
    if parse_summary_text(as_text) != info:
        raise ProvisionError("The JSON and text eFuse summaries disagree")
    return info, as_text


def preflight(port: str, board: str | None) -> Preflight:
    print(f"Checking the chip on {port}")
    chip = parse_flash_id(
        esptool(port, "flash-id", what="Reading the chip and flash id")
    )
    efuses, text = read_efuses(port)
    check_safe_to_flash(efuses)
    if board is not None:
        check_board(efuses, board)
    print(
        f"  ESP32-S3, MAC {chip.mac}, 16 MB flash, board id {efuses.board_id}, plain flash, no secure boot"
    )
    return Preflight(chip, efuses, text)


def check_mac(chip: ChipInfo, expected: str, *, allow_other: bool) -> None:
    if chip.mac.lower() != expected.lower():
        if not allow_other:
            raise ProvisionError(
                f"The device MAC {chip.mac} does not match the expected {expected}"
            )
        print(
            f"Warning: the device MAC {chip.mac} differs from the expected {expected}"
        )


def ensure_outside_git(path: Path) -> None:
    parent = path
    while not parent.exists():
        parent = parent.parent
    try:
        result = subprocess.run(
            ["git", "-C", str(parent), "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as error:
        raise ProvisionError(
            "Cannot run git to check that the backup directory is outside a repository"
        ) from error
    if result.returncode == 0:
        raise ProvisionError(
            f"The backup directory is inside the git work tree {result.stdout.strip()}"
        )
    if "not a git repository" not in result.stderr:
        raise ProvisionError(
            "git could not tell whether the backup directory is in a work tree"
        )


def check_backup_dir(path: Path) -> Path:
    path = path.expanduser().absolute()
    if path.exists():
        if not path.is_dir() or any(path.iterdir()):
            raise ProvisionError(
                f"The backup directory {path} must be empty or not exist"
            )
    ensure_outside_git(path.resolve())
    return path


def write_private(path: Path, data: bytes) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as handle:
        _ = handle.write(data)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while chunk := handle.read(1 << 20):
            digest.update(chunk)
    return digest.hexdigest()


def make_backup(port: str, directory: Path, pre: Preflight) -> Path:
    """Read the whole flash, verify it against the chip and write the manifest."""
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    directory.chmod(0o700)
    if any(directory.iterdir()):
        raise ProvisionError(f"The backup directory {directory} is not empty")
    write_private(directory / "efuses.txt", pre.efuse_text.encode())
    part = directory / "flash.bin.part"
    print("Reading the full flash")
    _ = esptool(
        port,
        "read-flash",
        "0",
        hex(FLASH_SIZE),
        str(part),
        what="Reading the flash",
        timeout=LONG_TIMEOUT,
        show=True,
    )
    part.chmod(0o600)
    if part.stat().st_size != FLASH_SIZE:
        raise ProvisionError("The flash dump has the wrong size")
    print("Verifying the dump against the chip")
    output = esptool(
        port,
        "verify-flash",
        "0",
        str(part),
        what="Verifying the backup",
        timeout=LONG_TIMEOUT,
        show=True,
    )
    if "digest matched" not in output:
        raise ProvisionError("The backup was not verified by the chip")
    image = directory / "flash.bin"
    _ = part.replace(image)
    digest = sha256_file(image)
    write_private(directory / "flash.bin.sha256", f"{digest}  flash.bin\n".encode())
    manifest = {
        "mac": pre.chip.mac,
        "chip": CHIP,
        "flash_size": FLASH_SIZE,
        "board_id": pre.efuses.board_id,
        "sha256": digest,
        "created_at": datetime.now(UTC).isoformat(timespec="seconds"),
        "tool": TOOL,
    }
    write_private(
        directory / "manifest.json", (json.dumps(manifest, indent=2) + "\n").encode()
    )
    print(f"Backup saved in {directory}")
    return image


def load_backup(directory: Path) -> tuple[dict[str, str | int], Path]:
    try:
        manifest = json.loads((directory / "manifest.json").read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ProvisionError(f"Cannot read {directory}/manifest.json") from error
    image = directory / "flash.bin"
    if not isinstance(manifest, dict) or not image.is_file():
        raise ProvisionError("The backup directory is incomplete")
    if manifest.get("chip") != CHIP or manifest.get("flash_size") != FLASH_SIZE:
        raise ProvisionError("The backup manifest is not for a 16 MB ESP32-S3")
    if image.stat().st_size != FLASH_SIZE:
        raise ProvisionError("flash.bin is not exactly 16 MiB")
    if sha256_file(image) != manifest.get("sha256"):
        raise ProvisionError("flash.bin does not match the sha256 in the manifest")
    if not isinstance(manifest.get("mac"), str) or not isinstance(
        manifest.get("board_id"), int
    ):
        raise ProvisionError("The backup manifest has no MAC address or board id")
    return manifest, image


def write_image(port: str, address: int, image: Path, *, what: str) -> None:
    output = esptool(
        port,
        "write-flash",
        *KEEP,
        hex(address),
        str(image),
        what=what,
        timeout=LONG_TIMEOUT,
        after="hard-reset",
        show=True,
    )
    if "Hash of data verified" not in output:
        raise ProvisionError(f"{what} was not verified by the chip")


def restore(
    port: str,
    image: Path,
    expected_mac: str,
    *,
    allow_other: bool,
    board_id: int | None,
) -> None:
    if image.stat().st_size != FLASH_SIZE:
        raise ProvisionError("The image must be exactly 16 MiB")
    pre = preflight(port, None)
    check_mac(pre.chip, expected_mac, allow_other=allow_other)
    if board_id is not None and pre.efuses.board_id != board_id:
        raise ProvisionError("The board id of this device differs from the backup")
    write_image(port, 0, image, what="Restoring the image")
    print("Restore finished and verified. The device was reset.")


def check_firmware(path: Path, board: str) -> bytes:
    """Check that the file is an ESP32-S3 factory image of the ZeroMOUSE project for the board."""
    try:
        data = path.read_bytes()
    except OSError as error:
        raise ProvisionError(f"Cannot read the firmware {path}") from error
    if len(data) < 0x10000 or data[0] != 0xE9:
        raise ProvisionError("The firmware is not an ESP image")
    (chip_id,) = struct.unpack_from("<H", data, 12)
    if chip_id != CHIP_ID_ESP32S3:
        raise ProvisionError(f"The firmware is for chip id {chip_id}, not the ESP32-S3")
    if data[3] >> 4 != 4:
        raise ProvisionError("The firmware header does not declare 16 MB flash")
    if len(data) > FLASH_SIZE:
        raise ProvisionError("The firmware is larger than the flash")
    try:
        table = parse_partition_table(data, 0x8000)
    except ProvisionError as error:
        raise ProvisionError(
            "The firmware is not a factory image, it has no partition table at 0x8000"
        ) from error
    if not any(p.type == 0 and p.offset < len(data) for p in table):
        raise ProvisionError("The firmware has no app partition")
    if PROJECT_NAME not in data:
        raise ProvisionError(f"The firmware does not contain {PROJECT_NAME.decode()}")
    for name, marker in BOARD_MARKERS.items():
        if (marker in data) != (name == board):
            raise ProvisionError(f"The firmware is not a build for the {board} board")
    return data


def require_tools() -> None:
    for tool in ("esptool", "espefuse", "git"):
        if shutil.which(tool) is None:
            raise ProvisionError(f"{tool} was not found on PATH")
