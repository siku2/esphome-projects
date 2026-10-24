"""Chip checks, backups and flashing through the esptool CLIs."""

import hashlib
import json
import os
import re
import shutil
import subprocess
import threading
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import UTC, date, datetime
from pathlib import Path

from esphome_provision import secrets
from esphome_provision.common import ProvisionError, strip_ansi
from esphome_provision.efuse import (
    EfuseValue,
    check_safe_to_flash,
    parse_summary_json,
    parse_summary_text,
    safety_info,
    safety_names,
)
from esphome_provision.firmware import Chip, chip_by_esptool_id, chip_by_label

TOOL = "esphome_provision 1"
SHORT_TIMEOUT = 120
LONG_TIMEOUT = 1800
KEEP = ["--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]
FLASH_SIZE_UNITS = {"KB": 1024, "MB": 1024 * 1024}


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
    chip: str | None,
    what: str,
    timeout: float = SHORT_TIMEOUT,
    after: str = "no-reset",
    show: bool = False,
) -> str:
    """Run esptool. Without a chip name, esptool detects the chip itself."""
    chip_args = ["--chip", chip] if chip else []
    cmd = ["esptool", *chip_args, "--port", port, "--after", after, *args]
    return run(cmd, what=what, timeout=timeout, show=show)


@dataclass(frozen=True)
class ChipInfo:
    chip: Chip
    mac: str
    flash_size: int


def parse_flash_size(text: str) -> int:
    match = re.fullmatch(r"(\d+)(KB|MB)", text)
    if match is None:
        raise ProvisionError(f"Cannot read the flash size {text!r} from esptool")
    return int(match.group(1)) * FLASH_SIZE_UNITS[match.group(2)]


def parse_flash_id(output: str) -> ChipInfo:
    line = re.search(r"^Chip type:\s+(.+)$", output, re.MULTILINE)
    if line is None:
        raise ProvisionError("Could not read the chip type from esptool")
    if "Secure Download" in line.group(1):
        raise ProvisionError("The chip is not in normal download mode")
    name = re.match(r"ESP32(?:-(?:S2|S3|C2|C3|C6|H2))?\b", line.group(1))
    chip = chip_by_label(name.group(0)) if name else None
    if chip is None:
        raise ProvisionError(f"The chip {line.group(1)!r} is not supported")
    mac = re.search(
        r"^MAC:\s+((?:[0-9a-f]{2}:){5}[0-9a-f]{2})\s*$", output, re.MULTILINE
    )
    if mac is None:
        raise ProvisionError("Could not read the MAC address from esptool")
    size = re.search(r"^Detected flash size:\s+(\S+)\s*$", output, re.MULTILINE)
    if size is None:
        raise ProvisionError("Could not read the flash size from esptool")
    return ChipInfo(chip, mac.group(1), parse_flash_size(size.group(1)))


def detect_chip(port: str) -> ChipInfo:
    return parse_flash_id(
        esptool(port, "flash-id", chip=None, what="Reading the chip and flash id")
    )


@dataclass(frozen=True)
class Preflight:
    chip: ChipInfo
    efuse_text: str
    extra_efuses: dict[str, str]


def read_efuses(
    port: str, chip: ChipInfo, extra: Sequence[str]
) -> tuple[dict[str, EfuseValue], str]:
    safety = safety_names(chip.chip.esptool_id, chip.chip.label)
    names = [*safety, *extra]
    base = ["espefuse", "--chip", chip.chip.esptool_id, "--port", port]
    as_json = run(
        [*base, "summary", "--format", "json", *names],
        what="Reading eFuses",
        timeout=SHORT_TIMEOUT,
    )
    as_text = run(
        [*base, "summary"], what="Reading the eFuse summary", timeout=SHORT_TIMEOUT
    )
    values = parse_summary_json(as_json, names)
    if parse_summary_text(as_text, names) != values:
        raise ProvisionError("The JSON and text eFuse summaries disagree")
    check_safe_to_flash(safety_info(values, safety))
    return values, as_text


def preflight(
    port: str,
    *,
    extra_efuses: Sequence[str] = (),
    check_chip: Callable[[ChipInfo], None] | None = None,
) -> Preflight:
    """Check the chip, the flash and the eFuses. The callback may refuse the chip."""
    print(f"Checking the chip on {port}")
    chip = detect_chip(port)
    if check_chip is not None:
        check_chip(chip)
    values, text = read_efuses(port, chip, extra_efuses)
    print(
        f"  {chip.chip.label}, MAC {chip.mac}, {chip.flash_size >> 20} MB flash, "
        + "plain flash, no secure boot"
    )
    extra = {name: str(values[name]) for name in extra_efuses}
    return Preflight(chip, text, extra)


def check_mac(chip: ChipInfo, expected: str, *, allow_other: bool) -> None:
    if chip.mac.lower() != expected.lower():
        if not allow_other:
            raise ProvisionError(
                f"The device MAC {chip.mac} does not match the expected {expected}"
            )
        print(
            f"Warning: the device MAC {chip.mac} differs from the expected {expected}"
        )


def run_git(parent: Path, *args: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            ["git", "-C", str(parent), *args],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise ProvisionError(
            "Cannot run git to check where the backup directory is"
        ) from error


def check_git_placement(path: Path) -> None:
    """Allow a path in a git work tree only if it is ignored or not in our own repository."""
    parent = path
    while not parent.exists():
        parent = parent.parent
    result = run_git(parent, "rev-parse", "--show-toplevel")
    if result.returncode != 0:
        if "not a git repository" in result.stderr:
            return
        raise ProvisionError(
            "git could not tell whether the backup directory is in a work tree"
        )
    top = Path(result.stdout.strip()).resolve()
    ignored = run_git(parent, "check-ignore", "-q", str(path))
    if ignored.returncode == 0:
        return
    if ignored.returncode != 1:
        raise ProvisionError(
            "git could not tell whether the backup directory is ignored"
        )
    if top == secrets.repo_root().resolve():
        raise ProvisionError(
            f"The backup directory is inside the git work tree {top} and is not ignored. "
            + "Backups contain device keys, so add the directory to .gitignore"
        )
    print(
        f"The backup is inside the git work tree {top} and contains device keys. "
        + "Commit it only to a private repository."
    )


def default_backup_dir(name: str, today: date | None = None) -> Path:
    """Return backups/<name>-<date> in the current directory, numbered if it is taken."""
    base = Path("backups") / f"{name}-{(today or date.today()).isoformat()}"
    path = base
    number = 2
    while path.exists() and (not path.is_dir() or any(path.iterdir())):
        path = base.with_name(f"{base.name}-{number}")
        number += 1
    return path


def check_backup_dir(path: Path) -> Path:
    path = path.expanduser().absolute()
    if path.exists():
        if not path.is_dir() or any(path.iterdir()):
            raise ProvisionError(
                f"The backup directory {path} must be empty or not exist"
            )
    check_git_placement(path.resolve())
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


def make_manifest(pre: Preflight, digest: str) -> dict[str, object]:
    return {
        "mac": pre.chip.mac,
        "chip": pre.chip.chip.esptool_id,
        "flash_size": pre.chip.flash_size,
        "efuses": pre.extra_efuses,
        "sha256": digest,
        "created_at": datetime.now(UTC).isoformat(timespec="seconds"),
        "tool": TOOL,
    }


def write_manifest(directory: Path, manifest: dict[str, object]) -> None:
    write_private(
        directory / "manifest.json", (json.dumps(manifest, indent=2) + "\n").encode()
    )


def make_backup(port: str, directory: Path, pre: Preflight) -> Path:
    """Read the whole flash, verify it against the chip and write the manifest."""
    chip = pre.chip.chip.esptool_id
    size = pre.chip.flash_size
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
        hex(size),
        str(part),
        chip=chip,
        what="Reading the flash",
        timeout=LONG_TIMEOUT,
        show=True,
    )
    part.chmod(0o600)
    if part.stat().st_size != size:
        raise ProvisionError("The flash dump has the wrong size")
    print("Verifying the dump against the chip")
    output = esptool(
        port,
        "verify-flash",
        "0",
        str(part),
        chip=chip,
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
    write_manifest(directory, make_manifest(pre, digest))
    print(f"Backup saved in {directory}")
    return image


@dataclass(frozen=True)
class Backup:
    image: Path
    mac: str
    chip: Chip
    flash_size: int
    efuses: dict[str, str]


def load_backup(directory: Path) -> Backup:
    try:
        manifest = json.loads((directory / "manifest.json").read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ProvisionError(f"Cannot read {directory}/manifest.json") from error
    image = directory / "flash.bin"
    if not isinstance(manifest, dict) or not image.is_file():
        raise ProvisionError("The backup directory is incomplete")
    chip = chip_by_esptool_id(manifest.get("chip"))
    size = manifest.get("flash_size")
    if chip is None or not isinstance(size, int) or size <= 0:
        raise ProvisionError("The backup manifest has no valid chip and flash size")
    if image.stat().st_size != size:
        raise ProvisionError("flash.bin does not have the flash size of the manifest")
    if sha256_file(image) != manifest.get("sha256"):
        raise ProvisionError("flash.bin does not match the sha256 in the manifest")
    mac = manifest.get("mac")
    if not isinstance(mac, str):
        raise ProvisionError("The backup manifest has no MAC address")
    efuses = manifest.get("efuses")
    board_id = manifest.get("board_id")
    if isinstance(efuses, dict):
        extra = {str(k): str(v) for k, v in efuses.items()}
    elif isinstance(board_id, int):
        # Manifests of the first tool version only kept the first byte of the user data block.
        extra = {"BLOCK_USR_DATA": f"{board_id:02x}"}
    else:
        raise ProvisionError("The backup manifest has no eFuse record")
    return Backup(image, mac, chip, size, extra)


def write_image(port: str, chip: str, address: int, image: Path, *, what: str) -> None:
    output = esptool(
        port,
        "write-flash",
        *KEEP,
        hex(address),
        str(image),
        chip=chip,
        what=what,
        timeout=LONG_TIMEOUT,
        after="hard-reset",
        show=True,
    )
    if "Hash of data verified" not in output:
        raise ProvisionError(f"{what} was not verified by the chip")


def erase_flash(port: str, chip: str) -> None:
    _ = esptool(
        port,
        "erase-flash",
        chip=chip,
        what="Erasing the flash",
        timeout=LONG_TIMEOUT,
        show=True,
    )


def restore(
    port: str,
    image: Path,
    expected_mac: str,
    *,
    allow_other: bool,
    backup: Backup | None,
) -> None:
    """Write an image back. A backup adds the chip, flash size and eFuse record checks."""

    def check_chip(chip: ChipInfo) -> None:
        if image.stat().st_size != chip.flash_size:
            raise ProvisionError(
                f"The image must be exactly {chip.flash_size >> 20} MiB, the flash size of the device"
            )
        if backup is not None and (
            backup.chip != chip.chip or backup.flash_size != chip.flash_size
        ):
            raise ProvisionError(
                f"The backup is for a {backup.chip.label} with {backup.flash_size >> 20} MiB of flash, "
                + f"the device is an {chip.chip.label} with {chip.flash_size >> 20} MiB"
            )
        check_mac(chip, expected_mac, allow_other=allow_other)

    pre = preflight(
        port,
        extra_efuses=list(backup.efuses) if backup else [],
        check_chip=check_chip,
    )
    if backup is not None:
        for name, stored in backup.efuses.items():
            if not pre.extra_efuses[name].startswith(stored):
                raise ProvisionError(
                    f"The eFuse {name} of this device differs from the backup"
                )
    write_image(port, pre.chip.chip.esptool_id, 0, image, what="Restoring the image")
    print("Restore finished and verified. The device was reset.")


def require_tools() -> None:
    for tool in ("esptool", "espefuse", "git"):
        if shutil.which(tool) is None:
            raise ProvisionError(f"{tool} was not found on PATH")
