"""Remember the Home Assistant login between runs."""

import json
import os
from dataclasses import dataclass
from pathlib import Path

from esphome_provision.common import ProvisionError


@dataclass(frozen=True)
class StoredLogin:
    client_id: str
    refresh_token: str


def normalize_url(url: str) -> str:
    return url.strip().rstrip("/")


def store_path() -> Path:
    base = os.environ.get("XDG_STATE_HOME")
    root = Path(base) if base else Path.home() / ".local" / "state"
    return root / "esphome-provision" / "ha.json"


def read_all(path: Path) -> dict[str, dict[str, str]]:
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as error:
        raise ProvisionError(f"Cannot read the login store {path}") from error
    if not isinstance(data, dict):
        raise ProvisionError(f"The login store {path} is not valid")
    return data


def write_all(path: Path, data: dict[str, dict[str, str]]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    path.parent.chmod(0o700)
    temporary = path.with_suffix(".tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, "w") as handle:
        _ = handle.write(json.dumps(data, indent=2))
    temporary.chmod(0o600)
    _ = temporary.replace(path)


def load(url: str) -> StoredLogin | None:
    entry = read_all(store_path()).get(normalize_url(url))
    if not entry or not entry.get("refresh_token") or not entry.get("client_id"):
        return None
    return StoredLogin(entry["client_id"], entry["refresh_token"])


def save(url: str, login: StoredLogin) -> None:
    path = store_path()
    data = read_all(path)
    data[normalize_url(url)] = {
        "client_id": login.client_id,
        "refresh_token": login.refresh_token,
    }
    write_all(path, data)


def delete(url: str) -> bool:
    path = store_path()
    data = read_all(path)
    if data.pop(normalize_url(url), None) is None:
        return False
    write_all(path, data)
    return True
