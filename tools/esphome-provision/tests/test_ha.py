import asyncio
import os
import stat
from pathlib import Path
from typing import Any

import pytest

from esphome_provision import ha, ha_auth, ha_socket, ha_store
from esphome_provision.common import KEY_ENV, ProvisionError
from esphome_provision.ha_store import StoredLogin

KOPI_MAC = "aa:bb:cc:00:00:01"
URL = "https://ha.example"


ENTRIES = [{"entry_id": "e-kopi"}, {"entry_id": "e-arusha"}]


def device(mac: str | None, *entry_ids: str) -> dict[str, Any]:
    connections = [["mac", mac]] if mac else []
    return {"connections": connections, "config_entries": list(entry_ids)}


DEVICES = [
    device(KOPI_MAC, "e-kopi", "e-other"),
    device(None, "e-kopi"),
    device("aa:bb:cc:00:00:02", "e-arusha"),
]


def test_pick_entry_by_mac_ignores_case() -> None:
    assert ha.pick_entry(ENTRIES, DEVICES, KOPI_MAC.upper()) == "e-kopi"


def test_pick_entry_none() -> None:
    with pytest.raises(ProvisionError, match="no ESPHome device with MAC"):
        _ = ha.pick_entry(ENTRIES, DEVICES, "aa:bb:cc:00:00:09")


def test_pick_entry_several() -> None:
    devices = [*DEVICES, device(KOPI_MAC, "e-arusha")]
    with pytest.raises(ProvisionError, match="2 ESPHome entries"):
        _ = ha.pick_entry(ENTRIES, devices, KOPI_MAC)


def test_pick_entry_ignores_sub_device_without_mac() -> None:
    assert (
        ha.pick_entry(
            ENTRIES, [device(None, "e-kopi")] + DEVICES[2:], "aa:bb:cc:00:00:02"
        )
        == "e-arusha"
    )
    with pytest.raises(ProvisionError):
        _ = ha.pick_entry(ENTRIES, [device(None, "e-kopi")], KOPI_MAC)


def test_pick_entry_ignores_non_esphome_entry() -> None:
    with pytest.raises(ProvisionError):
        _ = ha.pick_entry(ENTRIES, [device(KOPI_MAC, "e-other")], KOPI_MAC)


NAMED = [
    ("e1", {"device_name": "bean-kopi", "host": "10.0.0.1"}),
    ("e2", {"device_name": "bean-arusha", "host": "10.0.0.2"}),
    ("e3", {}),
]


def test_pick_by_name_found() -> None:
    assert ha.pick_by_name(NAMED, "Bean-Kopi") == ("e1", "10.0.0.1")


def test_pick_by_name_none() -> None:
    with pytest.raises(ProvisionError, match="no ESPHome device named"):
        _ = ha.pick_by_name(NAMED, "bean-x")


def test_pick_by_name_several() -> None:
    with pytest.raises(ProvisionError, match="2 ESPHome devices named"):
        _ = ha.pick_by_name([*NAMED, ("e4", NAMED[0][1])], "bean-kopi")


def test_pick_by_name_without_host() -> None:
    with pytest.raises(ProvisionError, match="pass --host"):
        _ = ha.pick_by_name([("e", {"device_name": "n"})], "n")


def test_entry_data_paths() -> None:
    data = {"device_name": "n", "host": "h"}
    assert ha_socket.entry_data({"data": {"config": {"data": data}}}) == data
    assert ha_socket.entry_data({"data": {"data": data}}) == data
    assert ha_socket.entry_data({"data": "x"}) == {}


def test_key_order(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str | None, str | None]] = []

    async def lookup(
        url: str, mac: str | None, host: str | None, name: str | None
    ) -> tuple[str, str | None]:
        del name
        calls.append((url, mac, host))
        return "from-ha", None

    monkeypatch.setattr(ha, "lookup_key", lookup)
    monkeypatch.setenv(KEY_ENV, "from-env")
    assert ha.resolve_credentials("opt", URL) == ha.Credentials("opt", "--api-key")
    assert ha.resolve_credentials(None, URL) == ha.Credentials("from-env", KEY_ENV)
    assert calls == []
    monkeypatch.delenv(KEY_ENV)
    found = ha.resolve_credentials(None, URL, mac=KOPI_MAC)
    assert found == ha.Credentials("from-ha", ha.FROM_HA)
    assert calls == [(URL, KOPI_MAC, None)]
    assert ha.resolve_credentials(None, None, mac=KOPI_MAC) is None


def test_ha_url_from_option_or_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ha.URL_ENV, "env-url")
    assert ha.ha_url("option-url") == "option-url"
    assert ha.ha_url(None) == "env-url"
    monkeypatch.delenv(ha.URL_ENV)
    assert ha.ha_url(None) is None


@pytest.fixture
def state(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path))
    monkeypatch.delenv(ha_auth.TOKEN_ENV, raising=False)
    return tmp_path / "esphome-provision" / "ha.json"


def mode(path: Path) -> int:
    return stat.S_IMODE(os.stat(path).st_mode)


def test_store_round_trip_and_modes(state: Path) -> None:
    assert ha_store.load(URL) is None
    ha_store.save(URL + "/", StoredLogin("client", "refresh"))
    assert ha_store.load(URL) == StoredLogin("client", "refresh")
    assert mode(state) == 0o600
    assert mode(state.parent) == 0o700


def test_store_keeps_other_servers_and_deletes(state: Path) -> None:
    ha_store.save(URL, StoredLogin("c1", "r1"))
    ha_store.save("http://other", StoredLogin("c2", "r2"))
    assert ha_store.delete(URL)
    assert not ha_store.delete(URL)
    assert ha_store.load(URL) is None
    assert ha_store.load("http://other") == StoredLogin("c2", "r2")
    assert mode(state) == 0o600


def test_store_path_falls_back_to_home(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.delenv("XDG_STATE_HOME", raising=False)
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    assert ha_store.store_path() == tmp_path / ".local/state/esphome-provision/ha.json"


def test_authorize_url() -> None:
    link = ha_auth.authorize_url(
        URL + "/", "http://127.0.0.1:5000/", "http://127.0.0.1:5000/callback", "s"
    )
    assert link.startswith(f"{URL}/auth/authorize?response_type=code&")
    assert "client_id=http%3A%2F%2F127.0.0.1%3A5000%2F" in link
    assert "redirect_uri=http%3A%2F%2F127.0.0.1%3A5000%2Fcallback" in link
    assert link.endswith("&state=s")


def test_parse_callback() -> None:
    assert ha_auth.parse_callback({"code": "c", "state": "s"}, "s") == "c"
    with pytest.raises(ProvisionError, match="wrong state"):
        _ = ha_auth.parse_callback({"code": "c", "state": "x"}, "s")
    with pytest.raises(ProvisionError, match="no code"):
        _ = ha_auth.parse_callback({"state": "s"}, "s")
    with pytest.raises(ProvisionError, match="refused"):
        _ = ha_auth.parse_callback({"error": "access_denied", "state": "s"}, "s")


def test_websocket_url() -> None:
    assert (
        ha_socket.websocket_url("https://ha.example/")
        == "wss://ha.example/api/websocket"
    )
    assert (
        ha_socket.websocket_url("http://10.0.0.1:8123")
        == "ws://10.0.0.1:8123/api/websocket"
    )
    with pytest.raises(ProvisionError):
        _ = ha_socket.websocket_url("ha.example")


class FakeHttp:
    pass


def fake_post(
    monkeypatch: pytest.MonkeyPatch, answers: list[tuple[int, dict[str, Any]]]
) -> list[dict[str, str]]:
    sent: list[dict[str, str]] = []

    async def post(
        http: object, url: str, path: str, data: dict[str, str]
    ) -> tuple[int, dict[str, Any]]:
        del http, url, path
        sent.append(data)
        return answers.pop(0)

    monkeypatch.setattr(ha_auth, "post_form", post)
    return sent


def get_token(http: object = None) -> str:
    return asyncio.run(ha_auth.access_token(http, URL))  # pyright: ignore[reportArgumentType]


def test_environment_token_skips_everything(
    state: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv(ha_auth.TOKEN_ENV, "long-lived")
    assert get_token() == "long-lived"
    assert not state.exists()


def test_stored_login_is_refreshed(
    state: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ha_store.save(URL, StoredLogin("client", "refresh"))
    sent = fake_post(monkeypatch, [(200, {"access_token": "fresh"})])
    assert get_token() == "fresh"
    assert sent == [
        {
            "grant_type": "refresh_token",
            "refresh_token": "refresh",
            "client_id": "client",
        }
    ]


def test_failed_refresh_logs_in_again(
    state: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ha_store.save(URL, StoredLogin("old-client", "old-refresh"))
    sent = fake_post(
        monkeypatch,
        [
            (400, {"error": "invalid_grant"}),
            (200, {"access_token": "a", "refresh_token": "new"}),
        ],
    )

    async def wait_for_code(url: str) -> tuple[str, str]:
        del url
        return "the-code", "new-client"

    monkeypatch.setattr(ha_auth, "wait_for_code", wait_for_code)
    assert get_token() == "a"
    assert sent[1] == {
        "grant_type": "authorization_code",
        "code": "the-code",
        "client_id": "new-client",
    }
    assert ha_store.load(URL) == StoredLogin("new-client", "new")


def test_refresh_server_error_is_not_a_new_login(
    state: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ha_store.save(URL, StoredLogin("client", "refresh"))
    _ = fake_post(monkeypatch, [(500, {})])
    with pytest.raises(ProvisionError, match="HTTP 500") as info:
        _ = get_token()
    assert "refresh" not in str(info.value)
