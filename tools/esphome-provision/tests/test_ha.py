import json
import subprocess

import pytest

from esphome_provision import ha
from esphome_provision.common import KEY_ENV, ProvisionError

KOPI_MAC = "aa:bb:cc:00:00:01"


def entry(name: str, mac: str, host: str, domain: str = "esphome") -> dict[str, object]:
    return {
        "domain": domain,
        "unique_id": mac,
        "data": {"device_name": name, "host": host, "port": 6053, "noise_psk": "k"},
    }


ENTRIES = [
    entry("bean-kopi", KOPI_MAC, "10.0.0.1"),
    entry("bean-arusha", "aa:bb:cc:00:00:02", "10.0.0.2"),
    entry("bean-kopi", "aa:bb:cc:00:00:03", "10.0.0.3", domain="other"),
]


def names(found: list[dict[str, object]]) -> list[object]:
    return [e["unique_id"] for e in found]


def test_match_by_name() -> None:
    assert names(ha.find_entries(ENTRIES, "bean-kopi", None, None)) == [KOPI_MAC]


def test_match_by_mac_ignores_case() -> None:
    assert names(ha.find_entries(ENTRIES, None, KOPI_MAC.upper(), None)) == [KOPI_MAC]


def test_match_by_host() -> None:
    found = ha.find_entries(ENTRIES, None, None, "10.0.0.2")
    assert names(found) == ["aa:bb:cc:00:00:02"]


def test_no_match() -> None:
    assert ha.find_entries(ENTRIES, "bean-none", None, None) == []
    assert ha.find_entries(ENTRIES, None, None, None) == []


def test_ambiguous_match() -> None:
    found = ha.find_entries(ENTRIES, "bean-kopi", None, "10.0.0.2")
    assert len(found) == 2


def test_remote_script_compiles() -> None:
    script = ha.remote_script("bean-kopi", None, None)
    assert "noise_psk" in script
    compile(script, "remote", "exec")


class Run:
    def __init__(self, stdout: str, code: int = 0, stderr: str = "") -> None:
        self.result = subprocess.CompletedProcess([], code, stdout, stderr)
        self.calls: list[tuple[list[str], str]] = []

    def __call__(
        self, command: list[str], **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        self.calls.append((command, str(kwargs["input"])))
        return self.result


def test_lookup_returns_the_key(monkeypatch: pytest.MonkeyPatch) -> None:
    answer = {"count": 1, "entry": {"host": "h", "noise_psk": "secret"}}
    run = Run(json.dumps(answer))
    monkeypatch.setattr(subprocess, "run", run)
    credentials = ha.lookup("root@ha", name="bean-kopi", mac=None, host=None)
    assert credentials == ha.Credentials("secret", "Home Assistant", "h")
    assert run.calls[0][0] == ["ssh", "-o", "BatchMode=yes", "root@ha", "python3", "-"]


@pytest.mark.parametrize("count", [0, 2])
def test_lookup_names_what_was_searched(
    monkeypatch: pytest.MonkeyPatch, count: int
) -> None:
    monkeypatch.setattr(subprocess, "run", Run(json.dumps({"count": count})))
    with pytest.raises(ProvisionError, match="name bean-kopi, host 10.0.0.1"):
        _ = ha.lookup("root@ha", name="bean-kopi", mac=None, host="10.0.0.1")


def test_lookup_errors_do_not_leak_the_output(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(subprocess, "run", Run("secret-output"))
    with pytest.raises(ProvisionError) as info:
        _ = ha.lookup("root@ha", name="n", mac=None, host=None)
    assert "secret" not in str(info.value)


def test_lookup_reports_ssh_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(subprocess, "run", Run("", 255, "Permission denied"))
    with pytest.raises(ProvisionError, match="Permission denied"):
        _ = ha.lookup("root@ha", name="n", mac=None, host=None)


class FakeLookup:
    def __init__(self) -> None:
        self.calls = 0

    def __call__(self, target: str, **kwargs: str | None) -> ha.Credentials:
        del target, kwargs
        self.calls += 1
        return ha.Credentials("from-ha", ha.FROM_HA, "10.0.0.9")


def test_key_order(monkeypatch: pytest.MonkeyPatch) -> None:
    lookup = FakeLookup()
    monkeypatch.setattr(ha, "lookup", lookup)
    monkeypatch.setenv(KEY_ENV, "from-env")
    first = ha.resolve_credentials("from-option", "root@ha", name="n")
    assert first == ha.Credentials("from-option", "--api-key")
    second = ha.resolve_credentials(None, "root@ha", name="n")
    assert second == ha.Credentials("from-env", KEY_ENV)
    assert lookup.calls == 0
    monkeypatch.delenv(KEY_ENV)
    third = ha.resolve_credentials(None, "root@ha", name="n")
    assert third is not None
    assert third.key == "from-ha"
    assert third.host == "10.0.0.9"
    assert ha.resolve_credentials(None, None, name="n") is None


def test_ssh_target_from_option_or_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ha.SSH_ENV, "env-target")
    assert ha.ssh_target("option-target") == "option-target"
    assert ha.ssh_target(None) == "env-target"
    monkeypatch.delenv(ha.SSH_ENV)
    assert ha.ssh_target(None) is None
