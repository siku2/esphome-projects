import asyncio

import pytest
from aioesphomeapi import InvalidEncryptionKeyAPIError
from aioesphomeapi.core import APIConnectionError

from esphome_provision import api
from esphome_provision.common import ProvisionError


def test_zero_psk_is_32_zero_bytes_in_base64() -> None:
    assert api.ZERO_PSK == "A" * 43 + "="


def test_key_message_for_the_default_key_names_the_options() -> None:
    message = api.invalid_key_message("10.0.0.1", api.ZERO_PSK)
    assert "Home Assistant" in message
    assert "--api-key" in message
    assert "install" in message
    assert ";" not in message


def test_key_message_for_a_given_key() -> None:
    assert "rejects the API key" in api.invalid_key_message("10.0.0.1", "abc")


class RejectingClient:
    def __init__(self, *args: object, **kwargs: object) -> None:
        del args, kwargs

    async def connect(self, **kwargs: object) -> None:
        del kwargs
        raise InvalidEncryptionKeyAPIError("bad key")


def test_invalid_key_becomes_a_provision_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(api, "APIClient", RejectingClient)
    session = api.Session("10.0.0.1", api.ZERO_PSK)
    with pytest.raises(ProvisionError, match="Home Assistant"):
        asyncio.run(session.connect(5))


def test_retry_reconnects_once_after_a_drop(monkeypatch: pytest.MonkeyPatch) -> None:
    async def no_sleep(delay: float) -> None:
        del delay

    monkeypatch.setattr(asyncio, "sleep", no_sleep)
    calls: list[int] = []

    async def operation() -> str:
        calls.append(1)
        if len(calls) == 1:
            raise APIConnectionError("Connection closed")
        return "ok"

    assert asyncio.run(api.retry_dropped(operation)) == "ok"
    assert len(calls) == 2


def test_retry_reports_the_key_error_of_the_reconnect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async def no_sleep(delay: float) -> None:
        del delay

    monkeypatch.setattr(asyncio, "sleep", no_sleep)
    calls: list[int] = []

    async def operation() -> None:
        calls.append(1)
        if len(calls) == 1:
            raise api.ConnectionDropped("dropped")
        raise ProvisionError(api.invalid_key_message("10.0.0.1", api.ZERO_PSK))

    with pytest.raises(ProvisionError, match="Home Assistant"):
        asyncio.run(api.retry_dropped(operation))


def test_retry_gives_up_after_two_drops(monkeypatch: pytest.MonkeyPatch) -> None:
    async def no_sleep(delay: float) -> None:
        del delay

    monkeypatch.setattr(asyncio, "sleep", no_sleep)

    async def operation() -> None:
        raise api.ConnectionDropped("dropped")

    with pytest.raises(ProvisionError, match="twice"):
        asyncio.run(api.retry_dropped(operation))
