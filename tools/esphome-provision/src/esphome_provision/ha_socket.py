"""The Home Assistant websocket calls that find an ESPHome encryption key."""

from typing import Any

import aiohttp

from esphome_provision import ha_store
from esphome_provision.common import ProvisionError
from esphome_provision.ha_auth import TOKEN_ENV


def websocket_url(url: str) -> str:
    base = ha_store.normalize_url(url)
    scheme, _, rest = base.partition("://")
    if scheme not in ("http", "https") or not rest:
        raise ProvisionError(f"{url} is not an http or https address")
    return f"{'wss' if scheme == 'https' else 'ws'}://{rest}/api/websocket"


class Connection:
    def __init__(self, socket: aiohttp.ClientWebSocketResponse) -> None:
        self.socket = socket
        self.counter = 0

    async def call(self, kind: str, **fields: Any) -> Any:
        self.counter += 1
        await self.socket.send_json({"id": self.counter, "type": kind, **fields})
        async for message in self.socket:
            if message.type != aiohttp.WSMsgType.TEXT:
                break
            answer = message.json()
            if answer.get("id") != self.counter:
                continue
            if not answer.get("success"):
                code = (answer.get("error") or {}).get("code", "unknown")
                raise ProvisionError(f"Home Assistant rejected {kind}: {code}")
            return answer.get("result")
        raise ProvisionError("Home Assistant closed the connection")


async def connect(
    http: aiohttp.ClientSession, url: str, token: str
) -> aiohttp.ClientWebSocketResponse:
    try:
        socket = await http.ws_connect(websocket_url(url), heartbeat=30)
        _ = await socket.receive_json()
        await socket.send_json({"type": "auth", "access_token": token})
        answer = await socket.receive_json()
    except (aiohttp.ClientError, TimeoutError, ValueError) as error:
        raise ProvisionError(
            f"Could not open the websocket to {url}: {type(error).__name__}"
        ) from error
    if answer.get("type") != "auth_ok":
        await socket.close()
        raise ProvisionError(
            "Home Assistant rejected the access token. "
            + f"Run ha login again or check {TOKEN_ENV}"
        )
    return socket


async def esphome_entries(connection: Connection) -> list[dict[str, Any]]:
    return await connection.call("config_entries/get", domain="esphome")


async def devices(connection: Connection) -> list[dict[str, Any]]:
    return await connection.call("config/device_registry/list")


async def encryption_key(connection: Connection, entry_id: str) -> str | None:
    result = await connection.call("esphome/get_encryption_key", entry_id=entry_id)
    return (result or {}).get("encryption_key")


DATA_PATHS = (
    ("data", "config", "data"),
    ("data", "data"),
    ("data", "entry", "data"),
    ("config", "data"),
)


def entry_data(diagnostics: dict[str, Any]) -> dict[str, Any]:
    """Return the config entry data from the diagnostics, or an empty dict."""
    for path in DATA_PATHS:
        node: Any = diagnostics
        for part in path:
            node = node.get(part) if isinstance(node, dict) else None
        if isinstance(node, dict) and "device_name" in node:
            return node
    return {}


async def diagnostics(
    http: aiohttp.ClientSession, url: str, token: str, entry_id: str
) -> dict[str, Any]:
    """Return the entry data of one config entry. Entries without diagnostics give an empty dict."""
    address = f"{ha_store.normalize_url(url)}/api/diagnostics/config_entry/{entry_id}"
    try:
        async with http.get(
            address, headers={"Authorization": f"Bearer {token}"}
        ) as response:
            if response.status != 200:
                return {}
            body = await response.json(content_type=None)
    except (aiohttp.ClientError, TimeoutError, ValueError):
        return {}
    return entry_data(body) if isinstance(body, dict) else {}
