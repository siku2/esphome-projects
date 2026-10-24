"""Log in to Home Assistant with its OAuth2 authorization code flow."""

import asyncio
import os
import secrets
import webbrowser
from collections.abc import Mapping
from typing import Any
from urllib.parse import urlencode

import aiohttp
from aiohttp import web

from esphome_provision import ha_store
from esphome_provision.common import ProvisionError
from esphome_provision.ha_store import StoredLogin

TOKEN_ENV = "ESPHOME_PROVISION_HA_TOKEN"
LOGIN_TIMEOUT = 300
REQUEST_TIMEOUT = aiohttp.ClientTimeout(total=30)


def authorize_url(url: str, client_id: str, redirect_uri: str, state: str) -> str:
    query = urlencode(
        {
            "response_type": "code",
            "client_id": client_id,
            "redirect_uri": redirect_uri,
            "state": state,
        }
    )
    return f"{ha_store.normalize_url(url)}/auth/authorize?{query}"


def parse_callback(query: Mapping[str, str], state: str) -> str:
    """Return the authorization code from the redirect, or raise."""
    if error := query.get("error"):
        raise ProvisionError(f"Home Assistant refused the login: {error}")
    if query.get("state") != state:
        raise ProvisionError("The login answer has the wrong state, try again")
    code = query.get("code")
    if not code:
        raise ProvisionError("The login answer has no code")
    return code


async def post_form(
    http: aiohttp.ClientSession, url: str, path: str, data: dict[str, str]
) -> tuple[int, dict[str, Any]]:
    try:
        async with http.post(
            f"{ha_store.normalize_url(url)}{path}", data=data, timeout=REQUEST_TIMEOUT
        ) as response:
            try:
                body = await response.json(content_type=None)
            except ValueError:
                body = {}
            return response.status, body if isinstance(body, dict) else {}
    except (aiohttp.ClientError, TimeoutError) as error:
        raise ProvisionError(
            f"Could not reach Home Assistant at {url}: {type(error).__name__}"
        ) from error


async def wait_for_code(url: str) -> tuple[str, str]:
    """Open the browser login and return the code and the client id that was used."""
    state = secrets.token_urlsafe(16)
    result: asyncio.Future[str] = asyncio.get_running_loop().create_future()

    async def callback(request: web.Request) -> web.Response:
        try:
            code = parse_callback(request.query, state)
        except ProvisionError as error:
            if not result.done():
                result.set_exception(error)
            return web.Response(status=400, text=f"Login failed: {error}")
        if not result.done():
            result.set_result(code)
        return web.Response(text="Login complete. You can close this tab.")

    app = web.Application()
    _ = app.router.add_get("/callback", callback)
    runner = web.AppRunner(app)
    await runner.setup()
    try:
        _ = await web.TCPSite(runner, "127.0.0.1", 0).start()
        port = runner.addresses[0][1]
        client_id = f"http://127.0.0.1:{port}/"
        link = authorize_url(url, client_id, f"{client_id}callback", state)
        print(f"Log in to Home Assistant in the browser:\n  {link}", flush=True)
        _ = webbrowser.open(link)
        try:
            code = await asyncio.wait_for(result, LOGIN_TIMEOUT)
        except TimeoutError:
            raise ProvisionError(
                f"No login within {LOGIN_TIMEOUT} s, run the command again"
            ) from None
        return code, client_id
    finally:
        await runner.cleanup()


def token_from(status: int, body: dict[str, Any], what: str) -> str:
    token = body.get("access_token")
    if status != 200 or not isinstance(token, str):
        raise ProvisionError(
            f"Home Assistant did not accept the {what} (HTTP {status})"
        )
    return token


async def login(http: aiohttp.ClientSession, url: str) -> str:
    """Log in through the browser, store the refresh token and return an access token."""
    code, client_id = await wait_for_code(url)
    status, body = await post_form(
        http,
        url,
        "/auth/token",
        {"grant_type": "authorization_code", "code": code, "client_id": client_id},
    )
    access = token_from(status, body, "login code")
    refresh_token = body.get("refresh_token")
    if not isinstance(refresh_token, str):
        raise ProvisionError("Home Assistant gave no refresh token")
    ha_store.save(url, StoredLogin(client_id, refresh_token))
    print(f"Logged in to {url}")
    return access


async def refresh(
    http: aiohttp.ClientSession, url: str, stored: StoredLogin
) -> str | None:
    """Return a new access token, or None if the refresh token is no longer valid."""
    status, body = await post_form(
        http,
        url,
        "/auth/token",
        {
            "grant_type": "refresh_token",
            "refresh_token": stored.refresh_token,
            "client_id": stored.client_id,
        },
    )
    if status == 400:
        return None
    return token_from(status, body, "stored login")


async def access_token(http: aiohttp.ClientSession, url: str) -> str:
    if token := os.environ.get(TOKEN_ENV):
        return token
    stored = ha_store.load(url)
    if stored:
        token = await refresh(http, url, stored)
        if token:
            return token
        print("The stored Home Assistant login expired, logging in again")
    return await login(http, url)


async def logout(http: aiohttp.ClientSession, url: str) -> None:
    stored = ha_store.load(url)
    if stored is None:
        print(f"No stored login for {url}")
        return
    try:
        _ = await post_form(http, url, "/auth/revoke", {"token": stored.refresh_token})
    except ProvisionError as error:
        print(f"Could not revoke the token, removing it locally only: {error}")
    _ = ha_store.delete(url)
    print(f"Logged out of {url}")
