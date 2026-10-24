"""Query the SurePetCare cloud and download hub firmware.

The hub fetches its firmware from hub.api.surehub.io over plain HTTP with just
its serial number. The account API at app.api.surehub.io returns that serial,
which the web UI hides. Credentials are read interactively and never stored.

Usage:
  python3 tools/sureflap/surepet.py devices
  python3 tools/sureflap/surepet.py firmware --serial H0xx-0xxxxxx --out ./fw
"""

import argparse
import getpass
import json
import sys
import urllib.error
import urllib.parse
import urllib.request

APP_API = "https://app.api.surehub.io"
HUB_API = "http://hub.api.surehub.io"
# Any stable client identifier works. The server only ties the token to it.
CLIENT_DEVICE_ID = "1050547"
USER_AGENT = "surepet-tool/1.0"
MAX_PAGES = 256


def _request(url, data=None, token=None, form=False):
    headers = {"User-Agent": USER_AGENT, "Accept": "application/json"}
    body = None
    if data is not None:
        if form:
            body = urllib.parse.urlencode(data).encode()
            headers["Content-Type"] = "application/x-www-form-urlencoded"
        else:
            body = json.dumps(data).encode()
            headers["Content-Type"] = "application/json"
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, data=body, headers=headers)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read()


def login(email, password):
    payload = {
        "email_address": email,
        "password": password,
        "device_id": CLIENT_DEVICE_ID,
    }
    raw = _request(f"{APP_API}/api/auth/login", data=payload)
    return json.loads(raw)["data"]["token"]


def list_devices(token):
    raw = _request(f"{APP_API}/api/me/start", token=token)
    data = json.loads(raw).get("data", {})
    devices = data.get("devices", [])
    if not devices:
        print("no devices in the response", file=sys.stderr)
        return
    for d in devices:
        version = (
            d.get("status", {})
            .get("version", {})
            .get("device", {})
            .get("firmware", "?")
        )
        print(
            f"id={d.get('id')} product={d.get('product_id')} serial={d.get('serial_number')} firmware={version}"
        )


def download_firmware(serial, bootloader, outdir):
    import os

    os.makedirs(outdir, exist_ok=True)
    for page in range(MAX_PAGES):
        payload = {
            "serial_number": serial,
            "page": str(page),
            "bootloader_version": bootloader,
        }
        raw = _request(f"{HUB_API}/api/firmware", data=payload, form=True)
        if not raw:
            print(f"page {page} empty, stopping ({page} pages downloaded)")
            return
        path = os.path.join(outdir, f"{serial}-{bootloader}-{page:02d}.bin")
        with open(path, "wb") as f:
            f.write(raw)
        print(f"page {page:3d}: {len(raw)} bytes -> {path}")
    print(f"reached the page cap of {MAX_PAGES}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("devices", help="log in and list devices with their serials")
    fw = sub.add_parser("firmware", help="download all firmware pages for a serial")
    fw.add_argument("--serial", required=True)
    fw.add_argument("--bootloader", default="1.177")
    fw.add_argument("--out", default="./firmware")
    args = parser.parse_args()

    if args.command == "devices":
        email = input("SurePetCare email: ")
        password = getpass.getpass("SurePetCare password: ")
        try:
            token = login(email, password)
        except urllib.error.HTTPError as e:
            print(
                f"login failed: {e.code} {e.read().decode(errors='replace')}",
                file=sys.stderr,
            )
            sys.exit(1)
        list_devices(token)
    elif args.command == "firmware":
        download_firmware(args.serial, args.bootloader, args.out)


if __name__ == "__main__":
    main()
