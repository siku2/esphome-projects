"""Shared error type and text helpers."""

import re

ANSI = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
KEY_ENV = "ESPHOME_API_KEY"
MAC = re.compile(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", re.IGNORECASE)


class ProvisionError(Exception):
    """A step failed. The message is shown to the user as is."""


def strip_ansi(text: str) -> str:
    return ANSI.sub("", text)
