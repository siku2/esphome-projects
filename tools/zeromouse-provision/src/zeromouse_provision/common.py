"""Shared error type and text helpers for the ZeroMOUSE installer."""

import re

ANSI = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")


class ProvisionError(Exception):
    """A step failed. The message is shown to the user as is."""


def strip_ansi(text: str) -> str:
    return ANSI.sub("", text)
