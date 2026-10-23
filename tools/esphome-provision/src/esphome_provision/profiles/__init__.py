"""Device profiles. A profile adds project specific checks and provisioning steps."""

import argparse
from typing import Any, Protocol

from esphome_provision.api import SessionFactory
from esphome_provision.common import ProvisionError
from esphome_provision.flasher import ChipInfo, Preflight


class ArgumentAdder(Protocol):
    def add_argument(self, *name_or_flags: str, **kwargs: Any) -> argparse.Action: ...


class Profile(Protocol):
    name: str
    project_name: bytes
    efuse_names: tuple[str, ...]

    def add_install_arguments(self, parser: ArgumentAdder) -> None:
        """Add the profile options to the install and backup commands."""

    def resume_arguments(self, args: argparse.Namespace) -> list[str]:
        """Return the profile options of args as a command line for install --resume."""

    def check_firmware(self, data: bytes, args: argparse.Namespace) -> None:
        """Refuse a firmware that does not fit the profile and the options."""

    def check_chip(self, chip: ChipInfo) -> None:
        """Refuse a chip or flash size the profile does not support."""

    def check_device(self, pre: Preflight, args: argparse.Namespace) -> None:
        """Refuse a device whose eFuses do not fit the options."""

    def prepare(
        self, image: bytes, args: argparse.Namespace, secrets: dict[str, str]
    ) -> Any:
        """Read what the provisioning needs from the backup image. May refuse."""

    async def provision(
        self, factory: SessionFactory, context: Any, args: argparse.Namespace
    ) -> None:
        """Run the API steps once the device is on Wi-Fi."""

    def add_subcommands(
        self, subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]"
    ) -> None:
        """Add the profile commands."""


def all_profiles() -> dict[str, Profile]:
    from esphome_provision.profiles.zeromouse import PROFILE as zeromouse

    return {zeromouse.name: zeromouse}


def detect_profile(firmware: bytes) -> Profile | None:
    found = [p for p in all_profiles().values() if p.project_name in firmware]
    if len(found) > 1:
        names = ", ".join(p.name for p in found)
        raise ProvisionError(
            f"The firmware matches several profiles ({names}), pass --profile"
        )
    return found[0] if found else None
