"""The ZeroMOUSE profile."""

import argparse
from typing import Any

from esphome_provision.api import SessionFactory
from esphome_provision.common import ProvisionError
from esphome_provision.flasher import ChipInfo, Preflight
from esphome_provision.profiles import ArgumentAdder
from esphome_provision.profiles.zeromouse import commands, steps
from esphome_provision.profiles.zeromouse.efuse import (
    USER_DATA,
    board_id,
    check_board,
)
from esphome_provision.profiles.zeromouse.nvs import identity_from_image, parse_nvs
from esphome_provision.profiles.zeromouse.settings import stock_settings

CHIP = "ESP32-S3"
FLASH_SIZE = 16 * 1024 * 1024
BOARDS = ["v1", "v2"]
# A driver tag that only the build for each board contains.
BOARD_MARKERS = {"v1": b"pt2046b", "v2": b"sths34pf80"}


def check_s3_arguments(args: argparse.Namespace) -> None:
    if bool(args.s3_endpoint) != bool(args.s3_bucket):
        raise ProvisionError("Give both --s3-endpoint and --s3-bucket or neither")


class ZeroMouseProfile:
    name: str = "zeromouse"
    project_name: bytes = b"siku2.zeromouse"
    efuse_names: tuple[str, ...] = (USER_DATA,)

    def add_install_arguments(self, parser: ArgumentAdder) -> None:
        _ = parser.add_argument(
            "--board",
            choices=BOARDS,
            help="Required to install, checked against the eFuses and the firmware",
        )
        _ = parser.add_argument("--device", help="Secret name prefix, for example kopi")
        _ = parser.add_argument(
            "--observe-only",
            action="store_true",
            help="Turn blocking and accepting cloud settings off before the cloud starts",
        )
        _ = parser.add_argument("--s3-endpoint")
        _ = parser.add_argument("--s3-bucket")

    def resume_arguments(self, args: argparse.Namespace) -> list[str]:
        result: list[str] = []
        for option, value in (
            ("--device", args.device),
            ("--s3-endpoint", args.s3_endpoint),
            ("--s3-bucket", args.s3_bucket),
        ):
            if value:
                result += [option, value]
        if args.observe_only:
            result.append("--observe-only")
        return result

    def check_firmware(self, data: bytes, args: argparse.Namespace) -> None:
        check_s3_arguments(args)
        if args.board is None:
            raise ProvisionError("The zeromouse profile needs --board v1 or v2")
        for board, marker in BOARD_MARKERS.items():
            if (marker in data) != (board == args.board):
                raise ProvisionError(
                    f"The firmware is not a build for the {args.board} board"
                )

    def check_chip(self, chip: ChipInfo) -> None:
        if chip.chip.label != CHIP:
            raise ProvisionError(f"The chip is an {chip.chip.label}, not an {CHIP}")
        if chip.flash_size != FLASH_SIZE:
            raise ProvisionError(
                f"The flash size is {chip.flash_size >> 20} MB, expected 16 MB"
            )

    def check_device(self, pre: Preflight, args: argparse.Namespace) -> None:
        found = board_id(pre.extra_efuses[USER_DATA])
        if args.board is not None:
            check_board(found, args.board)
        print(f"  board id {found}")

    def prepare(
        self, image: bytes, args: argparse.Namespace, secrets: dict[str, str]
    ) -> steps.Context:
        check_s3_arguments(args)
        identity, table, nvs = identity_from_image(image)
        print(
            f"Stock firmware found: partition table {table:#x}, nvs {nvs.offset:#x}, "
            + f"thing name {identity.thing_name}, event count {identity.event_count}, boot count {identity.boot_count}"
        )
        s3 = steps.s3_values(secrets, args.device) if args.device else None
        content = parse_nvs(image[nvs.offset : nvs.offset + nvs.size])
        skip = {"block_enabled"} if args.observe_only else set()
        settings = stock_settings(content, skip=skip)
        print(f"  {len(settings)} stock settings to carry over")
        return steps.Context(identity, s3, settings)

    async def provision(
        self, factory: SessionFactory, context: Any, args: argparse.Namespace
    ) -> None:
        await steps.provision(factory, context, args)

    def add_subcommands(
        self, subparsers: "argparse._SubParsersAction[argparse.ArgumentParser]"
    ) -> None:
        commands.add_subcommands(subparsers)
