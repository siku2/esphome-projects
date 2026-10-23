"""The ZeroMOUSE board id in the eFuse user data block."""

from esphome_provision.common import ProvisionError
from esphome_provision.efuse import HEX_BYTES

USER_DATA = "BLOCK_USR_DATA"
BOARD_IDS = {"v1": 0, "v2": 1}
BOARD_NAMES = {0: "V1 (legacy)", 1: "V2", 2: "KKZM"}


def board_id(raw: str) -> int:
    if not HEX_BYTES.fullmatch(raw.strip()):
        raise ProvisionError(
            "Cannot read the eFuse BLOCK_USR_DATA, it is hidden or malformed"
        )
    return int(raw.split()[0], 16)


def check_board(found: int, board: str) -> None:
    if found != BOARD_IDS[board]:
        name = BOARD_NAMES.get(found, f"unknown id {found}")
        raise ProvisionError(f"The board id says {name} but --board {board} was given")
