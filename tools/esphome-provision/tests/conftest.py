import os
from pathlib import Path

import pytest

DUMPS_ENV = "ZM_DUMPS_DIR"


@pytest.fixture
def dumps() -> Path:
    value = os.environ.get(DUMPS_ENV)
    if not value or not Path(value).is_dir():
        pytest.skip(f"{DUMPS_ENV} is not set")
    return Path(value)


def firmware_path(env: str) -> Path:
    value = os.environ.get(env)
    if not value or not Path(value).is_file():
        pytest.skip(f"{env} is not set")
    return Path(value)


@pytest.fixture
def firmware_v1() -> Path:
    return firmware_path("ZM_FIRMWARE_V1")


@pytest.fixture
def firmware_v2() -> Path:
    return firmware_path("ZM_FIRMWARE_V2")
