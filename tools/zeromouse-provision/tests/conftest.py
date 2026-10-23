import os
from pathlib import Path

import pytest

ENV = "ZM_DUMPS_DIR"


@pytest.fixture
def dumps() -> Path:
    value = os.environ.get(ENV)
    if not value or not Path(value).is_dir():
        pytest.skip(f"{ENV} is not set")
    return Path(value)
