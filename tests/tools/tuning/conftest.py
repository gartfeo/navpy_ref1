"""Every tuning test reads synthetic aircraft data, never the owner's private folder (hitl_data.py)."""
from __future__ import annotations

import pytest

from tests.tools.tuning.hitl_data import write_hitl_data
from tools.tuning.private_data import PRIVATE_DIR_ENV


@pytest.fixture(scope="session")
def synthetic_private_dir(tmp_path_factory):
    return write_hitl_data(tmp_path_factory.mktemp("navpy-hitl-tools"))


@pytest.fixture(autouse=True)
def _use_synthetic_private_dir(synthetic_private_dir, monkeypatch):
    monkeypatch.setenv(PRIVATE_DIR_ENV, str(synthetic_private_dir))
