"""Whether file logs roll over.

Long-lived logs rotate. An evaluator sets ``NAVPY_LOG_ROTATION=off`` so each
run keeps one complete backend log and one complete log per companion.
"""

from __future__ import annotations

import os
from collections.abc import Mapping


LOG_ROTATION_ENV = "NAVPY_LOG_ROTATION"


def rotation_max_bytes(
    default_bytes: int,
    env: Mapping[str, str] | None = None,
) -> int:
    """``RotatingFileHandler`` ``maxBytes``: 0 (never roll) when rotation is off."""
    value = (os.environ if env is None else env).get(LOG_ROTATION_ENV, "")
    setting = value.strip().lower()
    if setting in ("", "on"):
        return default_bytes
    if setting == "off":
        return 0
    raise ValueError(f"{LOG_ROTATION_ENV} must be 'on' or 'off', got {value!r}")


__all__ = ["LOG_ROTATION_ENV", "rotation_max_bytes"]
