"""Whether the determinism trace records at all, and how much it may record.

Split from the recorder because it is POLICY, not recording: two environment
variables, their defaults, and how a malformed value is treated. The recorder
had grown to its size limit carrying this, and the two have no reason to change
together -- the row layouts move with the landings, the off switch does not.

The gate is read ONCE, at import, into ``determinism_trace.ENABLED``. Reading it
per construction would let a mid-run environment change give two sources
different answers, and a trace that covers some sources and not others is worse
evidence than no trace.
"""

from __future__ import annotations

import os
from typing import Any


DETERMINISM_TRACE_ENV = "NAVPY_DETERMINISM_TRACE"
DETERMINISM_TRACE_CAPACITY_ENV = "NAVPY_DETERMINISM_TRACE_ROWS"
# ~1350 projected frames per archived vehicle-case across five row kinds, with
# generous headroom: a full leg fits, and a runaway latches overflowed instead
# of growing without bound.
DEFAULT_TRACE_CAPACITY = 150_000


def enabled_from_env(env: Any = None) -> bool:
    """Off unless explicitly switched on, and "0"/"off"/"false"/"no" are off."""
    raw = (os.environ if env is None else env).get(DETERMINISM_TRACE_ENV)
    if raw is None:
        return False
    return str(raw).strip().lower() not in ("", "0", "off", "false", "no")


def capacity_from_env(env: Any = None) -> int:
    """The row budget, falling back to the default rather than to unbounded.

    A malformed or non-positive value takes the default: the alternative is a
    trace that either grows without limit or records nothing, and both look
    like a clean run to a reader who does not check the status.
    """
    raw = (os.environ if env is None else env).get(
        DETERMINISM_TRACE_CAPACITY_ENV
    )
    if raw is None:
        return DEFAULT_TRACE_CAPACITY
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError):
        return DEFAULT_TRACE_CAPACITY
    return value if value > 0 else DEFAULT_TRACE_CAPACITY


__all__ = [
    "DEFAULT_TRACE_CAPACITY",
    "DETERMINISM_TRACE_CAPACITY_ENV",
    "DETERMINISM_TRACE_ENV",
    "capacity_from_env",
    "enabled_from_env",
]
