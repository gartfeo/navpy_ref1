"""1x pacing (``deterministic_dev_sim.md`` section 5, "Pacing to 1x").

The coordinator calls the pacer after each tick's reply with ``t_k``; it
sleeps until ``wall_start + t_k``, where the first tick it sees anchors
``wall_start``. A PC slower than 1x never sleeps, so the run runs slower.
It sits where the host delays sleep and never enters the journal, so a
paced run has the digest of an unpaced one.
"""

from __future__ import annotations

import time
from collections.abc import Callable


class RealTimePacer:
    def __init__(self, *, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self._clock = clock
        self._sleep = sleep
        self._anchor: float | None = None

    def __call__(self, t_us: int) -> None:
        if self._anchor is None:
            self._anchor = self._clock() - t_us / 1_000_000
            return
        wait = self._anchor + t_us / 1_000_000 - self._clock()
        if wait > 0:
            self._sleep(wait)


__all__ = ["RealTimePacer"]
