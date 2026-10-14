"""The logging stamp: every log record is timed by the lockstep clock.

Installed at bootstrap step 2, before the first record. A record factory sets
``created``, ``msecs`` and ``relativeCreated`` from the runtime's unix clock
(epoch plus ``t_k``; ``relativeCreated`` counts from the runtime's start), and
``logging.Formatter.converter`` is pinned to ``time.gmtime``, so ``asctime``
is the same on every host. The product's ``CacheLogger`` formats through a
``logging.Formatter`` (``logger_factory.py``), so its lines follow the stamp;
its worker creates each record when it handles the message, on the baton.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from navpy.sim.lockstep.lockstep_runtime import LockstepClock


def install_log_stamp(clock: LockstepClock) -> Callable[[], None]:
    """Stamp records from ``clock``; returns the uninstall (for tests)."""
    previous_factory = logging.getLogRecordFactory()
    previous_converter = logging.Formatter.converter

    def stamped_record(*args: Any, **kwargs: Any) -> logging.LogRecord:
        record = previous_factory(*args, **kwargs)
        unix_us = clock.unix_us()
        record.created = unix_us / 1_000_000
        record.msecs = (unix_us % 1_000_000) // 1000 + 0.0
        record.relativeCreated = clock.since_start_us() / 1000
        return record

    logging.setLogRecordFactory(stamped_record)
    logging.Formatter.converter = time.gmtime

    def uninstall() -> None:
        logging.setLogRecordFactory(previous_factory)
        logging.Formatter.converter = previous_converter

    return uninstall


__all__ = ["install_log_stamp"]
