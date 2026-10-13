"""The logging stamp: records are timed by t_k and formatted in UTC."""

from __future__ import annotations

import io
import logging
import time

from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.log_stamp import install_log_stamp

EPOCH_US = 1_760_000_000_123_456  # 2025-10-09T08:53:20.123456Z


def test_records_follow_t_k_in_utc():
    baton = BatonRuntime(start_us=40_000)
    runtime = LockstepRuntime.create(baton, epoch_us=EPOCH_US, seed=1, ordinal=1)
    stream = io.StringIO()
    handler = logging.StreamHandler(stream)
    handler.setFormatter(logging.Formatter("%(asctime)s|%(relativeCreated)d|%(message)s"))
    logger = logging.getLogger("lockstep.test_log_stamp")
    logger.addHandler(handler)
    logger.propagate = False
    records = []
    logger.addFilter(lambda record: records.append(record) or True)
    previous_factory = logging.getLogRecordFactory()
    uninstall = install_log_stamp(runtime.clock)
    try:
        assert logging.Formatter.converter is time.gmtime
        baton.tick(1_040_000)  # one sim second after the runtime's start
        logger.warning("one second in")
    finally:
        uninstall()
        logger.removeHandler(handler)
    (record,) = records
    unix_us = EPOCH_US + 1_040_000
    assert record.created == unix_us / 1_000_000
    assert record.msecs == 163.0  # .163456 s, truncated to whole milliseconds
    assert record.relativeCreated == 1000.0
    assert stream.getvalue() == "2025-10-09 08:53:21,163|1000|one second in\n"
    assert logging.getLogRecordFactory() is previous_factory
    assert logging.Formatter.converter is time.localtime
