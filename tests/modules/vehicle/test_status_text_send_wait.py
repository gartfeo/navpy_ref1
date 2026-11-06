"""Status texts wait for a busy bus send lock, for a bounded time (ready plan step 4, todo 167)."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.vehicle_messaging import (
    STATUS_TEXT_SEND_WAIT_S,
    VehicleMessaging,
)
from tests.logger.log_worker_join import unbounded_worker_join


def _messaging():
    mav = MagicMock()
    lock = threading.RLock()
    logger = MagicMock()
    messaging = VehicleMessaging(
        MavTransport(SimpleNamespace(mav=mav), lock),
        callbacks=MagicMock(),
        router=MagicMock(),
        logger_ref=LoggerRef(logger),
    )
    return messaging, mav, lock, logger


def _hold_lock(lock, held, release):
    with lock:
        held.set()
        release.wait(5.0)


def test_a_text_waits_for_a_send_in_progress_and_is_sent():
    messaging, mav, lock, logger = _messaging()
    held, release = threading.Event(), threading.Event()
    holder = threading.Thread(target=_hold_lock, args=(lock, held, release))
    holder.start()
    assert held.wait(5.0)

    sender = threading.Thread(target=messaging.send_status_text, args=("NAV 1",))
    sender.start()
    time.sleep(STATUS_TEXT_SEND_WAIT_S / 5)
    mav.statustext_send.assert_not_called()  # still waiting, not dropped
    release.set()
    sender.join(5.0)
    holder.join(5.0)

    mav.statustext_send.assert_called_once()
    assert mav.statustext_send.call_args[0][1] == b"NAV 1"
    logger.warning.assert_not_called()


def test_the_wait_is_bounded_and_a_dropped_text_is_logged():
    messaging, mav, lock, logger = _messaging()
    held, release = threading.Event(), threading.Event()
    holder = threading.Thread(target=_hold_lock, args=(lock, held, release))
    holder.start()
    assert held.wait(5.0)
    try:
        started = time.perf_counter()
        messaging.send_status_text("NAV 2")
        waited_s = time.perf_counter() - started
    finally:
        release.set()
        holder.join(5.0)

    assert waited_s >= STATUS_TEXT_SEND_WAIT_S * 0.9
    assert waited_s < STATUS_TEXT_SEND_WAIT_S + 1.0
    mav.statustext_send.assert_not_called()
    logger.warning.assert_called_once()
    text = logger.warning.call_args[0][0]
    assert "status text dropped" in text
    assert "'NAV 2'" in text
    # Text log only: a status-bound warning would queue another status text.
    assert logger.warning.call_args.kwargs.get("dest") is None
    assert logger.warning.call_args.kwargs.get("status") is None


def test_the_owning_thread_sends_without_waiting():
    messaging, mav, lock, logger = _messaging()
    with lock:  # e.g. a flush on a thread inside a transaction: the lock is re-entrant
        messaging.send_status_text("NAV 3")

    mav.statustext_send.assert_called_once()
    logger.warning.assert_not_called()


def test_a_deferred_flush_waits_once_on_a_stuck_lock_and_keeps_the_rest():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    messaging, mav, lock, logger = _messaging()
    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = messaging  # set_vehicle reads a parameter; only sends matter here
    status.defer_status_texts(True)
    for index in range(5):
        status.send_log(f"NAV {index}", LogStatusDest.DRONE)

    held, release = threading.Event(), threading.Event()
    holder = threading.Thread(target=_hold_lock, args=(lock, held, release))
    holder.start()
    assert held.wait(5.0)
    try:
        started = time.perf_counter()
        status.defer_status_texts(False, flush=True)  # NAV exit, on the caller's thread
        waited_s = time.perf_counter() - started
    finally:
        release.set()
        holder.join(5.0)

    # One bounded wait for the whole flush, not one per pending text.
    assert waited_s < 2 * STATUS_TEXT_SEND_WAIT_S + 0.5
    mav.statustext_send.assert_not_called()
    logger.warning.assert_called_once()

    status.close()  # the rest, in order, once the bus is free
    sent = [call[0][1] for call in mav.statustext_send.call_args_list]
    assert sent == [b"NAV 1", b"NAV 2", b"NAV 3", b"NAV 4"]


def test_a_flush_whose_every_send_waits_stops_at_its_budget():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.link import status_logger_network
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    sent = []

    class _SlowVehicle:
        """Every send succeeds after most of the lock bound, as behind a holder that keeps clearing late."""

        def send_status_text(self, message):
            time.sleep(STATUS_TEXT_SEND_WAIT_S * 0.9)
            sent.append(message)
            return True

    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = _SlowVehicle()
    status.defer_status_texts(True)
    queued = [f"NAV {index}" for index in range(6)]
    for text in queued:
        status.send_log(text, LogStatusDest.DRONE)

    started = time.perf_counter()
    status.defer_status_texts(False, flush=True)  # NAV exit, on the caller's thread
    waited_s = time.perf_counter() - started

    budget_s = status_logger_network.DEFERRED_FLUSH_SEND_BUDGET_S
    # At most the budget plus one send's wait (and pacing), not one wait per text.
    assert waited_s < budget_s + STATUS_TEXT_SEND_WAIT_S + 0.5
    # One or two sends, by timer resolution; the rest stays pending, in order.
    assert sent in (queued[:1], queued[:2])
    assert sent + status._pending_status_texts == queued

    # The next text (on the logger worker) sends the rest first, in order, with
    # the same budget; close sends whatever is still left.
    expected = queued + ["READY"]
    before = len(sent)
    started = time.perf_counter()
    status.send_log("READY", LogStatusDest.DRONE)
    waited_s = time.perf_counter() - started
    assert waited_s < budget_s + STATUS_TEXT_SEND_WAIT_S + 0.5
    assert len(sent) - before in (1, 2)
    assert sent + status._pending_status_texts == expected
    assert status.close() == []
    assert sent == expected


def test_a_worker_drain_holds_the_status_lock_only_for_its_budget():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.kernel.logging.cache_log_level import CacheLogLevel
    from navpy.core.kernel.logging.cache_log_output import CacheStatusOutput
    from navpy.core.link import status_logger_network
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    class _SlowVehicle:
        def send_status_text(self, message):
            time.sleep(STATUS_TEXT_SEND_WAIT_S * 0.9)
            return True

    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = _SlowVehicle()
    status._pending_status_texts = [f"NAV {index}" for index in range(30)]
    output = CacheStatusOutput(
        SimpleNamespace(status_level=CacheLogLevel.ERROR, status_update_interval=0.0),
        status,
    )

    # The logger worker emits a text, which drains the pending ones under the output lock.
    worker = threading.Thread(target=output.emit, args=("READY", CacheLogLevel.INFO))
    worker.start()
    time.sleep(STATUS_TEXT_SEND_WAIT_S / 2)
    started = time.perf_counter()
    output.defer(True, flush=False)  # NAV entry, on the navigation thread
    waited_s = time.perf_counter() - started
    worker.join(10.0)

    budget_s = status_logger_network.DEFERRED_FLUSH_SEND_BUDGET_S
    assert waited_s < budget_s + STATUS_TEXT_SEND_WAIT_S + 0.5
    assert len(status._pending_status_texts) > 20  # the rest waits for later


def test_close_sends_every_text_left_by_a_budgeted_flush():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    sent = []

    class _SlowVehicle:
        def send_status_text(self, message):
            time.sleep(STATUS_TEXT_SEND_WAIT_S * 0.9)
            sent.append(message)
            return True

    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = _SlowVehicle()
    status.defer_status_texts(True)
    queued = [f"NAV {index}" for index in range(6)]
    for text in queued:
        status.send_log(text, LogStatusDest.DRONE)
    status.defer_status_texts(False, flush=True)
    assert sent != queued  # the budget left some

    assert status.close() == []
    assert sent == queued


def test_close_returns_the_texts_it_could_not_send():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    class _BusyVehicle:
        def send_status_text(self, message):
            return message != "NAV 1"

    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = _BusyVehicle()
    status._pending_status_texts = ["NAV 0", "NAV 1", "NAV 2"]
    assert status.close() == ["NAV 1"]
    assert status._pending_status_texts == []


def test_logger_close_writes_unsent_status_texts_to_the_text_log():
    from navpy.core.kernel.config.logger_args import LoggerArgsStub
    from navpy.core.kernel.logging.cache_logger import CacheLogger

    text = MagicMock()
    sink = MagicMock()
    sink.close.return_value = ["NAV 7"]
    logger = CacheLogger(LoggerArgsStub(), text, sink)
    with unbounded_worker_join():
        logger.close()

    warnings = [call[0][0] for call in text.warning.call_args_list]
    assert any("status text not sent at close" in w and "'NAV 7'" in w for w in warnings)


def test_a_drain_of_fast_sends_counts_its_pacing_in_the_budget():
    from navpy.core.kernel.config.logger_args import LogStatusDest
    from navpy.core.link import status_logger_network
    from navpy.core.link.status_logger_network import GroundStationLoggerNetwork

    sent = []

    class _FastVehicle:
        def send_status_text(self, message):
            sent.append(message)
            return True

    status = GroundStationLoggerNetwork(
        1, SimpleNamespace(status_dest=[LogStatusDest.DRONE], defer_status_logs=True), 0,
    )
    status.vehicle = _FastVehicle()
    queued = [f"NAV {index}" for index in range(100)]  # 100 paced sends take about 1 s
    status._pending_status_texts = list(queued)

    started = time.perf_counter()
    status.send_log("READY", LogStatusDest.DRONE)  # on the logger worker
    waited_s = time.perf_counter() - started

    budget_s = status_logger_network.DEFERRED_FLUSH_SEND_BUDGET_S
    assert waited_s < budget_s + 0.2
    assert 0 < len(sent) < 50
    assert sent + status._pending_status_texts == queued + ["READY"]
