"""A MavBus is closed only once its reader really stopped."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from navpy.plugins.vehicle.ardupilot.mav_bus_lifecycle import MavBusLifecycle


def _lifecycle(reader_stops):
    connection = MagicMock()
    unregister = MagicMock()
    membership = MagicMock(is_empty=True)
    lifecycle = MavBusLifecycle(connection, "udp:0.0.0.0:5760", membership, unregister)
    reader = MagicMock()
    reader.join.side_effect = reader_stops
    lifecycle.bind_reader(reader)
    return lifecycle, connection, unregister


def test_a_reader_stuck_in_a_handler_keeps_the_bus_closing_and_fails_each_attempt():
    lifecycle, connection, unregister = _lifecycle([False, False])

    for _attempt in range(2):
        with pytest.raises(TimeoutError, match="did not stop"):
            lifecycle.close()
        assert lifecycle.is_closing and not lifecycle.is_closed
    connection.close.assert_not_called()
    unregister.assert_not_called()


def test_the_retry_closes_once_the_reader_stopped():
    lifecycle, connection, unregister = _lifecycle([False, True])

    with pytest.raises(TimeoutError):
        lifecycle.close()
    lifecycle.close()

    assert lifecycle.is_closed
    connection.close.assert_called_once()
    unregister.assert_called_once()


def test_the_reader_finishes_a_close_that_timed_out_on_it():
    # Backend vehicle removal gives up on a busy reader; when the reader exits,
    # the bus must still close and unregister, or a reconnect waits forever.
    lifecycle, connection, unregister = _lifecycle([False])

    with pytest.raises(TimeoutError):
        lifecycle.close()
    lifecycle.reader_exited()

    assert lifecycle.wait_closed(0.0)
    connection.close.assert_called_once()
    unregister.assert_called_once()


def test_a_reader_that_exits_during_the_join_timeout_does_not_fail_the_close():
    lifecycle, connection, unregister = _lifecycle([False])
    lifecycle.reader_exited()  # exited just as the join gave up

    lifecycle.close()

    assert lifecycle.is_closed
    unregister.assert_called_once()


def test_a_normal_close_with_a_real_reader_does_not_deadlock():
    import threading
    import time

    from navpy.plugins.vehicle.ardupilot.mav_bus_reader import MavBusReader

    connection = MagicMock()
    connection.recv_match.side_effect = lambda **_kw: time.sleep(0.01)
    unregister = MagicMock()
    lifecycle = MavBusLifecycle(connection, "udp:0.0.0.0:5760", MagicMock(), unregister)
    reader = MavBusReader(connection, "udp:0.0.0.0:5760", MagicMock(), MagicMock(),
                          lifecycle.stop_event, on_exit=lifecycle.reader_exited)
    lifecycle.bind_reader(reader)
    reader.start()

    started = time.monotonic()
    lifecycle.close()

    assert lifecycle.is_closed and time.monotonic() - started < 1.0
    assert not any(t.name == reader._thread.name and t.is_alive() for t in threading.enumerate())
