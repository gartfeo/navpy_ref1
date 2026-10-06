"""
Tests for the confirm-request resend-while-CONFIRMING loop (D-07/D-11),
added on top of the existing ConfirmationManager suite in test_confirmation_manager.py
(which stays green: RESEND_INTERVAL_S=2.0 always exceeds every
confirm_wait_time_sec used there, so no resend fires in those tests).

The loop has exactly two exits: a final confirm response arrives, or the
single confirm_wait_time_sec window (counted from the first send) expires.
"""
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import (
    TaskConfirmRequestMsg, TaskConfirmResponseMsg,
)
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.location import Location
from navpy.modules.nav import confirmation_manager as tm_module
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from tests.detection_factory import make_detected_poi


class _RaceEvent(threading.Event):
    """Event whose wait() runs a callback the first time it times out, and
    then still reports the timeout.

    That is exactly the resend loop's race window: the GCS response lands
    after wait() has already decided to time out, but before the loop
    reaches its pre-broadcast guard. Only the pending-confirmation event is
    ever waited on (the cancel handle is polled with is_set()), so the
    callback can only fire from the resend loop's own wait.
    """

    # Both hooks are set by a test and fire once, then clear.
    on_first_timeout = None   # fires inside a wait() that times out
    on_first_false_is_set = None  # fires when the resend guard samples False

    def __init__(self):
        super().__init__()
        # Only the pending-confirmation event is ever waited on, so this
        # flag tells it apart from the worker's cancel handle.
        self._is_pending_token = False

    def wait(self, timeout=None):
        self._is_pending_token = True
        if super().wait(timeout):
            return True
        callback = _RaceEvent.on_first_timeout
        if callback is not None:
            _RaceEvent.on_first_timeout = None
            callback()
        return False

    def is_set(self):
        signalled = super().is_set()
        if signalled or not self._is_pending_token:
            return signalled
        callback = _RaceEvent.on_first_false_is_set
        if callback is not None:
            _RaceEvent.on_first_false_is_set = None
            callback()
        return False


class _ThreadingWithRaceEvent:
    """Stand-in for confirmation_manager's ``threading`` module that hands out
    _RaceEvent in place of threading.Event and proxies everything else.
    Patched onto that module alone, so no other thread in the test process
    sees a different Event class."""

    Event = _RaceEvent

    def __getattr__(self, name):
        return getattr(threading, name)


class ConfirmationManagerResendTests(unittest.TestCase):
    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 1.0
        self.mock_args.is_confirm_on_fail = False

        self.mock_network = MagicMock(spec=NetworkAbc)

        self.confirmation_manager = ConfirmationManager(sys_id=1, args=self.mock_args, logger=self.mock_logger)
        self.confirmation_manager.set_network(self.mock_network)

        self.poi = make_detected_poi(
            obj_id=501, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.poi.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))

    def _confirm_request_calls(self):
        """Broadcast calls that are TaskConfirmRequestMsg (excludes self-assign)."""
        return [
            c[0][0] for c in self.mock_network.broadcast.call_args_list
            if isinstance(c[0][0], TaskConfirmRequestMsg)
        ]

    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.1)
    def test_resends_until_window_expires_then_times_out(self):
        """Exit 2: with no response at all the loop resends on the
        RESEND_INTERVAL_S cadence, then resolves at the end of the window via
        status_on_fail (TIMEOUT_REJECTED under the REJECT policy,
        D-14/Pitfall 4)."""
        self.mock_args.confirm_wait_time_sec = 0.6

        self.confirmation_manager.review([self.poi])
        time.sleep(0.25)
        self.assertGreaterEqual(
            len(self._confirm_request_calls()), 2, "expected at least 2 resends by 0.25s",
        )

        time.sleep(0.6)  # well past the window
        self.assertEqual(
            self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED,
        )

    def _await_status(self, *expected, timeout=3.0):
        """Block until the POI reaches one of *expected* statuses."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.confirmation_manager.get_status(self.poi)
            if status in expected:
                return status
            time.sleep(0.005)
        return self.confirmation_manager.get_status(self.poi)

    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.05)
    def test_response_landing_at_wait_timeout_cancels_that_resend(self):
        """Exit 1, at its exact race boundary: a final response that lands
        between wait() timing out and the resend broadcast must cancel that
        broadcast, leaving only the initial send.

        Deterministic -- the response is delivered from inside the very
        wait() call that times out, so the loop always reaches its
        pre-broadcast guard with the response already in hand.
        """
        # Long window: nothing but the response can end this round, so a
        # surviving resend cannot be explained by the timeout path.
        self.mock_args.confirm_wait_time_sec = 5.0
        delivered = threading.Event()

        def _deliver_response():
            self.confirmation_manager.on_message(TaskConfirmResponseMsg(
                receiver_id=1,
                task_id=self.poi.identity.obj_id,
                is_confirmed=True,
            ))
            delivered.set()

        _RaceEvent.on_first_timeout = _deliver_response
        self.addCleanup(setattr, _RaceEvent, "on_first_timeout", None)

        with patch.object(tm_module, "threading", _ThreadingWithRaceEvent()):
            self.confirmation_manager.review([self.poi])
            self.assertTrue(
                delivered.wait(timeout=3.0), "response was never delivered by the race hook",
            )
            self.assertEqual(
                self._await_status(ConfirmationStatus.CONFIRMED), ConfirmationStatus.CONFIRMED,
            )

        self.assertEqual(
            len(self._confirm_request_calls()), 1,
            "the resend racing the arriving response must be cancelled: only "
            "the initial confirm-request should ever be sent",
        )

    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.05)
    def test_no_resend_is_sent_after_the_response_is_consumed(self):
        """The response handler and the resend must share one atomic order:
        once the handler has consumed the pending token, NO further
        confirm-request may go out.

        The response is delivered from another thread at the moment the guard
        has just sampled the token as not-set -- the interval a plain
        check-then-send leaves open. The hook waits for that thread, so if
        the check and the send are not atomic the handler completes first and
        the send that follows is a post-decision send.
        """
        self.mock_args.confirm_wait_time_sec = 5.0
        events = []
        events_lock = threading.Lock()

        def _record(name):
            with events_lock:
                events.append(name)

        def _recording_broadcast(message):
            if isinstance(message, TaskConfirmRequestMsg):
                _record("send")
        self.mock_network.broadcast.side_effect = _recording_broadcast

        def _deliver_from_other_thread():
            def _worker():
                self.confirmation_manager.on_message(TaskConfirmResponseMsg(
                    receiver_id=1,
                    task_id=self.poi.identity.obj_id,
                    is_confirmed=True,
                ))
                _record("response-consumed")
            thread = threading.Thread(target=_worker, daemon=True)
            thread.start()
            # Give the response every chance to land BEFORE the resend. If
            # check-and-send is atomic this join times out (the handler is
            # waiting on the lock the loop holds), which is the pass case.
            thread.join(timeout=0.5)

        _RaceEvent.on_first_false_is_set = _deliver_from_other_thread
        self.addCleanup(setattr, _RaceEvent, "on_first_false_is_set", None)

        with patch.object(tm_module, "RESEND_INTERVAL_S", 0.05), \
                patch.object(tm_module, "threading", _ThreadingWithRaceEvent()):
            self.confirmation_manager.review([self.poi])
            self.assertEqual(
                self._await_status(ConfirmationStatus.CONFIRMED), ConfirmationStatus.CONFIRMED,
            )

        with events_lock:
            ordering = list(events)
        self.assertIn("response-consumed", ordering, "the response was never processed")
        consumed_at = ordering.index("response-consumed")
        self.assertNotIn(
            "send", ordering[consumed_at:],
            f"a confirm-request was sent after the response was consumed: {ordering}",
        )

    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.05)
    def test_window_counted_from_first_send_not_extended_by_resends(self):
        """D-11: the single confirm_wait_time_sec window is counted from the
        FIRST send; the resends inside it must not restart or extend it."""
        self.mock_args.confirm_wait_time_sec = 0.3
        start = time.monotonic()

        self.confirmation_manager.review([self.poi])

        deadline = start + self.mock_args.confirm_wait_time_sec + 0.4  # generous slack
        while time.monotonic() < deadline:
            if self.confirmation_manager.get_status(self.poi) in (
                    ConfirmationStatus.CONFIRMED, ConfirmationStatus.REJECTED, ConfirmationStatus.TIMEOUT_REJECTED):
                break
            time.sleep(0.01)

        elapsed = time.monotonic() - start
        self.assertGreaterEqual(
            len(self._confirm_request_calls()), 2, "expected resends inside the window",
        )
        self.assertLess(
            elapsed, self.mock_args.confirm_wait_time_sec + 0.3,
            "resends must not restart/extend the confirm_wait_time_sec window",
        )

    def test_resend_cadence_is_wall_clock_not_sim_scaled(self):
        """RESEND_INTERVAL_S paces off time.monotonic() (real wall time).
        confirmation_manager.py has no SIM_SPEEDUP dependency anywhere in this
        loop (no vehicle.sim_speedup() call, no AAS_SIM_SPEEDUP read), so
        even if some OTHER part of the process were running against a
        10x-accelerated sim clock, this resend cadence is unaffected -- it
        is governed purely by real wall-clock time, matching the
        CONFIRM_MAX_DWELL / task_confirm.py resend-gap precedent."""
        with patch.object(tm_module, "RESEND_INTERVAL_S", 0.15):
            self.mock_args.confirm_wait_time_sec = 0.65
            timestamps = []
            lock = threading.Lock()

            def _timed_broadcast(message):
                if isinstance(message, TaskConfirmRequestMsg):
                    with lock:
                        timestamps.append(time.monotonic())

            self.mock_network.broadcast.side_effect = _timed_broadcast

            self.confirmation_manager.review([self.poi])
            time.sleep(0.75)

        self.assertGreaterEqual(len(timestamps), 3, "expected initial send + at least 2 resends")
        gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
        for gap in gaps:
            self.assertAlmostEqual(
                gap, 0.15, delta=0.1,
                msg=f"resend gap {gap:.3f}s should track RESEND_INTERVAL_S wall-clock",
            )

    def test_short_window_sends_exactly_one_request_no_resend(self):
        """Regression guard matching test_confirmation_manager.py's existing
        expectations: with the default RESEND_INTERVAL_S (2.0s) and a short
        confirm_wait_time_sec, exactly one confirm-request is sent (no
        resend fires before the window closes)."""
        self.mock_args.confirm_wait_time_sec = 0.1
        self.confirmation_manager.review([self.poi])
        time.sleep(0.25)
        self.assertEqual(len(self._confirm_request_calls()), 1)


if __name__ == "__main__":
    unittest.main()
