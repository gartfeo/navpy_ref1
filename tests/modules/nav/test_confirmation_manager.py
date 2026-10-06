import threading
import time
import unittest
from dataclasses import replace
from time import sleep
from unittest.mock import MagicMock, patch

from navpy.args.nav_args import NavArgs
from navpy.modules.comm.messages.available_task_msg import (
    TaskAssignRequestMsg,
    TaskConfirmRequestMsg,
    TaskConfirmResponseMsg,
)
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.nav.confirmation_manager import (
    ConfirmationStatus,
    ConfirmationManager,
)
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from navpy.modules.vision.models.detection_components import ConfirmationEvidence
from tests.detection_factory import make_detected_poi
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location


class ConfirmationManagerTests(unittest.TestCase):
    def setUp(self):
        """
        Set up mocks and the ConfirmationManager instance before each test.
        """
        # Mock ILogger
        self.mock_logger = MagicMock(spec=ILogger)

        # Mock NavArgs
        self.mock_nav_args = MagicMock(spec=NavArgs)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.confirm_wait_time_sec = 0.1  # short wait for tests

        # Mock NetworkAbc
        self.mock_network = MagicMock(spec=NetworkAbc)

        # Initialize ConfirmationManager
        self.confirmation_manager = ConfirmationManager(
            sys_id=1,
            args=self.mock_nav_args,
            logger=self.mock_logger
        )

        # Create real DetectedObject objects with a valid location
        t1 = make_detected_poi(
            obj_id=101,
            size_class=DetectionSizeClass.S,
            x_error=0,
            y_error=0,
            reference_height_m=0,
            k=0,
            g_data=None,
            uas_att=None
        )
        t1.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        t2 = make_detected_poi(
            obj_id=202,
            size_class=DetectionSizeClass.M,
            x_error=0,
            y_error=0,
            reference_height_m=0,
            k=0,
            g_data=None,
            uas_att=None
        )
        t2.set_p_t_g_loc(Location(lat=11, lng=21, alt=110))
        self.detected_pois = [t1, t2]

    def _install_response_round(self, poi, event):
        """Create the exact lease/token shape used by a production review."""
        lease = self.confirmation_manager._state.start_review(poi, threading.Event)
        confirmation = self.confirmation_manager._state.begin_round(
            poi,
            lease,
            lambda: event,
        )
        self.assertIsNotNone(confirmation)
        return lease

    def _wait_until(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            sleep(0.005)
        return bool(predicate())

    def test_synchronous_confirmation_callback_does_not_deadlock_round_send(self):
        """A loopback transport may deliver the response inside broadcast()."""
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.confirm_wait_time_sec = 1.0
        poi = self.detected_pois[0]
        self.confirmation_manager._media.send = MagicMock()

        def respond_during_broadcast(message):
            if not isinstance(message, TaskConfirmRequestMsg):
                return
            if message.meta is None:
                message.set_meta(MsgMeta(501, 1, 1000, 5000))
            request_meta = message.meta
            self.confirmation_manager.on_message(TaskConfirmResponseMsg(
                receiver_id=1,
                task_id=poi.identity.obj_id,
                is_confirmed=True,
                meta=MsgMeta(
                    request_meta.boot_id,
                    request_meta.msg_seq,
                    request_meta.time_ms + 1,
                    request_meta.ttl_ms,
                ),
            ))

        self.mock_network.broadcast.side_effect = respond_during_broadcast

        self.confirmation_manager.review([poi])

        self.assertTrue(
            self._wait_until(
                lambda: self.confirmation_manager.get_status(poi)
                is ConfirmationStatus.CONFIRMED,
            ),
            "synchronous inbound response deadlocked the outbound transaction",
        )
        self.assertNotIn(poi.identity.obj_id, self.confirmation_manager._state.pending_events())
        self.confirmation_manager._media.send.assert_not_called()

    def test_stale_response_token_cannot_resolve_reused_poi_id(self):
        """A response belongs to the request round, not merely its task ID."""
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.confirm_wait_time_sec = 2.0
        poi = self.detected_pois[0]
        requests = []

        def capture_request(message):
            if not isinstance(message, TaskConfirmRequestMsg):
                return
            if message.meta is None:
                message.set_meta(MsgMeta(601, len(requests) + 1, 1000, 5000))
            if not requests or requests[-1] is not message:
                requests.append(message)

        self.mock_network.broadcast.side_effect = capture_request

        self.confirmation_manager.review([poi])
        self.assertTrue(self._wait_until(lambda: len(requests) == 1))
        first_meta = requests[0].meta
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(
                first_meta.boot_id,
                first_meta.msg_seq,
                first_meta.time_ms + 1,
                first_meta.ttl_ms,
            ),
        ))
        self.assertTrue(self._wait_until(lambda: self.confirmation_manager._state.active_worker_count == 0))

        self.confirmation_manager.review([poi])
        self.assertTrue(self._wait_until(lambda: len(requests) == 2))
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(
                first_meta.boot_id,
                first_meta.msg_seq,
                first_meta.time_ms + 2,
                first_meta.ttl_ms,
            ),
        ))

        self.assertIs(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.CONFIRMING,
            "a delayed response from the first round resolved the replacement round",
        )
        self.assertIn(poi.identity.obj_id, self.confirmation_manager._state.pending_events())

        current_meta = requests[1].meta
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(
                current_meta.boot_id,
                current_meta.msg_seq,
                current_meta.time_ms + 1,
                current_meta.ttl_ms,
            ),
        ))
        self.assertTrue(self._wait_until(lambda: self.confirmation_manager._state.active_worker_count == 0))

    def test_metadata_less_response_fails_closed_after_poi_id_reuse(self):
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.confirm_wait_time_sec = 2.0
        poi = self.detected_pois[0]
        requests = []

        def capture_request(message):
            if not isinstance(message, TaskConfirmRequestMsg):
                return
            if message.meta is None:
                message.set_meta(MsgMeta(701, len(requests) + 1, 1000, 5000))
            if not requests or requests[-1] is not message:
                requests.append(message)

        self.mock_network.broadcast.side_effect = capture_request
        self.confirmation_manager.review([poi])
        self.assertTrue(self._wait_until(lambda: len(requests) == 1))
        first_meta = requests[0].meta
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(
                first_meta.boot_id,
                first_meta.msg_seq,
                first_meta.time_ms + 1,
                first_meta.ttl_ms,
            ),
        ))
        self.assertTrue(self._wait_until(lambda: self.confirmation_manager._state.active_worker_count == 0))

        self.confirmation_manager.review([poi])
        self.assertTrue(self._wait_until(lambda: len(requests) == 2))
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
        ))

        self.assertIs(self.confirmation_manager.get_status(poi), ConfirmationStatus.CONFIRMING)
        current_meta = requests[1].meta
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(
                current_meta.boot_id,
                current_meta.msg_seq,
                current_meta.time_ms + 1,
                current_meta.ttl_ms,
            ),
        ))
        self.assertTrue(self._wait_until(lambda: self.confirmation_manager._state.active_worker_count == 0))

    def test_auto_confirm_no_network(self):
        """
        If is_auto_confirm == True or no network is set, the manager should
        automatically set the status to CONFIRMED immediately.
        """
        self.mock_nav_args.is_auto_confirm = True
        self.confirmation_manager.review(self.detected_pois)

        sleep(0.05)  # let the background thread finish

        for poi in self.detected_pois:
            actual_status = self.confirmation_manager.get_status(poi)
            self.assertEqual(
                actual_status,
                ConfirmationStatus.CONFIRMED,
                f"POI {poi.identity.obj_id} should be CONFIRMED in auto-confirm mode.",
            )

    def test_no_network(self):
        """
        If network is None (and is_auto_confirm=False),
        the manager should still confirm (the code logs "no network").
        """
        self.confirmation_manager._network = None
        self.mock_nav_args.is_auto_confirm = False

        self.confirmation_manager.review(self.detected_pois)
        sleep(0.05)

        for poi in self.detected_pois:
            actual_status = self.confirmation_manager.get_status(poi)
            self.assertEqual(
                actual_status,
                ConfirmationStatus.CONFIRMED,
                f"POI {poi.identity.obj_id} should be CONFIRMED because there's no network.",
            )

    def test_network_broadcast_called(self):
        """
        If network is set, the manager should broadcast a self-assignment AND a confirm
        request for each POI (2 POIs × 2 broadcasts = 4 total).

        No freshness callback is wired on this bare ConfirmationManager (that
        injection is NavController's job -- see
        test_confirm_on_fail_freshness.py), so per D-22/Pitfall 5 the
        timeout fails CLOSED to TIMEOUT_REJECTED even with
        is_confirm_on_fail=True: an un-wired freshness signal must never be
        interpreted as "fresh" and silently auto-confirm.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.is_confirm_on_fail = True

        self.confirmation_manager.review(self.detected_pois)
        sleep(0.2)  # wait enough time for threads to broadcast

        # 2 self-assignments + 2 confirm requests = 4 broadcasts
        expected = len(self.detected_pois) * 2
        call_count = self.mock_network.broadcast.call_count
        self.assertEqual(
            call_count,
            expected,
            f"Expected {expected} broadcast calls (self-assign + confirm per POI), got {call_count}",
        )
        self.assertEqual(
            self.confirmation_manager.get_status(self.detected_pois[0]),
            ConfirmationStatus.TIMEOUT_REJECTED,
            "No freshness callback wired: timeout must fail CLOSED, not auto-confirm.",
        )
        self.assertEqual(
            self.confirmation_manager.get_status(self.detected_pois[1]),
            ConfirmationStatus.TIMEOUT_REJECTED,
            "No freshness callback wired: timeout must fail CLOSED, not auto-confirm.",
        )

    def test_network_incoming_confirm_response(self):
        """
        If the ground station responds with a confirm, the POI should become CONFIRMED.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False

        # Review only one POI
        poi = self.detected_pois[0]
        self.confirmation_manager.review([poi])

        # Give time for the background thread to broadcast and set status=CONFIRMING
        sleep(0.05)

        # Create a response indicating the GCS confirms the POI
        # Adjust constructor to match your real code
        response_msg = TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True
        )
        # manager listens in on_message
        self.confirmation_manager.on_message(response_msg)

        # WAIT a bit longer so the event is set and the manager updates the status
        sleep(0.05)

        actual_status = self.confirmation_manager.get_status(poi)
        self.assertEqual(
            actual_status,
            ConfirmationStatus.CONFIRMED,
            "Should set status to CONFIRMED upon positive response.",
        )

    def test_network_incoming_reject_response(self):
        """
        If the ground station responds with a reject, the POI should become REJECTED.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False

        poi = self.detected_pois[0]
        self.confirmation_manager.review([poi])
        sleep(0.05)

        # GCS rejects the POI
        response_msg = TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=False
        )
        self.confirmation_manager.on_message(response_msg)

        sleep(0.05)

        actual_status = self.confirmation_manager.get_status(poi)
        self.assertEqual(
            actual_status,
            ConfirmationStatus.REJECTED,
            "Should set status to REJECTED upon negative response.",
        )

    def test_late_confirmation_cannot_resurrect_timed_out_poi(self):
        """Run 000253 received approval after nav_cwt had already rejected.

        The timeout resolution is TIMEOUT_REJECTED (D-14/Pitfall 4, system/
        timeout-origin, distinct from an operator's explicit REJECTED) --
        the late approve must not resurrect it either way.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.is_confirm_on_fail = False
        self.mock_nav_args.confirm_wait_time_sec = 0.02
        poi = self.detected_pois[0]

        self.confirmation_manager.review([poi])
        sleep(0.06)
        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
        ))

        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )
        self.assertNotIn(poi.identity.obj_id, self.confirmation_manager._state.pending_events())

    def test_first_confirmation_consumes_token_and_duplicate_is_ignored(self):
        """A true duplicate (same decision, same is_confirmed value, resent --
        e.g. the GCS route's x3 resend of one POST) is a no-op after the
        first copy consumes the token: dev's pop-once ordering, no
        double-pop. This is NOT the same case as a later response carrying a
        DIFFERENT value once CONFIRMED (that is the operator's separate
        cancel decision -- see test_confirmed_then_reject_response_becomes_
        rejected). Duplicates are idempotent by construction here: once the
        second copy's is_confirmed matches the already-applied status, the
        no-token branch simply has nothing to change."""
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.confirm_wait_time_sec = 1.0
        poi = self.detected_pois[0]

        self.confirmation_manager.review([poi])
        sleep(0.05)
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
        ))
        # Duplicate copy of the SAME approve decision (same value).
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=poi.identity.obj_id,
            is_confirmed=True,
        ))
        sleep(0.02)

        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.CONFIRMED,
        )
        self.assertNotIn(poi.identity.obj_id, self.confirmation_manager._state.pending_events())

    def test_confirmed_then_reject_response_becomes_rejected(self):
        """Cancel/abort path: a CONFIRMED POI flips to REJECTED on a later
        negative response (CONFIRMED -> REJECTED stays allowed).

        Exercises the REAL production lifecycle, not a synthetic token
        injection: review() registers the one live token; the approve
        response consumes it (CONFIRMING -> CONFIRMED); the operator's later
        cancel response then arrives with NO live token (the only token was
        already consumed by the approve) -- this is exactly how a cancel
        response reaches the handler in production, since the GCS only lets
        the operator cancel an already-approved card
        (useTaskConfirmation.js's respond() gates cancel on status ===
        'approved'). Safe against wire duplicates because the network
        layer's MsgUID dedup cache collapses each GCS resend batch (the x3
        send) to one delivery before this handler ever runs.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.confirm_wait_time_sec = 1.0
        poi = self.detected_pois[0]
        task_id = poi.identity.obj_id

        self.confirmation_manager.review([poi])
        sleep(0.05)

        # Approve: consumes the one live token, CONFIRMING -> CONFIRMED.
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=True,
        ))
        self.assertEqual(self.confirmation_manager.get_status(poi), ConfirmationStatus.CONFIRMED)
        self.assertNotIn(task_id, self.confirmation_manager._state.pending_events())

        # Cancel: a later, separate operator decision. No live token exists
        # (the approve already consumed it) -- this is the production shape.
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=False,
        ))

        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.REJECTED,
            "A no-token negative response after approval must recall (REJECT) the POI.",
        )

    def test_no_token_positive_response_is_ignored_for_confirming_poi(self):
        """A no-token positive response for a still-CONFIRMING POI (no
        approve has happened yet) must not confirm it -- only a live-token
        response resolves the original confirm request."""
        poi = self.detected_pois[0]
        task_id = poi.identity.obj_id
        self.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=True,
        ))

        self.assertEqual(self.confirmation_manager.get_status(poi), ConfirmationStatus.CONFIRMING)

    def test_no_token_positive_response_is_ignored_for_rejected_poi(self):
        """A no-token positive response for an already-REJECTED POI must
        not un-reject it (monotonic guard also holds outside a live token)."""
        poi = self.detected_pois[0]
        task_id = poi.identity.obj_id
        self.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=True,
        ))

        self.assertEqual(self.confirmation_manager.get_status(poi), ConfirmationStatus.REJECTED)

    def test_rejected_poi_not_reconfirmed_by_late_approve(self):
        """Monotonic guard: a late/duplicate approve must NOT un-reject a
        recalled POI, but it must still wake any waiting confirm thread."""
        import threading

        from navpy.modules.vision.poi_identity import get_poi_task_id

        poi = self.detected_pois[0]
        task_id = get_poi_task_id(poi)
        event = threading.Event()
        self._install_response_round(poi, event)
        self.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=True,
        ))

        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.REJECTED,
            "A late approve must not un-reject a recalled POI.",
        )
        self.assertTrue(
            event.is_set(),
            "Waiter must be woken even when the confirm is ignored.",
        )

    def test_reconfirm_after_rereview_allowed(self):
        """Re-acquisition still works: review() rewrites CONFIRMING, so a
        subsequent approve confirms (CONFIRMING -> CONFIRMED).

        A live pending-confirmation token is registered before the response,
        matching how review()'s worker (_request_confirmation_from_network)
        would register one in production -- dev's ordering requires a live
        token for any response to take effect.
        """
        import threading

        from navpy.modules.vision.poi_identity import get_poi_task_id

        poi = self.detected_pois[0]
        task_id = get_poi_task_id(poi)
        self.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        event = threading.Event()
        self._install_response_round(poi, event)

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=True,
        ))

        self.assertEqual(
            self.confirmation_manager.get_status(poi),
            ConfirmationStatus.CONFIRMED,
            "A re-reviewed (CONFIRMING) POI must be confirmable again.",
        )
        self.assertTrue(event.is_set())

    def test_repeat_reject_is_idempotent_and_wakes_waiter(self):
        """A duplicate negative response leaves REJECTED and wakes waiters."""
        import threading

        from navpy.modules.vision.poi_identity import get_poi_task_id

        poi = self.detected_pois[0]
        task_id = get_poi_task_id(poi)
        event = threading.Event()
        self._install_response_round(poi, event)
        self.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=task_id, is_confirmed=False,
        ))

        self.assertEqual(self.confirmation_manager.get_status(poi), ConfirmationStatus.REJECTED)
        self.assertTrue(event.is_set())

    def test_reset_clears_data(self):
        """
        Ensure reset() clears the stored statuses and pending confirmations.
        """
        for t in self.detected_pois:
            self.confirmation_manager.update_status(t, ConfirmationStatus.CONFIRMING)

        for t in self.detected_pois:
            self.assertEqual(
                self.confirmation_manager.get_status(t),
                ConfirmationStatus.CONFIRMING,
                f"POI {t.identity.obj_id} should be in CONFIRMING status before reset.",
            )

        self.confirmation_manager.reset()
        for t in self.detected_pois:
            self.assertIsNone(
                self.confirmation_manager.get_status(t),
                "POI status should be cleared after reset."
            )

    def test_reset_cancels_worker_blocked_before_registration(self):
        """A not-yet-scheduled review worker is invalid after reset."""
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.is_confirm_on_fail = True
        poi = self.detected_pois[0]

        queued = []
        thread = MagicMock()

        def capture_thread(**kwargs):
            queued.append(kwargs)
            return thread

        self.confirmation_manager._coordinator._thread_factory = capture_thread
        self.confirmation_manager.review([poi])
        self.assertEqual(len(queued), 1)
        self.confirmation_manager.reset()
        queued[0]["target"](*queued[0]["args"])

        self.mock_network.broadcast.assert_not_called()
        self.assertIsNone(self.confirmation_manager.get_status(poi))
        self.assertNotIn(poi.identity.obj_id, self.confirmation_manager._state.pending_events())
        self.assertEqual(self.confirmation_manager._state.active_worker_count, 0)

    def test_reset_after_cancel_check_skips_registration_and_status(self):
        """A worker that PASSES its initial cancel check, then reset() fires,
        must not register a fresh confirmation or recreate status.

        This closes the window the initial-check test does not: reset()'s
        cancel is set under the lock and the worker's registration re-checks it
        under the same lock, so the timeout path (status_on_fail) can never run
        for an abandoned POI. The worker is stalled inside the
        self-assignment broadcast (after the initial check) so reset() lands in
        exactly that post-check window.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = False
        self.mock_nav_args.is_confirm_on_fail = True
        poi = self.detected_pois[0]
        task_id = poi.identity.task_id

        gate = threading.Event()
        entered = threading.Event()

        def gated_broadcast(_poi):
            entered.set()
            gate.wait(1.0)

        with patch.object(
            self.confirmation_manager._assignment,
            "publish",
            side_effect=gated_broadcast,
        ):
            self.confirmation_manager.review([poi])
            self.assertTrue(entered.wait(1.0), "worker never reached broadcast")
            # reset() fires in the post-check window while the worker is stalled.
            self.confirmation_manager.reset()
            gate.set()
            # Allow the worker to run past the broadcast and its (skipped)
            # registration, plus longer than confirm_wait_time_sec so any
            # erroneous timeout status-write would have landed.
            sleep(0.2)

        # No fresh confirmation registered, no status recreated, no confirm
        # REQUEST sent (the self-assignment was patched out).
        self.assertNotIn(task_id, self.confirmation_manager._state.pending_events())
        self.assertIsNone(self.confirmation_manager.get_status(poi))
        self.mock_network.broadcast.assert_not_called()
        self.assertEqual(self.confirmation_manager._state.active_worker_count, 0)

    def test_status_map_uses_task_id_not_local_obj_id(self):
        """POIs with the same local obj_id keep separate statuses by task_id."""
        poi_a = make_detected_poi(
            obj_id=7,
            task_id=101,
            size_class=DetectionSizeClass.S,
            x_error=0,
            y_error=0,
            reference_height_m=0,
            k=0,
            g_data=None,
            uas_att=None,
        )
        poi_b = make_detected_poi(
            obj_id=7,
            task_id=202,
            size_class=DetectionSizeClass.M,
            x_error=0,
            y_error=0,
            reference_height_m=0,
            k=0,
            g_data=None,
            uas_att=None,
        )

        self.confirmation_manager.update_status(poi_a, ConfirmationStatus.CONFIRMED)

        self.assertEqual(self.confirmation_manager.get_status(poi_a), ConfirmationStatus.CONFIRMED)
        self.assertIsNone(self.confirmation_manager.get_status(poi_b))


    def test_auto_confirm_broadcasts_self_assignment(self):
        """
        Auto-confirm with network sends a self-assignment TaskAssignRequestMsg
        for each POI (no confirm request in this path).
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = True

        self.confirmation_manager.review(self.detected_pois)
        sleep(0.1)

        # Only self-assignments — no confirm requests in auto-confirm path
        call_count = self.mock_network.broadcast.call_count
        self.assertEqual(
            call_count,
            len(self.detected_pois),
            f"Expected {len(self.detected_pois)} self-assignment broadcasts, got {call_count}",
        )
        # All broadcasts should be TaskAssignRequestMsg
        for call in self.mock_network.broadcast.call_args_list:
            msg = call[0][0]
            self.assertIsInstance(msg, TaskAssignRequestMsg)
            self.assertEqual(msg.sender_id, 1)
            self.assertEqual(msg.receiver_id, 1)

    def test_self_assignment_uses_debug_location_in_simulation(self):
        """
        In simulation, self-assignment should use t_g_loc_debug (exact) over
        p_t_g_l (georef), matching peer assignment behavior.
        """
        self.confirmation_manager = ConfirmationManager(
            sys_id=1,
            args=self.mock_nav_args,
            logger=self.mock_logger,
            is_simulation=True,
        )
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = True

        poi = self.detected_pois[0]
        poi.replace_geo(replace(
            poi.geo,
            truth_poi_location=Location(lat=55.5, lng=37.7, alt=200),
        ))

        self.confirmation_manager.review([poi])
        sleep(0.1)

        msg = self.mock_network.broadcast.call_args_list[0][0][0]
        self.assertIsInstance(msg, TaskAssignRequestMsg)
        self.assertAlmostEqual(msg.task.location.lat, 55.5)
        self.assertAlmostEqual(msg.task.location.lng, 37.7)
        self.assertAlmostEqual(msg.task.location.alt, 200)

    def test_self_assignment_uses_georef_when_not_simulation(self):
        """
        Outside simulation, self-assignment should use p_t_g_l (georef)
        even when t_g_loc_debug is present.
        """
        self.confirmation_manager.set_network(self.mock_network)
        self.mock_nav_args.is_auto_confirm = True

        poi = self.detected_pois[0]
        poi.replace_geo(replace(
            poi.geo,
            truth_poi_location=Location(lat=55.5, lng=37.7, alt=200),
        ))

        self.confirmation_manager.review([poi])
        sleep(0.1)

        msg = self.mock_network.broadcast.call_args_list[0][0][0]
        self.assertIsInstance(msg, TaskAssignRequestMsg)
        # Should use p_t_g_l (lat=10, lng=20), not debug (55.5, 37.7)
        self.assertAlmostEqual(msg.task.location.lat, 10)
        self.assertAlmostEqual(msg.task.location.lng, 20)

    def test_no_self_assignment_without_network(self):
        """
        No self-assignment broadcast when network is None.
        """
        self.confirmation_manager._network = None
        self.mock_nav_args.is_auto_confirm = False

        self.confirmation_manager.review(self.detected_pois)
        sleep(0.1)

        self.mock_network.broadcast.assert_not_called()

        # POIs should still be confirmed (no-network fallback)
        for poi in self.detected_pois:
            self.assertEqual(
                self.confirmation_manager.get_status(poi),
                ConfirmationStatus.CONFIRMED,
            )


class TestConfirmationImagePropagation(unittest.TestCase):
    """Tests that confirmation image options are wired through.

    These cover behaviour not otherwise testable in isolation: neither
    ``create_confirmation_thumbnail`` (tested elsewhere) nor the runtime
    ``_act_confirm`` gates prove that ``confirmation_manager`` actually forwards
    ``poi.confirmation_degraded`` to the thumbnail builder, or that the
    call site leaves the default POI-centered crop enabled for operator
    recognition.
    """

    def setUp(self):
        import numpy as np
        from unittest.mock import MagicMock

        from navpy.args.nav_args import NavArgs
        from navpy.logger.cache_logger import ILogger
        from navpy.modules.comm.network_abc import NetworkAbc

        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 0.1
        self.mock_args.is_confirm_on_fail = False
        self.mock_network = MagicMock(spec=NetworkAbc)
        self.mock_network.send_image.return_value = 1

        self.confirmation_manager = ConfirmationManager(
            sys_id=1, args=self.mock_args, logger=self.mock_logger,
        )
        self.confirmation_manager.set_network(self.mock_network)

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.poi = make_detected_poi(
            obj_id=303,
            size_class=DetectionSizeClass.S,
            x_error=0, y_error=0,
            reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.poi.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        self.poi.capture_confirmation(ConfirmationEvidence.capture(
            frame,
            (320.0, 240.0, 100.0, 80.0),
            None,
        ))

    def test_degraded_flag_forwarded_to_thumbnail_builder(self):
        """poi.confirmation_degraded=True must reach create_confirmation_thumbnail."""
        from unittest.mock import patch

        self.poi.set_confirmation_degraded(True)
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ) as spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.assertTrue(spy.called)
        _, kwargs = spy.call_args
        self.assertTrue(kwargs.get("degraded"), "degraded kwarg must be True")

    def test_degraded_flag_defaults_false_when_not_set(self):
        """Fresh POIs without the flag still produce a non-degraded image."""
        from unittest.mock import patch

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ) as spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.assertTrue(spy.called)
        _, kwargs = spy.call_args
        self.assertFalse(kwargs.get("degraded"), "degraded kwarg must be False by default")

    def test_confirmation_thumbnail_uses_fallback_delivery_location_crop(self):
        """ConfirmationManager must not disable POI-centered thumbnail crop."""
        from unittest.mock import patch

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ) as spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.assertTrue(spy.called)
        _, kwargs = spy.call_args
        self.assertNotIn(
            "crop_to_poi", kwargs,
            "ConfirmationManager should leave crop_to_poi at the thumbnail default",
        )

    def test_confirmation_artifacts_saved_after_image_send(self):
        """Source frame and sent thumbnail artifacts are saved from the send path."""
        from unittest.mock import patch

        self.mock_logger.log_path = "C:\\logs\\session"
        self.poi.capture_confirmation(ConfirmationEvidence.capture(
            self.poi.confirmation.frame,
            self.poi.confirmation.bbox_cxcywh,
            [(320.0, 240.0, 100.0, 80.0)],
            degraded=True,
        ))

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
        ) as save_spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.mock_network.send_image.assert_called_once_with(303, "b64-image")
        save_spy.assert_called_once()
        kwargs = save_spy.call_args.kwargs
        self.assertEqual(kwargs["log_path"], "C:\\logs\\session")
        self.assertEqual(kwargs["sys_id"], 1)
        self.assertEqual(kwargs["poi_id"], 303)
        self.assertIs(kwargs["source_frame"], self.poi.confirmation.frame)
        self.assertEqual(kwargs["sent_image_b64"], "b64-image")
        self.assertEqual(kwargs["bbox_cxcywh"], self.poi.confirmation.bbox_cxcywh)
        self.assertEqual(kwargs["frame_bboxes"], self.poi.confirmation.frame_bboxes)
        self.assertTrue(kwargs["confirmation_degraded"])

    def test_artifacts_use_the_single_vehicle_sys_id(self):
        """There is no wire-id/log-id split any more: the companion shares its
        aircraft's sysid, so messaging and artifact filenames use the same id."""
        from unittest.mock import patch

        manager = ConfirmationManager(
            sys_id=7,
            args=self.mock_args,
            logger=self.mock_logger,
        )
        manager.set_network(self.mock_network)
        self.mock_logger.log_path = "C:\\logs\\session"

        self.assertEqual(manager._sys_id, 7)
        self.assertFalse(hasattr(manager, "_log_sys_id"))

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
        ) as save_spy:
            manager.review([self.poi])
            sleep(0.2)

        save_spy.assert_called_once()
        self.assertEqual(save_spy.call_args.kwargs["sys_id"], 7)

    def test_confirmation_artifacts_not_saved_without_thumbnail(self):
        """No artifact write is attempted when thumbnail creation fails."""
        from unittest.mock import patch

        self.mock_logger.log_path = "C:\\logs\\session"

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value=None,
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
        ) as save_spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.mock_network.send_image.assert_not_called()
        save_spy.assert_not_called()

    def test_confirmation_artifact_failure_does_not_block_image_send(self):
        """Artifact-save failures warn but do not interrupt confirmation flow."""
        from unittest.mock import patch

        self.mock_logger.log_path = "C:\\logs\\session"

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
            side_effect=OSError("disk full"),
        ):
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.mock_network.send_image.assert_called_once_with(303, "b64-image")
        self.assertEqual(self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED)
        warning_messages = [
            call.args[0] for call in self.mock_logger.warning.call_args_list
        ]
        self.assertTrue(
            any("Failed to save confirmation image artifacts for P303" in msg for msg in warning_messages)
        )

    def test_confirmation_artifacts_saved_when_image_send_raises(self):
        """A network send exception still leaves best-effort debug artifacts."""
        from unittest.mock import patch

        self.mock_logger.log_path = "C:\\logs\\session"
        self.mock_network.send_image.side_effect = OSError("link failed")

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
        ) as save_spy:
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.mock_network.send_image.assert_called_once_with(303, "b64-image")
        save_spy.assert_called_once()
        self.assertEqual(self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED)

    def test_artifact_failure_does_not_mask_image_send_failure(self):
        """If both send and artifact save fail, the send failure remains visible."""
        from unittest.mock import patch

        self.mock_logger.log_path = "C:\\logs\\session"
        self.mock_network.send_image.side_effect = OSError("link failed")

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ), patch(
            "navpy.modules.nav.confirmation_manager.save_confirmation_image_artifacts",
            side_effect=OSError("disk full"),
        ):
            self.confirmation_manager.review([self.poi])
            sleep(0.2)

        self.mock_network.send_image.assert_called_once_with(303, "b64-image")
        self.assertEqual(self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED)

        error_messages = [
            call.args[0] for call in self.mock_logger.error.call_args_list
        ]
        warning_messages = [
            call.args[0] for call in self.mock_logger.warning.call_args_list
        ]
        self.assertTrue(any("link failed" in msg for msg in error_messages))
        self.assertTrue(any("disk full" in msg for msg in warning_messages))


if __name__ == '__main__':
    unittest.main()
