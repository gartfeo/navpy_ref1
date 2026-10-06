"""
Tests for the on-demand thumbnail decouple (D-12, plan 02-04 Task 2): the
confirm-request resend loop never re-sends the image, and a
SwarmRequestMsg(RESOURCE, THUMBNAIL) addressed to a still-pending target
re-sends the chunked thumbnail without touching the request resend loop or
the confirm_wait_time_sec window (D-11). Sequence-gap NACK / RETRANSMIT and
FORCE_CONFIRM auto-fire are explicitly out of scope here (D-27 deferral;
FORCE_CONFIRM belongs to 02-05).
"""
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import TaskConfirmRequestMsg
from navpy.modules.comm.messages.swarm_request_msg import (
    SwarmRequestMsg, REQUEST_TYPE_RESOURCE, REQUEST_TYPE_RETRANSMIT,
    REQUEST_TYPE_FORCE_CONFIRM, SUBJECT_TYPE_THUMBNAIL,
)
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.location import Location
from navpy.modules.nav import confirmation_manager as tm_module
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from navpy.modules.vision.models.detection_components import ConfirmationEvidence
from tests.detection_factory import make_detected_target


class ConfirmationManagerThumbnailRequestTests(unittest.TestCase):
    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 0.6
        self.mock_args.is_confirm_on_fail = False

        self.mock_network = MagicMock(spec=NetworkAbc)
        self.mock_network.send_image.return_value = 1

        def _fake_broadcast(message):
            # Mirror NetworkAbc.broadcast()'s real auto-fill-meta behavior.
            if not message.has_meta():
                message.set_meta_from_provider()
        self.mock_network.broadcast.side_effect = _fake_broadcast

        self.confirmation_manager = ConfirmationManager(sys_id=1, args=self.mock_args, logger=self.mock_logger)
        self.confirmation_manager.set_network(self.mock_network)

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.target = make_detected_target(
            obj_id=501, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.target.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        self.target.capture_confirmation(ConfirmationEvidence.capture(
            frame,
            (320.0, 240.0, 100.0, 80.0),
            None,
        ))

    def _confirm_request_calls(self):
        """Broadcast calls that are TaskConfirmRequestMsg (excludes self-assign)."""
        return [
            c[0][0] for c in self.mock_network.broadcast.call_args_list
            if isinstance(c[0][0], TaskConfirmRequestMsg)
        ]

    @staticmethod
    def _resource_request(task_id, receiver_id=1, subject_type=SUBJECT_TYPE_THUMBNAIL,
                           request_type=REQUEST_TYPE_RESOURCE):
        return SwarmRequestMsg(
            sender_id=0, receiver_id=receiver_id,
            request_type=request_type, subject_type=subject_type, subject_id=task_id,
        )

    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.1)
    def test_resend_loop_never_resends_image(self):
        """The retry loop re-broadcasts only the small request -- send_image
        from the request path is called at most once even with K resends."""
        self.mock_args.confirm_wait_time_sec = 0.5

        # Capture the review worker thread the coordinator spawns and join it in
        # cleanup (fencing the round via reset so it exits promptly), so it never
        # outlives the test -- this test's 0.45s sleep is shorter than the 0.5s
        # confirm window, so the worker is still running when the body returns.
        started: list[threading.Thread] = []
        real_factory = self.confirmation_manager._coordinator._thread_factory

        def _recording_factory(**kwargs):
            thread = real_factory(**kwargs)
            started.append(thread)
            return thread

        self.confirmation_manager._coordinator._thread_factory = _recording_factory

        def _drain_review_threads():
            self.confirmation_manager.reset()  # signals the response event so the worker exits
            survivors = []
            for thread in started:
                thread.join(timeout=2.0)
                if thread.is_alive():
                    survivors.append(thread)
            self.assertFalse(
                survivors, "confirmation review worker(s) did not stop after reset",
            )

        self.addCleanup(_drain_review_threads)

        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.45)

        confirm_calls = self._confirm_request_calls()
        self.assertGreaterEqual(len(confirm_calls), 2, "expected at least one resend")
        self.mock_network.send_image.assert_called_once_with(501, "b64-image")

    def test_swarm_request_resource_for_pending_target_resends_thumbnail(self):
        """SWARM_REQUEST(RESOURCE, THUMBNAIL) for a still-pending task_id
        triggers exactly one additional chunked send_image, without
        re-broadcasting the request."""
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.05)  # initial send has landed, still well inside the window

            confirm_count_before = len(self._confirm_request_calls())
            self.confirmation_manager.on_message(self._resource_request(501))

            self.assertEqual(self.mock_network.send_image.call_count, 2)
            self.assertEqual(
                len(self._confirm_request_calls()), confirm_count_before,
                "on-demand thumbnail fetch must not re-broadcast the request",
            )
            time.sleep(0.6)  # let the worker exit cleanly before teardown

    def test_swarm_request_resource_for_unknown_task_id_is_noop(self):
        """A RESOURCE/THUMBNAIL request for a task_id with no pending
        confirmation is a no-op -- no send_image, no error."""
        self.confirmation_manager.on_message(self._resource_request(999999))
        self.mock_network.send_image.assert_not_called()

    def test_swarm_request_resource_for_finished_target_is_noop(self):
        """Once a confirmation resolves, a late RESOURCE/THUMBNAIL request
        for that task_id is a no-op."""
        self.mock_args.confirm_wait_time_sec = 0.05
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.3)  # window elapses, confirmation resolves

        self.assertEqual(self.mock_network.send_image.call_count, 1)
        self.confirmation_manager.on_message(self._resource_request(501))
        self.assertEqual(
            self.mock_network.send_image.call_count, 1, "no additional send for a finished target",
        )

    def test_resource_thumbnail_does_not_start_after_round_reset(self):
        """Reset between lookup and media must fence the stale resource send."""
        worker = self.confirmation_manager._state.start_review(self.target, threading.Event)
        confirmation = self.confirmation_manager._state.begin_round(
            self.target,
            worker,
            threading.Event,
        )
        self.assertIsNotNone(confirmation)
        original_pending_target = self.confirmation_manager._state.pending_target

        def pending_target_then_reset(target_id):
            target = original_pending_target(target_id)
            self.confirmation_manager.reset()
            return target

        with patch.object(
            self.confirmation_manager._state,
            "pending_target",
            side_effect=pending_target_then_reset,
        ), patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.on_message(self._resource_request(501))

        self.mock_network.send_image.assert_not_called()

    def test_resource_media_programmer_typeerror_propagates(self):
        request = self._resource_request(501)
        with patch.object(
            self.confirmation_manager._state,
            "pending_target",
            return_value=self.target,
        ), patch.object(
            self.confirmation_manager._state,
            "send_pending_if_current",
            side_effect=TypeError("bad media callback"),
        ):
            with self.assertRaisesRegex(TypeError, "bad media callback"):
                self.confirmation_manager.on_message(request)

    def test_resource_media_oserror_is_contained(self):
        request = self._resource_request(501)
        with patch.object(
            self.confirmation_manager._state,
            "pending_target",
            return_value=self.target,
        ), patch.object(
            self.confirmation_manager._state,
            "send_pending_if_current",
            side_effect=OSError("image link down"),
        ):
            self.confirmation_manager.on_message(request)

        self.mock_logger.error.assert_called_once()
        self.assertIn("image link down", self.mock_logger.error.call_args.args[0])

    def test_retransmit_request_type_is_not_actioned(self):
        """D-27: no sequence-gap/RETRANSMIT auto-fire is added -- a
        REQUEST_TYPE_RETRANSMIT message must not trigger a thumbnail resend."""
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.05)
            self.confirmation_manager.on_message(
                self._resource_request(501, request_type=REQUEST_TYPE_RETRANSMIT))
            self.assertEqual(self.mock_network.send_image.call_count, 1)
            time.sleep(0.6)

    def test_force_confirm_request_type_is_not_actioned(self):
        """A REQUEST_TYPE_FORCE_CONFIRM message must not trigger a thumbnail
        resend here (out of scope; 02-05 owns FORCE_CONFIRM)."""
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.05)
            self.confirmation_manager.on_message(
                self._resource_request(501, request_type=REQUEST_TYPE_FORCE_CONFIRM))
            self.assertEqual(self.mock_network.send_image.call_count, 1)
            time.sleep(0.6)

    def test_swarm_request_addressed_to_different_drone_is_ignored(self):
        """receiver_id != this drone's sys_id -- on_message's existing guard
        drops it before it reaches the SWARM_REQUEST handler."""
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.05)
            self.confirmation_manager.on_message(self._resource_request(501, receiver_id=99))
            self.assertEqual(self.mock_network.send_image.call_count, 1)
            time.sleep(0.6)

    def test_thumbnail_refetch_does_not_extend_window(self):
        """D-11: a thumbnail re-fetch mid-window must not restart/extend the
        single confirm_wait_time_sec window."""
        self.mock_args.confirm_wait_time_sec = 0.3
        start = time.monotonic()
        with patch(
            "navpy.modules.nav.confirmation_manager.create_confirmation_thumbnail",
            return_value="b64-image",
        ):
            self.confirmation_manager.review([self.target])
            time.sleep(0.05)
            self.confirmation_manager.on_message(self._resource_request(501))

            deadline = start + self.mock_args.confirm_wait_time_sec + 0.4  # generous slack
            while time.monotonic() < deadline:
                if self.confirmation_manager.get_status(self.target) in (
                        ConfirmationStatus.CONFIRMED, ConfirmationStatus.REJECTED, ConfirmationStatus.TIMEOUT_REJECTED):
                    break
                time.sleep(0.01)

        elapsed = time.monotonic() - start
        self.assertLess(
            elapsed, self.mock_args.confirm_wait_time_sec + 0.3,
            "thumbnail re-fetch must not restart/extend the confirm_wait_time_sec window",
        )


if __name__ == "__main__":
    unittest.main()
