"""A paced confirmation image never holds up the round (todo 110, ``radio_budget.md`` round 7)."""

import base64
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.location import Location
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_request_msg import (
    REQUEST_TYPE_RESOURCE,
    SUBJECT_TYPE_THUMBNAIL,
    SwarmRequestMsg,
)
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg, TaskConfirmResponseMsg
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.ports.detect_data import DetectionSizeClass
from navpy.core.ports.detection_components import ConfirmationEvidence
from navpy.core.review import confirmation_manager as tm_module
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.plugins.radio.mavlink import network_mavlink
from navpy.plugins.radio.mavlink.image_pacer import wire_bytes
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink
from tests.detection_factory import make_detected_poi

CHUNK_S = 0.05
CHUNKS = 20  # 1 s of image
IMAGE_B64 = base64.b64encode(bytes(253 * CHUNKS)).decode()


class _Subscription:
    def cancel(self) -> None:
        pass


class _Vehicle:
    def __init__(self) -> None:
        self.sent = []

    def on_message(self, name, callback):
        return _Subscription()

    def send_mavlink_message(self, message, *, source_component=None) -> None:
        self.sent.append((time.monotonic(), message))

    def chunks(self):
        return [m for _t, m in self.sent if type(m).__name__ == "MAVLink_encapsulated_data_message"]

    def requests(self):
        return [m for _t, m in self.sent if type(m).__name__ == "MAVLink_navlink_message"]


class _RoundHarness:
    def setUp(self):
        chunk = network_mavlink.MAVLink_encapsulated_data_message(seqnr=0, data=[0] * 253)
        patcher = patch.object(network_mavlink, "IMAGE_BYTES_PER_S", wire_bytes(chunk) / CHUNK_S)
        patcher.start()
        self.addCleanup(patcher.stop)
        thumbnail = patch(
            "navpy.core.review.confirmation_manager.create_confirmation_thumbnail",
            return_value=IMAGE_B64,
        )
        thumbnail.start()
        self.addCleanup(thumbnail.stop)
        args = MagicMock(spec=NavArgs)
        args.is_auto_confirm = False
        args.confirm_wait_time_sec = 3.0
        args.is_confirm_on_fail = False
        self.vehicle = _Vehicle()
        self.network = NetworkMavlink(1, self.vehicle, MagicMock(spec=ILogger))
        self.addCleanup(self.network.close)
        self.requests: list[TaskConfirmRequestMsg] = []
        broadcast = self.network.broadcast

        def recording_broadcast(message):
            broadcast(message)
            if isinstance(message, TaskConfirmRequestMsg):
                self.requests.append(message)

        self.network.broadcast = recording_broadcast
        self.manager = ConfirmationManager(sys_id=1, args=args, logger=MagicMock(spec=ILogger))
        self.manager.set_network(self.network)
        self.poi = make_detected_poi(
            obj_id=501, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.poi.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        self.poi.capture_confirmation(ConfirmationEvidence.capture(
            np.zeros((480, 640, 3), dtype=np.uint8), (320.0, 240.0, 100.0, 80.0), None,
        ))

    def _wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return bool(predicate())

    def _respond(self):
        meta = self.requests[0].meta
        self.manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1, task_id=501, is_confirmed=True,
            meta=MsgMeta(meta.boot_id, meta.msg_seq, meta.time_ms + 1, meta.ttl_ms),
        ))

    def test_a_response_mid_image_resolves_at_once_and_ends_the_image(self):
        self.manager.review([self.poi])
        self.assertTrue(self._wait(lambda: len(self.vehicle.chunks()) >= 3))
        started = time.monotonic()
        self._respond()
        self.assertIs(self.manager.get_status(self.poi), ConfirmationStatus.CONFIRMED)
        self.assertLess(time.monotonic() - started, CHUNK_S * 2)
        sent = len(self.vehicle.chunks())
        time.sleep(CHUNK_S * 4)
        self.assertEqual(len(self.vehicle.chunks()), sent, "a frame went out after the response")
        self.assertLess(len(self.vehicle.chunks()), CHUNKS)



class PacedImageRoundTests(_RoundHarness, unittest.TestCase):
    @patch.object(tm_module, "RESEND_INTERVAL_S", 0.2)
    def test_request_resends_keep_their_cadence_while_the_image_is_sent(self):
        self.manager.review([self.poi])
        self.assertTrue(self._wait(lambda: len(self.vehicle.chunks()) == CHUNKS))
        # The image took 1 s; the request went out at 0 and every 0.2 s during it.
        self.assertGreaterEqual(len(self.requests), 4)
        self._respond()

    def test_a_thumbnail_request_returns_at_once_and_the_image_follows(self):
        self.manager.review([self.poi])
        self.assertTrue(self._wait(lambda: len(self.vehicle.chunks()) == CHUNKS))
        meta = self.requests[0].meta
        request = SwarmRequestMsg(
            sender_id=0, receiver_id=1, request_type=REQUEST_TYPE_RESOURCE,
            subject_type=SUBJECT_TYPE_THUMBNAIL, subject_id=501,
        )
        request.set_meta(MsgMeta(meta.boot_id, meta.msg_seq, meta.time_ms + 1, meta.ttl_ms))
        started = time.monotonic()
        self.manager.on_message(request)
        self.assertLess(time.monotonic() - started, CHUNK_S * 2, "the handler waited for the image")
        self.assertTrue(self._wait(lambda: len(self.vehicle.chunks()) == 2 * CHUNKS))
        self._respond()


class _SyncNetwork(NetworkAbc):
    """The base synchronous send_image (WiFi), a chunk every CHUNK_S."""

    def __init__(self) -> None:
        super().__init__(1, MagicMock(), MagicMock(spec=ILogger),
                         image_chunk_size=253, inter_chunk_delay_ms=CHUNK_S * 1000)
        self.chunks: list[int] = []
        self.requests: list[TaskConfirmRequestMsg] = []

    def broadcast(self, message) -> None:
        if isinstance(message, TaskConfirmRequestMsg):
            self.requests.append(message)

    def broadcast_data(self, data) -> None:
        pass

    def close(self) -> None:
        pass

    def _send_image_header(self, poi_id, total_size, num_chunks) -> None:
        pass

    def _send_image_chunk(self, poi_id, sequence, data) -> None:
        self.chunks.append(sequence)


class SynchronousImageRoundTests(_RoundHarness, unittest.TestCase):
    """A response mid-image, with the image sent on the round's own worker."""

    def setUp(self):
        super().setUp()
        self.network = _SyncNetwork()
        self.requests = self.network.requests
        self.vehicle.chunks = lambda: self.network.chunks
        self.manager.set_network(self.network)


class ConfirmationMediaHandOverTests(unittest.TestCase):
    """Handing the image to the link is fenced, so a stale round cannot replace a current image."""

    def _media(self, network):
        from navpy.core.review.confirmation_media import ConfirmationMedia

        return ConfirmationMedia(
            sys_id=1, network=lambda: network, logger=MagicMock(spec=ILogger),
            thumbnail_builder=lambda **kwargs: IMAGE_B64, artifact_saver=lambda **kwargs: None,
        )

    def _poi(self):
        poi = make_detected_poi(
            obj_id=7, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0, g_data=None, uas_att=None,
        )
        poi.capture_confirmation(ConfirmationEvidence.capture(
            np.zeros((48, 64, 3), dtype=np.uint8), (32.0, 24.0, 10.0, 8.0), None,
        ))
        return poi

    def _queueing_network(self):
        vehicle = _Vehicle()
        network = NetworkMavlink(1, vehicle, MagicMock(spec=ILogger))
        self.addCleanup(network.close)
        return network, vehicle

    def test_a_refused_hand_over_queues_nothing(self):
        network, vehicle = self._queueing_network()
        self.assertFalse(self._media(network).send(self._poi(), 7, lambda send: False))
        time.sleep(0.05)
        self.assertEqual(vehicle.sent, [])

    def test_a_refused_hand_over_leaves_the_link_queue_empty(self):
        network, vehicle = self._queueing_network()
        self.assertEqual(network.send_image(7, IMAGE_B64, lambda unit: False), CHUNKS)
        time.sleep(0.05)
        self.assertEqual(vehicle.sent, [])

    def test_a_refused_hand_over_does_not_replace_the_waiting_image_of_its_poi(self):
        network, vehicle = self._queueing_network()

        def image(byte: int) -> str:
            return base64.b64encode(bytes([byte]) * 253).decode()

        network.send_image(1, image(1), lambda unit: (unit(), True)[1])  # being sent
        network.send_image(7, image(2), lambda unit: (unit(), True)[1])  # waits behind it
        network.send_image(7, image(3), lambda unit: False)  # a stale round's image
        self.assertTrue(self._wait_for(lambda: len(vehicle.chunks()) == 2))
        self.assertEqual([chunk.data[0] for chunk in vehicle.chunks()], [1, 2])

    @staticmethod
    def _wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            time.sleep(0.005)
        return predicate()

    def test_a_refusal_during_the_send_reports_the_round_gone(self):
        answers = iter([True, True, False])  # the round check, the header, then a chunk

        def run_unit(unit):
            if next(answers):
                unit()
                return True
            return False

        self.assertFalse(self._media(_SyncNetwork()).send(self._poi(), 7, run_unit))

    def test_the_fence_is_held_for_one_step_at_a_time(self):
        network, _vehicle = self._queueing_network()
        local = threading.local()
        depths = []

        def run_unit(send):
            local.depth = getattr(local, "depth", 0) + 1
            depths.append(local.depth)
            try:
                send()
            finally:
                local.depth -= 1
            return True

        self.assertTrue(self._media(network).send(self._poi(), 7, run_unit))
        deadline = time.monotonic() + 5
        while len(depths) < 3 and time.monotonic() < deadline:  # the hand-over, then frames
            time.sleep(0.005)
        self.assertGreaterEqual(len(depths), 3)
        self.assertEqual(set(depths), {1}, "the fence was taken inside itself")

    def test_a_refusal_on_the_link_s_sender_thread_is_logged(self):
        network, _vehicle = self._queueing_network()
        logger = MagicMock(spec=ILogger)
        media = self._media(network)
        media._logger = logger
        answers = iter([True, True, True, False])  # the round check, hand-over, header, a chunk

        def run_unit(unit):
            if next(answers, False):
                unit()
                return True
            return False

        self.assertTrue(media.send(self._poi(), 7, run_unit))  # handed over: send() is done

        def stopped():
            return [c.args[0] for c in logger.info.call_args_list if "image send stopped" in c.args[0]]

        self.assertTrue(self._wait_for(lambda: stopped()))
        self.assertEqual(stopped(), ["P7 image send stopped: confirmation resolved."])


IMAGE_A = base64.b64encode(bytes([1]) * 253).decode()
IMAGE_B = base64.b64encode(bytes([2]) * 253).decode()


def _admit(unit):
    unit()
    return True


class ConfirmationArtifactsPerRoundTests(unittest.TestCase):
    """A round keeps one set of artifacts; a resent copy of its image adds none."""

    def setUp(self):
        self.saver = MagicMock()  # returns the written artifacts
        self.images = iter([IMAGE_A, IMAGE_A])

    def _media(self, network=None):
        from navpy.core.review.confirmation_media import ConfirmationMedia

        logger = MagicMock(spec=ILogger)
        logger.log_path = "session-logs"
        return ConfirmationMedia(
            sys_id=1, network=lambda: network or _SyncNetwork(), logger=logger,
            thumbnail_builder=lambda **kwargs: next(self.images), artifact_saver=self.saver,
        )

    def _sent_images(self):
        return [c.kwargs["sent_image_b64"] for c in self.saver.call_args_list]

    def test_a_resent_copy_saves_nothing(self):
        media, poi = self._media(), ConfirmationMediaHandOverTests._poi(None)
        media.send(poi, 7, _admit)
        media.send(poi, 7, _admit, resend=True)
        self.assertEqual(self._sent_images(), [IMAGE_A])

    def test_a_resend_with_a_new_image_saves_it(self):
        media, poi = self._media(), ConfirmationMediaHandOverTests._poi(None)
        self.images = iter([IMAGE_A, IMAGE_B])
        media.send(poi, 7, _admit)
        media.send(poi, 7, _admit, resend=True)
        self.assertEqual(self._sent_images(), [IMAGE_A, IMAGE_B])

    def test_a_failed_save_lets_a_resend_save(self):
        media, poi = self._media(), ConfirmationMediaHandOverTests._poi(None)
        self.saver.side_effect = [OSError("disk full"), MagicMock()]
        media.send(poi, 7, _admit)
        media.send(poi, 7, _admit, resend=True)
        self.assertEqual(self._sent_images(), [IMAGE_A, IMAGE_A])

    def test_a_resend_while_the_first_send_is_sending_saves_nothing(self):
        poi = ConfirmationMediaHandOverTests._poi(None)

        class _ResendDuringSend:
            resent = False

            def send_image(self, poi_id, image_b64, run_unit=None):
                if not self.resent:  # the GCS's request, on the inbound thread
                    self.resent = True
                    inbound = threading.Thread(target=lambda: media.send(poi, poi_id, _admit, resend=True))
                    inbound.start()
                    inbound.join()
                return 1

        media = self._media(_ResendDuringSend())
        media.send(poi, 7, _admit)
        self.assertEqual(self._sent_images(), [IMAGE_A])

    def test_every_round_saves_even_the_same_image(self):
        media, poi = self._media(), ConfirmationMediaHandOverTests._poi(None)
        media.send(poi, 7, _admit)
        media.send(poi, 7, _admit)
        self.assertEqual(self._sent_images(), [IMAGE_A, IMAGE_A])


if __name__ == "__main__":
    unittest.main()
