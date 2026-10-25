"""A confirmation response does not wait for the rest of the image (todo 110).

The round's outbound fence covers each image send unit (header, chunk), not
the whole image: a response from another thread resolves between two chunks,
and the chunks still to come are not sent.
"""
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
from navpy.core.link.task_confirmation_msg import TaskConfirmRequestMsg, TaskConfirmResponseMsg
from navpy.core.ports.detect_data import DetectionSizeClass
from navpy.core.ports.detection_components import ConfirmationEvidence
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.review import confirmation_manager as tm_module
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from tests.detection_factory import make_detected_poi

CHUNKS = 20
CHUNK_SIZE = 10
RESPOND_AT_CHUNK = 2


class _ChunkNetwork(NetworkAbc):
    def __init__(self, on_chunk):
        super().__init__(1, None, MagicMock(), image_chunk_size=CHUNK_SIZE,
                         inter_chunk_delay_ms=20.0)
        self.on_chunk = on_chunk
        self.requests = []
        self.chunks = []

    def broadcast(self, message):
        if not message.has_meta():
            message.set_meta_from_provider()
        if isinstance(message, TaskConfirmRequestMsg):
            self.requests.append(message)

    def broadcast_data(self, data):
        pass

    def close(self):
        pass

    def _send_image_header(self, poi_id, total_size, num_chunks):
        pass

    def _send_image_chunk(self, poi_id, sequence, data):
        self.chunks.append(sequence)
        self.on_chunk(sequence)


class ConfirmationImageFenceTests(unittest.TestCase):
    def setUp(self):
        args = MagicMock(spec=NavArgs)
        args.is_auto_confirm = False
        args.confirm_wait_time_sec = 5.0
        args.is_confirm_on_fail = False
        self.manager = ConfirmationManager(sys_id=1, args=args, logger=MagicMock(spec=ILogger))
        self.poi = make_detected_poi(
            obj_id=601, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.poi.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        self.poi.capture_confirmation(ConfirmationEvidence.capture(
            np.zeros((48, 64, 3), dtype=np.uint8), (32.0, 24.0, 10.0, 8.0), None,
        ))

    def _response(self, request):
        meta = request.meta
        return TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=self.poi.identity.obj_id,
            is_confirmed=True,
            meta=MsgMeta(meta.boot_id, meta.msg_seq, meta.time_ms + 1, meta.ttl_ms),
        )

    def test_response_mid_image_resolves_without_waiting_and_stops_the_image(self):
        resolved_after = {}
        responder_done = threading.Event()

        def respond():
            self.manager.on_message(self._response(network.requests[0]))
            resolved_after["chunks"] = len(network.chunks)
            responder_done.set()

        def on_chunk(sequence):
            if sequence == RESPOND_AT_CHUNK:
                threading.Thread(target=respond, daemon=True).start()

        network = _ChunkNetwork(on_chunk)
        self.manager.set_network(network)
        image_b64 = base64.b64encode(bytes(CHUNKS * CHUNK_SIZE)).decode()
        with patch.object(tm_module, "create_confirmation_thumbnail", return_value=image_b64):
            self.manager.review([self.poi])
            self.assertTrue(responder_done.wait(5.0), "response never resolved")
            deadline = time.monotonic() + 5.0
            while (self.manager.get_status(self.poi) is not ConfirmationStatus.CONFIRMED
                   and time.monotonic() < deadline):
                time.sleep(0.01)

        self.assertIs(self.manager.get_status(self.poi), ConfirmationStatus.CONFIRMED)
        self.assertLess(resolved_after["chunks"], CHUNKS, "response waited for the whole image")
        time.sleep(0.1)  # the sender would have sent the rest by now
        self.assertLess(len(network.chunks), CHUNKS, "image kept sending after the response")


class SendImageRunUnitTests(unittest.TestCase):
    def test_refused_unit_stops_the_transfer(self):
        network = _ChunkNetwork(lambda _seq: None)
        network._inter_chunk_delay_sec = 0.0
        allowed = iter([True, True, True, False])  # header, chunk 0, chunk 1, refuse

        def run_unit(unit):
            if not next(allowed):
                return False
            unit()
            return True

        sent = network.send_image(5, base64.b64encode(bytes(CHUNKS * CHUNK_SIZE)).decode(),
                                  run_unit=run_unit)
        self.assertEqual(sent, 2)
        self.assertEqual(network.chunks, [0, 1])


if __name__ == "__main__":
    unittest.main()
