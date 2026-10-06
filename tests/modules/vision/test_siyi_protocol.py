"""Regression tests for the SIYI SDK package."""

import importlib
import unittest
from unittest.mock import Mock, patch

from navpy.modules.vision.peripheral.siyi import SIYISDK
from navpy.modules.vision.peripheral.siyi.siyi_message import COMMAND, MountDirMsg, SIYIMESSAGE


class FakeSocket:
    """Socket test double for send/receive assertions."""

    def __init__(self, packets=None):
        self._packets = list(packets or [])
        self.sent = []
        self.closed = False
        self.timeout = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendto(self, payload, address):
        self.sent.append((payload, address))

    def recvfrom(self, _buffer_size):
        if not self._packets:
            raise OSError("no packet queued")
        packet = self._packets.pop(0)
        return bytes.fromhex(packet), ("127.0.0.1", 37260)

    def close(self):
        self.closed = True


class FakeThread:
    """Thread test double that tracks start/join calls."""

    def __init__(self, start_side_effect=None):
        self._start_side_effect = start_side_effect
        self._alive = False
        self.join_calls = 0
        self.start_calls = 0

    def start(self):
        self.start_calls += 1
        self._alive = True
        if self._start_side_effect is not None:
            self._start_side_effect()

    def is_alive(self):
        return self._alive

    def join(self):
        self.join_calls += 1
        self._alive = False


class TestSiyiPackage(unittest.TestCase):
    def test_package_import_exposes_sdk(self):
        module = importlib.import_module("navpy.modules.vision.peripheral.siyi")
        self.assertIs(module.SIYISDK, SIYISDK)


class TestSiyiMessage(unittest.TestCase):
    def test_sequence_is_encoded_and_wraps(self):
        msg = SIYIMESSAGE()

        first = msg.firmwareVerMsg()
        second = msg.hwIdMsg()

        self.assertEqual(first[10:14], "0100")
        self.assertEqual(second[10:14], "0200")

        msg._seq = 65535
        wrapped = msg.firmwareVerMsg()
        self.assertEqual(wrapped[10:14], "0000")

    def test_mount_direction_constants_match_manual(self):
        self.assertEqual(MountDirMsg.RESERVED, 0)
        self.assertEqual(MountDirMsg.NORMAL, 1)
        self.assertEqual(MountDirMsg.UPSIDE, 2)

    def test_absolute_zoom_rejects_invalid_range(self):
        msg = SIYIMESSAGE()
        self.assertEqual(msg.absoluteZoomMsg(31.0), "")


class TestSiyiSdk(unittest.TestCase):
    def tearDown(self):
        # Individual tests that instantiate SDK objects clean up them explicitly.
        pass

    def test_parse_hardware_id_decodes_ascii_and_camera_type(self):
        sdk = SIYISDK()
        try:
            self.assertTrue(sdk.parseHardwareIDMsg("364231323334353637383930", 7))
            self.assertEqual(sdk.getHardwareID(), "6B1234567890")
            self.assertEqual(sdk.getCameraTypeString(), "ZR10")
        finally:
            sdk.disconnect()

    def test_buffer_callback_resyncs_on_bad_prefix_and_parses_stream_ack(self):
        sdk = SIYISDK()
        packet = SIYIMESSAGE().encodeMsg("01", COMMAND.SET_DATA_STREAM)
        sdk._socket = FakeSocket(packets=["00" + packet])

        try:
            sdk.bufferCallback()
            self.assertEqual(sdk.getDataStreamFeedback(), 1)
            self.assertEqual(sdk._request_data_stream_msg.seq, 1)
        finally:
            sdk.disconnect()

    def test_buffer_callback_parses_absolute_zoom_ack_without_warning(self):
        sdk = SIYISDK()
        packet = SIYIMESSAGE().encodeMsg("", COMMAND.ABSOLUTE_ZOOM)
        sdk._socket = FakeSocket(packets=[packet])

        try:
            with patch.object(sdk._logger, "warning") as warning:
                sdk.bufferCallback()
            self.assertEqual(sdk._request_absolute_zoom_msg.seq, 1)
            warning.assert_not_called()
        finally:
            sdk.disconnect()

    def test_buffer_callback_ignores_socket_close_during_shutdown(self):
        sdk = SIYISDK()
        sdk._stop = True
        closed_socket = Mock()
        closed_socket.recvfrom.side_effect = OSError(10038, "closed")
        sdk._socket = closed_socket

        try:
            with patch.object(sdk._logger, "error") as error:
                sdk.bufferCallback()
            error.assert_not_called()
        finally:
            sdk.disconnect()

    def test_disconnect_does_not_join_current_thread(self):
        sdk = SIYISDK()
        current = FakeThread()
        current._alive = True
        recv_thread = FakeThread()
        recv_thread._alive = True
        conn_thread = FakeThread()
        conn_thread._alive = True
        att_thread = FakeThread()
        att_thread._alive = True

        sdk._socket = FakeSocket()
        sdk._recv_thread = recv_thread
        sdk._conn_thread = current
        sdk._g_info_thread = att_thread
        sdk._g_att_thread = FakeThread()
        sdk._g_att_thread._alive = True

        with patch("navpy.modules.vision.peripheral.siyi.siyi_sdk.threading.current_thread", return_value=current):
            sdk.disconnect()

        self.assertEqual(current.join_calls, 0)
        self.assertEqual(recv_thread.join_calls, 1)
        self.assertEqual(att_thread.join_calls, 1)
        self.assertIsNone(sdk._socket)

    def test_connect_recreates_socket_between_retries(self):
        sdk = SIYISDK()
        created_sockets = []
        attempt = {"count": 0}

        def fake_create_socket():
            fake_socket = FakeSocket()
            created_sockets.append(fake_socket)
            sdk._socket = fake_socket

        def fake_init_threads():
            attempt["count"] += 1
            conn_effect = None
            if attempt["count"] == 2:
                conn_effect = lambda: setattr(sdk, "_connected", True)

            sdk._recv_thread = FakeThread()
            sdk._conn_thread = FakeThread(start_side_effect=conn_effect)
            sdk._g_info_thread = FakeThread()
            sdk._g_att_thread = FakeThread()

        try:
            with patch.object(sdk, "_create_socket", side_effect=fake_create_socket), \
                patch.object(sdk, "_init_threads", side_effect=fake_init_threads), \
                patch("navpy.modules.vision.peripheral.siyi.siyi_sdk.time", side_effect=[0.0, 0.1, 0.2]), \
                patch("navpy.modules.vision.peripheral.siyi.siyi_sdk.sleep", lambda *_: None):
                self.assertTrue(sdk.connect(maxWaitTime=0.05, maxRetries=2))
        finally:
            sdk.disconnect()

        self.assertEqual(len(created_sockets), 2)
        self.assertEqual(len(created_sockets[0].sent), 0)
        self.assertGreaterEqual(len(created_sockets[1].sent), 2)

    def test_request_absolute_zoom_invalid_value_returns_false_without_sending(self):
        sdk = SIYISDK()
        fake_socket = FakeSocket()
        sdk._socket = fake_socket

        try:
            self.assertFalse(sdk.requestAbsoluteZoom(31.0))
            self.assertEqual(fake_socket.sent, [])
        finally:
            sdk.disconnect()

    def test_current_zoom_sample_exposes_seq_and_level(self):
        sdk = SIYISDK()
        try:
            with patch(
                "navpy.modules.vision.peripheral.siyi.siyi_sdk.monotonic",
                return_value=12.5,
            ):
                self.assertTrue(sdk.parseCurrentZoomLevelMsg("0305", 42))
            self.assertEqual(sdk.getCurrentZoomLevelSample(), (42, 3.5, 12.5))
            self.assertFalse(sdk.parseCurrentZoomLevelMsg("03", 43))
            self.assertEqual(sdk.getCurrentZoomLevelSample(), (42, 3.5, 12.5))
        finally:
            sdk.disconnect()

    def test_attitude_sample_exposes_atomic_receipt_time(self):
        sdk = SIYISDK()
        try:
            payload = "640038ff3200000000000000"
            with patch(
                "navpy.modules.vision.peripheral.siyi.siyi_sdk.time",
                return_value=123.45,
            ):
                self.assertTrue(sdk.parseAttitudeMsg(payload, 9))

            self.assertEqual(
                sdk.getAttitudeSample(),
                (9, 123.45, 10.0, -20.0, 5.0),
            )
        finally:
            sdk.disconnect()

    def test_set_gimbal_rotation_accepts_manual_yaw_range(self):
        sdk = SIYISDK()
        sdk.requestGimbalAttitude = Mock()
        sdk.requestGimbalSpeed = Mock()
        sdk._last_att_seq = 0
        sdk._att_msg.seq = 1
        sdk._att_msg.yaw = 90.0
        sdk._att_msg.pitch = 0.0

        try:
            sdk.setGimbalRotation(90.0, 0.0)
            sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
        finally:
            sdk.disconnect()


if __name__ == "__main__":
    unittest.main()
