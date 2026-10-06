"""Focused tests for OpenCV source routing."""

import threading
import unittest
from unittest.mock import Mock, patch

import cv2

from navpy.modules.vision.frame_capture_opencv import (
    OpenCvFrameCapture,
    open_jetson_capture,
    open_source_capture,
    open_stream_capture,
    open_webcam_capture,
)

from tests.modules.vision.test_frame_provider import _Logger


class TestOpenCvSourceRouting(unittest.TestCase):
    def test_jetson_metadata_failure_releases_capture(self):
        capture = Mock()
        capture.isOpened.return_value = True
        capture.get.side_effect = cv2.error("metadata failed")
        with patch(
            "navpy.modules.vision.frame_capture_opencv.cv2.VideoCapture",
            return_value=capture,
        ):
            with self.assertRaisesRegex(cv2.error, "metadata failed"):
                open_jetson_capture("rtsp://camera/stream", _Logger())

        capture.release.assert_called_once_with()

    def test_webcam_read_failure_releases_capture(self):
        capture = Mock()
        capture.isOpened.return_value = True
        capture.read.side_effect = cv2.error("read failed")
        with patch(
            "navpy.modules.vision.frame_capture_opencv.cv2.VideoCapture",
            return_value=capture,
        ):
            with self.assertRaisesRegex(cv2.error, "read failed"):
                open_webcam_capture(0, _Logger())

        capture.release.assert_called_once_with()

    def test_stream_fallback_releases_failed_primary_capture(self):
        primary = Mock()
        primary.isOpened.return_value = False
        fallback = Mock()
        fallback.isOpened.return_value = True
        with patch(
            "navpy.modules.vision.frame_capture_opencv.cv2.VideoCapture",
            side_effect=[primary, fallback],
        ):
            result = open_stream_capture("camera.mp4")

        self.assertIs(result, fallback)
        primary.release.assert_called_once_with()
        fallback.release.assert_not_called()

    def test_release_waits_for_inflight_capture_access(self):
        grab_entered = threading.Event()
        release_grab = threading.Event()
        released = threading.Event()
        running = {"value": True}
        capture = Mock()

        def blocking_grab():
            grab_entered.set()
            release_grab.wait(timeout=2.0)
            return False

        capture.grab.side_effect = blocking_grab
        capture.release.side_effect = released.set
        backend = OpenCvFrameCapture(capture)
        worker = threading.Thread(
            target=backend.run,
            args=(Mock(), lambda: running["value"]),
        )
        worker.start()
        self.assertTrue(grab_entered.wait(timeout=1.0))
        running["value"] = False
        interrupter = threading.Thread(target=backend.request_stop)
        interrupter.start()

        self.assertFalse(released.wait(timeout=0.05))
        release_grab.set()
        worker.join(timeout=2.0)
        interrupter.join(timeout=2.0)

        self.assertFalse(worker.is_alive())
        self.assertFalse(interrupter.is_alive())
        capture.release.assert_called_once_with()

    def test_release_failure_is_reported_and_remains_retryable(self):
        capture = Mock()
        capture.release.side_effect = [cv2.error("release failed"), None]
        backend = OpenCvFrameCapture(capture)

        self.assertFalse(backend.request_stop())
        backend.close()

        self.assertEqual(capture.release.call_count, 2)

    def test_none_source_has_no_capture(self):
        self.assertIsNone(open_source_capture(None, _Logger()))

    def test_integer_source_uses_webcam_opener(self):
        capture = Mock()
        with patch(
            "navpy.modules.vision.frame_capture_opencv.open_webcam_capture",
            return_value=capture,
        ) as open_webcam:
            result = open_source_capture(3, _Logger())
        self.assertIs(result, capture)
        self.assertEqual(open_webcam.call_args.args[0], 3)

    def test_string_source_uses_stream_opener(self):
        capture = Mock()
        with patch(
            "navpy.modules.vision.frame_capture_opencv.open_stream_capture",
            return_value=capture,
        ) as open_stream:
            result = open_source_capture("camera.mp4", _Logger())
        self.assertIs(result, capture)
        open_stream.assert_called_once_with("camera.mp4")


if __name__ == "__main__":
    unittest.main()
