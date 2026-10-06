"""Public-contract tests for FrameProvider."""

import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from navpy.modules.vision.frame_provider import FrameProvider


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, msg, *args):
        self.messages.append(("info", msg % args if args else msg))

    def warning(self, msg, *args):
        self.messages.append(("warning", msg % args if args else msg))

    def error(self, msg, *args):
        self.messages.append(("error", msg % args if args else msg))

    def debug(self, msg, *args):
        pass


class TestFrameProviderPushMode(unittest.TestCase):
    def test_get_frame_returns_none_initially(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        frame, width, height = provider.get_frame()
        self.assertIsNone(frame)
        self.assertEqual((width, height), (0, 0))
        provider.stop()

    def test_push_frame_and_get_frame(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        provider.push_frame(image)
        frame, width, height = provider.get_frame()
        self.assertIsNotNone(frame)
        self.assertEqual((width, height), (640, 480))
        np.testing.assert_array_equal(frame, image)
        provider.stop()

    def test_push_none_clears_frame(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        provider.push_frame(np.zeros((480, 640, 3), dtype=np.uint8))
        provider.push_frame(None)
        frame, width, height = provider.get_frame()
        self.assertIsNone(frame)
        self.assertEqual((width, height), (0, 0))
        provider.stop()

    def test_latest_frame_wins(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        first = np.full((100, 200, 3), 50, dtype=np.uint8)
        latest = np.full((300, 400, 3), 100, dtype=np.uint8)
        provider.push_frame(first)
        provider.push_frame(latest)
        frame, width, height = provider.get_frame()
        self.assertEqual((width, height), (400, 300))
        np.testing.assert_array_equal(frame, latest)
        provider.stop()

    def test_push_mode_does_not_start_a_capture_backend(self):
        logger = _Logger()
        provider = FrameProvider(source=None, logger=logger)
        provider.start()
        self.assertIn(
            ("info", "FrameProvider: push mode (no capture thread)"),
            logger.messages,
        )
        provider.stop()

    def test_get_frame_state_includes_sequence(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        _, _, _, sequence_before = provider.get_frame_state()
        provider.push_frame(np.zeros((20, 30, 3), dtype=np.uint8))
        frame, width, height, sequence_after = provider.get_frame_state()
        self.assertIsNotNone(frame)
        self.assertEqual((width, height), (30, 20))
        self.assertGreater(sequence_after, sequence_before)
        self.assertEqual(sequence_after, provider.get_frame_seq())
        provider.stop()

    def test_snapshot_keeps_time_sequence_and_owned_pixels(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        image = np.full((20, 30, 3), 17, dtype=np.uint8)
        with patch(
            "navpy.modules.vision.frame_publication.time.time",
            return_value=123.5,
        ):
            provider.push_frame(image)
        snapshot = provider.get_frame_snapshot()
        self.assertIsNot(snapshot.frame, image)
        self.assertEqual((snapshot.width, snapshot.height), (30, 20))
        self.assertEqual(snapshot.sequence, provider.get_frame_seq())
        self.assertEqual(snapshot.published_at_s, 123.5)
        image.fill(99)
        self.assertEqual(int(snapshot.frame[0, 0, 0]), 17)
        provider.stop()

    def test_wait_for_newer_frame_returns_after_publish(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        start_sequence = provider.get_frame_seq()

        def publish() -> None:
            time.sleep(0.02)
            provider.push_frame(np.zeros((10, 12, 3), dtype=np.uint8))

        threading.Thread(target=publish, daemon=True).start()
        result = provider.wait_for_newer_frame(start_sequence, timeout=0.5)
        self.assertIsNotNone(result)
        frame, width, height, sequence = result
        self.assertIsNotNone(frame)
        self.assertEqual((width, height), (12, 10))
        self.assertGreater(sequence, start_sequence)
        provider.stop()

    def test_wait_for_newer_frame_times_out(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        result = provider.wait_for_newer_frame(
            provider.get_frame_seq(),
            timeout=0.01,
        )
        self.assertIsNone(result)
        provider.stop()

    def test_thread_safety(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        errors = []

        def push_loop() -> None:
            for index in range(100):
                provider.push_frame(
                    np.full((48, 64, 3), index % 256, dtype=np.uint8)
                )

        def get_loop() -> None:
            for _ in range(100):
                frame, width, height = provider.get_frame()
                if frame is not None and (width, height) != (64, 48):
                    errors.append(f"Bad dims: {width}x{height}")

        push_thread = threading.Thread(target=push_loop)
        get_thread = threading.Thread(target=get_loop)
        push_thread.start()
        get_thread.start()
        push_thread.join()
        get_thread.join()
        self.assertEqual(errors, [])
        provider.stop()


class TestFrameProviderLifecycle(unittest.TestCase):
    def test_double_start(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        provider.start()
        provider.stop()

    def test_stop_without_start(self):
        FrameProvider(source=None, logger=_Logger()).stop()

    def test_push_after_stop_still_publishes(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        provider.stop()
        provider.push_frame(np.zeros((10, 10, 3), dtype=np.uint8))
        frame, width, height = provider.get_frame()
        self.assertIsNotNone(frame)
        self.assertEqual((width, height), (10, 10))

    def test_already_new_frame_is_readable_after_stop(self):
        provider = FrameProvider(source=None, logger=_Logger())
        provider.start()
        sequence = provider.get_frame_seq()
        provider.push_frame(np.zeros((4, 5, 3), dtype=np.uint8))
        provider.stop()
        result = provider.wait_for_newer_frame(sequence, timeout=0.01)
        self.assertIsNotNone(result)


if __name__ == "__main__":
    unittest.main()
