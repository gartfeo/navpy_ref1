"""Focused tests for capture-source selection and runtime state."""

import threading
import unittest
from unittest.mock import patch

import numpy as np

from navpy.exception_groups import BaseExceptionGroup
from navpy.logger.logger_api import ConsoleLogger
from navpy.modules.vision.frame_capture_runtime import FrameCaptureRuntime
from navpy.modules.vision.frame_publication import FramePublicationStore

from tests.modules.vision.test_frame_provider import _Logger


class _BlockingCapture:
    def __init__(self):
        self.started = threading.Event()
        self.stop_requested = threading.Event()
        self.release = threading.Event()
        self.exited = threading.Event()
        self.closed = threading.Event()
        self.request_calls = 0
        self.close_calls = 0
        self.closed_while_running = False

    def run(self, publish, is_running):
        self.started.set()
        self.release.wait(timeout=2.0)
        publish(np.ones((2, 2, 3), dtype=np.uint8), None)
        self.exited.set()

    def request_stop(self):
        self.request_calls += 1
        self.stop_requested.set()
        return True

    def close(self):
        self.close_calls += 1
        self.closed_while_running = not self.exited.is_set()
        self.closed.set()


class _ReturningCapture:
    def __init__(self):
        self.closed = threading.Event()
        self.close_calls = 0

    def run(self, publish, is_running):
        return

    def request_stop(self):
        return True

    def close(self):
        self.close_calls += 1
        self.closed.set()


class _RollbackFailureCapture(_BlockingCapture):
    def request_stop(self):
        self.request_calls += 1
        raise RuntimeError("interrupt failed")


class TestFrameCaptureRuntime(unittest.TestCase):
    def _blocking_runtime(self):
        publications = FramePublicationStore()
        runtime = FrameCaptureRuntime("camera", _Logger(), publications)
        backend = _BlockingCapture()
        opened = patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        )
        opened_mock = opened.start()
        self.addCleanup(opened.stop)
        timeout = patch(
            "navpy.modules.vision.frame_capture_runtime._STOP_WARNING_S",
            0.0,
        )
        timeout.start()
        self.addCleanup(timeout.stop)
        runtime.start()
        self.assertTrue(backend.started.wait(timeout=1.0))
        return runtime, publications, backend, opened_mock

    def test_stop_timeout_returns_false_and_retains_session_for_retry(self):
        runtime, _, backend, opened = self._blocking_runtime()
        self.assertFalse(runtime.stop())
        self.assertTrue(runtime.has_capture_thread)
        self.assertEqual(backend.close_calls, 0)

        runtime.start()
        self.assertEqual(opened.call_count, 1)
        self.assertFalse(runtime.stop())

        backend.release.set()
        self.assertTrue(backend.closed.wait(timeout=1.0))
        self.assertTrue(runtime.stop())
        self.assertEqual(backend.close_calls, 1)
        self.assertFalse(backend.closed_while_running)
        self.assertFalse(runtime.has_capture_thread)

    def test_inflight_capture_result_is_rejected_after_stop(self):
        runtime, publications, backend, _ = self._blocking_runtime()
        before = publications.sequence()
        self.assertFalse(runtime.stop())
        backend.release.set()
        self.assertTrue(backend.closed.wait(timeout=1.0))

        self.assertTrue(runtime.stop())
        self.assertEqual(publications.sequence(), before)

    def test_concurrent_stop_closes_one_capture_generation_once(self):
        runtime, _, backend, _ = self._blocking_runtime()
        results = []
        first = threading.Thread(target=lambda: results.append(runtime.stop()))
        second = threading.Thread(target=lambda: results.append(runtime.stop()))
        first.start()
        self.assertTrue(backend.stop_requested.wait(timeout=1.0))
        second.start()
        first.join(timeout=2.0)
        second.join(timeout=2.0)

        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertEqual(results, [False, False])
        self.assertEqual(backend.close_calls, 0)
        backend.release.set()
        self.assertTrue(backend.closed.wait(timeout=1.0))
        self.assertTrue(runtime.stop())
        self.assertGreaterEqual(backend.request_calls, 2)
        self.assertEqual(backend.close_calls, 1)

    def test_thread_start_failure_rolls_back_and_closes_backend(self):
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        backend = _BlockingCapture()
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ):
            with patch(
                "navpy.modules.vision.frame_capture_runtime.threading.Thread.start",
                side_effect=RuntimeError("thread start failed"),
            ):
                with self.assertRaisesRegex(RuntimeError, "thread start failed"):
                    runtime.start()

        self.assertFalse(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)
        self.assertEqual(backend.request_calls, 1)
        self.assertEqual(backend.close_calls, 1)

    def test_thread_start_failure_after_launch_retains_session_for_stop(self):
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        backend = _BlockingCapture()
        original_start = threading.Thread.start

        def launch_then_fail(thread):
            original_start(thread)
            raise RuntimeError("thread launch acknowledgement failed")

        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ), patch(
            "navpy.modules.vision.frame_capture_runtime.threading.Thread.start",
            launch_then_fail,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "thread launch acknowledgement failed",
            ):
                runtime.start()

        self.assertTrue(backend.started.wait(timeout=1.0))
        self.assertFalse(runtime.is_running)
        self.assertTrue(runtime.has_capture_thread)
        with patch(
            "navpy.modules.vision.frame_capture_runtime._STOP_WARNING_S",
            0.01,
        ):
            self.assertFalse(runtime.stop())
        self.assertEqual(backend.close_calls, 0)

        backend.release.set()
        self.assertTrue(backend.closed.wait(timeout=1.0))
        self.assertTrue(runtime.stop())
        self.assertFalse(runtime.has_capture_thread)
        self.assertEqual(backend.close_calls, 1)

    def test_thread_construction_failure_rolls_back_and_closes_backend(self):
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        backend = _BlockingCapture()
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ):
            with patch(
                "navpy.modules.vision.frame_capture_runtime.threading.Thread",
                side_effect=RuntimeError("thread construction failed"),
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "thread construction failed",
                ):
                    runtime.start()

        self.assertEqual(backend.request_calls, 1)
        self.assertEqual(backend.close_calls, 1)
        self.assertFalse(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)

    def test_session_construction_failure_closes_raw_backend(self):
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        backend = _BlockingCapture()
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ):
            with patch(
                "navpy.modules.vision.frame_capture_session.CaptureSession",
                side_effect=RuntimeError("session construction failed"),
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "session construction failed",
                ):
                    runtime.start()

        self.assertEqual(backend.request_calls, 1)
        self.assertEqual(backend.close_calls, 1)
        self.assertFalse(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)

    def test_thread_start_failure_still_closes_when_interrupt_raises(self):
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        backend = _RollbackFailureCapture()
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ):
            with patch(
                "navpy.modules.vision.frame_capture_runtime.threading.Thread.start",
                side_effect=RuntimeError("thread start failed"),
            ):
                with self.assertRaises(BaseExceptionGroup) as raised:
                    runtime.start()

        messages = [str(error) for error in raised.exception.exceptions]
        self.assertEqual(messages, ["thread start failed", "interrupt failed"])
        self.assertEqual(backend.request_calls, 1)
        self.assertEqual(backend.close_calls, 1)
        self.assertFalse(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)

    def test_natural_worker_exit_closes_and_allows_restart(self):
        first = _ReturningCapture()
        second = _ReturningCapture()
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            side_effect=[first, second],
        ) as opened:
            runtime.start()
            self.assertTrue(first.closed.wait(timeout=1.0))
            self.assertFalse(runtime.is_running)
            self.assertFalse(runtime.has_capture_thread)
            runtime.start()
            self.assertTrue(second.closed.wait(timeout=1.0))

        self.assertEqual(opened.call_count, 2)
        self.assertEqual(first.close_calls, 1)
        self.assertEqual(second.close_calls, 1)

    def test_delayed_bootstrap_is_cancelled_before_backend_close(self):
        class _Capture:
            def __init__(self):
                self.run_calls = 0
                self.close_calls = 0

            def run(self, _publish, _is_running):
                self.run_calls += 1

            def request_stop(self):
                return True

            def close(self):
                self.close_calls += 1

        backend = _Capture()
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        release = threading.Event()
        original_start = threading.Thread.start
        delayed_thread = None
        launcher = None

        def delayed_start(thread):
            nonlocal delayed_thread, launcher
            delayed_thread = thread

            def launch_later():
                self.assertTrue(release.wait(timeout=1.0))
                original_start(thread)

            launcher = threading.Thread(target=launch_later, daemon=True)
            original_start(launcher)
            return None

        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ), patch.object(threading.Thread, "start", delayed_start):
            runtime.start()
            self.assertTrue(runtime.stop())

        self.assertEqual(backend.close_calls, 1)
        release.set()
        launcher.join(timeout=1.0)
        delayed_thread.join(timeout=1.0)

        self.assertEqual(backend.run_calls, 0)
        self.assertFalse(runtime.has_capture_thread)

    def test_unexpected_normal_backend_return_is_persistent_failure(self):
        backend = _ReturningCapture()
        runtime = FrameCaptureRuntime(
            "camera",
            _Logger(),
            FramePublicationStore(),
        )
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=backend,
        ):
            runtime.start()

        self.assertTrue(backend.closed.wait(timeout=1.0))
        for _ in range(2):
            with self.assertRaisesRegex(
                RuntimeError,
                "capture backend stopped unexpectedly",
            ):
                runtime.raise_if_failed()

    def test_push_mode_has_no_capture_thread(self):
        runtime = FrameCaptureRuntime(
            None,
            _Logger(),
            FramePublicationStore(),
        )
        runtime.start()
        self.assertTrue(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)
        self.assertTrue(runtime.stop())
        self.assertFalse(runtime.is_running)

    def test_failed_configured_source_fails_start_and_leaves_runtime_stopped(self):
        logger = ConsoleLogger()
        runtime = FrameCaptureRuntime(
            "rtsp://camera/stream",
            logger,
            FramePublicationStore(),
        )
        with patch(
            "navpy.modules.vision.frame_capture_runtime.open_capture_backend",
            return_value=None,
        ):
            with patch("builtins.print") as output:
                with self.assertRaisesRegex(
                    RuntimeError,
                    "failed to open configured source",
                ):
                    runtime.start()
        self.assertFalse(runtime.is_running)
        self.assertFalse(runtime.has_capture_thread)
        self.assertTrue(
            any(
                "failed to open configured source" in str(call)
                for call in output.call_args_list
            )
        )
        self.assertTrue(runtime.stop())


if __name__ == "__main__":
    unittest.main()
