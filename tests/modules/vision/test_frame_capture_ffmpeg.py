"""Focused tests for the FFmpeg capture backend."""

import subprocess
import unittest
from unittest.mock import Mock, patch

from navpy.modules.vision.frame_capture_ffmpeg import (
    FfmpegFrameCapture,
    probe_resolution,
)

from tests.modules.vision.test_frame_provider import _Logger


class TestFfmpegFrameCapture(unittest.TestCase):
    def test_probe_failure_returns_none_and_logs(self):
        logger = _Logger()
        result = Mock(returncode=1, stdout="", stderr="unreachable")
        with patch(
            "navpy.modules.vision.frame_capture_ffmpeg.subprocess.run",
            return_value=result,
        ):
            self.assertIsNone(
                probe_resolution("rtsp://camera/stream", logger)
            )
        self.assertTrue(
            any(
                "ffprobe returned 1" in message
                for _, message in logger.messages
            )
        )

    def test_launch_suppresses_progress_logging(self):
        logger = _Logger()
        process = Mock()
        process.stderr = Mock()
        with patch(
            "navpy.modules.vision.frame_capture_ffmpeg.probe_resolution",
            return_value=(640, 480),
        ):
            with patch(
                "navpy.modules.vision.frame_capture_ffmpeg.subprocess.Popen",
                return_value=process,
            ) as popen:
                with patch(
                    "navpy.modules.vision.frame_capture_ffmpeg.threading.Thread"
                ) as thread_class:
                    capture = FfmpegFrameCapture.open(
                        "rtsp://camera/stream",
                        logger,
                    )
        self.assertIsNotNone(capture)
        command = popen.call_args.args[0]
        self.assertIn("-hide_banner", command)
        self.assertIn("-loglevel", command)
        self.assertEqual(command[command.index("-loglevel") + 1], "fatal")
        thread_class.return_value.start.assert_called_once()

    def test_stderr_thread_start_failure_reaps_launched_process(self):
        logger = _Logger()
        process = Mock()
        process.stderr = Mock()
        with patch(
            "navpy.modules.vision.frame_capture_ffmpeg.probe_resolution",
            return_value=(640, 480),
        ):
            with patch(
                "navpy.modules.vision.frame_capture_ffmpeg.subprocess.Popen",
                return_value=process,
            ):
                with patch(
                    "navpy.modules.vision.frame_capture_ffmpeg.threading.Thread"
                ) as thread_class:
                    thread_class.return_value.start.side_effect = RuntimeError(
                        "thread start failed"
                    )
                    thread_class.return_value.is_alive.return_value = False
                    with self.assertRaisesRegex(
                        RuntimeError,
                        "thread start failed",
                    ):
                        FfmpegFrameCapture.open(
                            "rtsp://camera/stream",
                            logger,
                        )

        process.kill.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=2.0)

    def test_stderr_thread_construction_failure_reaps_launched_process(self):
        logger = _Logger()
        process = Mock()
        process.stderr = Mock()
        with patch(
            "navpy.modules.vision.frame_capture_ffmpeg.probe_resolution",
            return_value=(640, 480),
        ):
            with patch(
                "navpy.modules.vision.frame_capture_ffmpeg.subprocess.Popen",
                return_value=process,
            ):
                with patch(
                    "navpy.modules.vision.frame_capture_ffmpeg.threading.Thread",
                    side_effect=RuntimeError("thread construction failed"),
                ):
                    with self.assertRaisesRegex(
                        RuntimeError,
                        "thread construction failed",
                    ):
                        FfmpegFrameCapture.open(
                            "rtsp://camera/stream",
                            logger,
                        )

        process.kill.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=2.0)

    def test_capture_construction_failure_reaps_launched_process(self):
        logger = _Logger()
        process = Mock()
        process.stderr = Mock()

        class _FailingCapture(FfmpegFrameCapture):
            def __init__(self, *_args, **_kwargs):
                raise RuntimeError("capture construction failed")

        with patch(
            "navpy.modules.vision.frame_capture_ffmpeg.probe_resolution",
            return_value=(640, 480),
        ):
            with patch(
                "navpy.modules.vision.frame_capture_ffmpeg.subprocess.Popen",
                return_value=process,
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "capture construction failed",
                ):
                    _FailingCapture.open(
                        "rtsp://camera/stream",
                        logger,
                    )

        process.kill.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=2.0)

    def test_close_is_idempotent(self):
        process = Mock()
        capture = FfmpegFrameCapture(process, 10, 8, _Logger())
        capture.close()
        capture.close()
        process.kill.assert_called_once_with()
        process.wait.assert_called_once_with(timeout=2.0)

    def test_kill_failure_remains_retryable(self):
        process = Mock()
        process.kill.side_effect = [OSError("busy"), None]
        capture = FfmpegFrameCapture(process, 10, 8, _Logger())

        self.assertFalse(capture.request_stop())
        self.assertTrue(capture.request_stop())
        capture.close()

        self.assertEqual(process.kill.call_count, 2)
        process.wait.assert_called_once_with(timeout=2.0)

    def test_wait_timeout_remains_retryable(self):
        process = Mock()
        process.wait.side_effect = [
            subprocess.TimeoutExpired("ffmpeg", 2.0),
            None,
        ]
        capture = FfmpegFrameCapture(process, 10, 8, _Logger())

        with self.assertRaises(subprocess.TimeoutExpired):
            capture.close()
        capture.close()

        self.assertEqual(process.kill.call_count, 2)
        self.assertEqual(process.wait.call_count, 2)

    def test_stderr_worker_timeout_is_bounded_and_retryable(self):
        process = Mock()
        stderr_thread = Mock()
        stderr_thread.is_alive.side_effect = [True, True, False]
        capture = FfmpegFrameCapture(process, 10, 8, _Logger())
        capture._stderr_thread = stderr_thread

        with self.assertRaisesRegex(TimeoutError, "stderr worker did not stop"):
            capture.close()

        stderr_thread.join.assert_called_once_with(timeout=2.0)
        capture.close()
        self.assertEqual(process.wait.call_count, 2)


if __name__ == "__main__":
    unittest.main()
