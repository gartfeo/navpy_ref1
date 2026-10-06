"""Regressions for printf-style logger calls in the vision package.

``ILogger`` has no ``%``-style lazy formatting: its positional slots after
``msg`` already mean ``key``, ``status``, ``dest`` and ``check_interval``.
Passing format arguments positionally therefore either raises (too many
arguments) or silently binds them to routing metadata. These tests pin the
six call sites that used to do exactly that.
"""
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.cache_logger import ConsoleCacheLogger
from navpy.logger.logger_api import ILogger
from navpy.modules.vision.tracker_backends import (
    BotSortTrackerBackend,
    StrongSortTrackerBackend,
    tracker_config_from_settings,
)
from navpy.modules.vision.yolo_detector import Detection, YoloDetector
from navpy.modules.vision.tracking_command_router import TrackingCommandRouter

from tests.modules.vision.test_tracker_backends import (
    _FakeBotSort,
    _FakeReID,
    _FakeStrongSort,
)


class _LogCall(SimpleNamespace):
    """One recorded call, with every metadata slot the API exposes."""


class _RecordingLogger(ILogger):
    """Signature-faithful ILogger that records routing metadata per call.

    A ``Mock`` accepts any argument count, so it hides exactly the defect
    these tests cover. This spy mirrors the real signatures instead.
    """

    def __init__(self) -> None:
        self.infos: list[_LogCall] = []
        self.warnings: list[_LogCall] = []
        self.errors: list[_LogCall] = []

    def is_enabled_for(self, level: CacheLogLevel) -> bool:
        return True

    def with_prefix(self, prefix: object) -> "_RecordingLogger":
        return self

    def verbose(self, msg: object) -> None:
        pass

    def debug(self, msg: object) -> None:
        pass

    def info(self, msg, key="", status=None, dest=None, check_interval=False):
        self.infos.append(_LogCall(
            msg=msg, key=key, status=status, dest=dest,
            check_interval=check_interval,
        ))

    def warning(self, msg, key="", status=None, dest=None):
        self.warnings.append(_LogCall(msg=msg, key=key, status=status, dest=dest))

    def single_warning(self, msg: object, key: str) -> None:
        self.warnings.append(_LogCall(msg=msg, key=key, status=None, dest=None))

    def error(self, msg: object, ex: BaseException | None = None) -> None:
        self.errors.append(_LogCall(msg=msg, ex=ex))

    def defer_status_texts(self, enable: bool, *, flush: bool = True) -> None:
        pass

    def close(self) -> None:
        pass

    def refresh(self) -> None:
        pass


def _assert_no_routing_metadata(case, call, *, key=""):
    """The message must carry no accidental routing metadata."""
    case.assertEqual(call.key, key)
    case.assertIsNone(call.status)
    case.assertIsNone(call.dest)
    case.assertFalse(getattr(call, "check_interval", False))


class _FakeYOLO:
    def __init__(self, model_path):
        self.model_path = model_path


def _torch_with_cuda(available):
    return SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: bool(available)),
    )


def _fake_vision_modules(cuda=False):
    return {
        "ultralytics": SimpleNamespace(YOLO=_FakeYOLO),
        "torch": _torch_with_cuda(cuda),
    }


class TestYoloDetectorStartupLog(unittest.TestCase):
    def test_construction_with_real_cache_logger_does_not_raise(self):
        # The original defect: 7 positional arguments into info(), which
        # accepts at most 6, so every real-camera detector build died with
        # TypeError before any inference ran.
        logger = ConsoleCacheLogger()
        try:
            with patch.dict(sys.modules, _fake_vision_modules()):
                YoloDetector("model.pt", device="cpu", logger=logger)
        finally:
            logger.close()

    def test_startup_log_is_formatted_and_carries_no_metadata(self):
        logger = _RecordingLogger()

        with patch.dict(sys.modules, _fake_vision_modules()):
            detector = YoloDetector("model.pt", device="cpu", logger=logger)

        self.assertEqual(len(logger.infos), 1)
        call = logger.infos[0]
        _assert_no_routing_metadata(self, call)
        self.assertIn("model=model.pt", call.msg)
        self.assertIn(f"imgsz={detector.imgsz}", call.msg)
        self.assertIn("conf=0.35", call.msg)
        self.assertIn("device=cpu", call.msg)
        self.assertIn("half=False", call.msg)
        self.assertIn("classes=None", call.msg)

    def test_fp16_fallback_warning_is_formatted_and_carries_no_metadata(self):
        logger = _RecordingLogger()
        detector = YoloDetector.__new__(YoloDetector)
        detector.model = _FailingHalfYOLO()
        detector.imgsz = 640
        detector.conf = 0.2
        detector.classes = None
        detector.device = 0
        detector.half = True
        detector._logger = logger

        detector.detect(np.zeros((10, 10, 3), dtype=np.uint8))

        self.assertEqual(len(logger.warnings), 1)
        call = logger.warnings[0]
        _assert_no_routing_metadata(self, call)
        self.assertIn("fp16 unsupported", call.msg)


class TestTrackerBackendStartupLog(unittest.TestCase):
    def _settings(self, backend, **overrides):
        settings = {"backend": backend, "reid_device": "cpu"}
        settings.update(overrides)
        return tracker_config_from_settings(settings, detector_conf=0.2)

    def test_botsort_startup_log_carries_no_metadata(self):
        # Previously bound reid.enabled -> key, reid_device -> status,
        # half -> dest and cmc_method -> check_interval.
        logger = _RecordingLogger()
        with patch(
            "navpy.modules.vision.tracker_backends_boxmot._load_botsort_deps",
            return_value=(_FakeReID, _FakeBotSort),
        ):
            BotSortTrackerBackend(
                self._settings("botsort"),
                detector_device="cpu",
                logger=logger,
            )

        self.assertEqual(len(logger.infos), 1)
        call = logger.infos[0]
        _assert_no_routing_metadata(self, call)
        # Exact text: every argument the %-style call used to pass must still
        # reach the message, not just the two that are easy to spot.
        self.assertEqual(
            call.msg,
            "Tracker backend loaded: botsort with_reid=True reid_device=cpu "
            "half=False cmc=sof",
        )

    def test_strongsort_startup_log_carries_no_metadata(self):
        # cmc_method=None used to leave dest=None, letting a bool status
        # reach both status routes and fail later at .encode().
        logger = _RecordingLogger()
        with patch(
            "navpy.modules.vision.tracker_backends_boxmot._load_strongsort_deps",
            return_value=(_FakeReID, _FakeStrongSort),
        ):
            StrongSortTrackerBackend(
                self._settings("strongsort", min_hits=1, cmc_method=None),
                detector_device="cpu",
                logger=logger,
            )

        self.assertEqual(len(logger.infos), 1)
        call = logger.infos[0]
        _assert_no_routing_metadata(self, call)
        self.assertEqual(
            call.msg,
            "Tracker backend loaded: strongsort reid_device=cpu half=False "
            "cmc=None",
        )


class TestDeepSearchLoopLog(unittest.TestCase):
    def test_candidate_log_preserves_per_count_dedupe(self):
        # The old numeric key deduped once per candidate count. Keep that
        # behavior with a named string key while rendering the count.
        from navpy.modules.vision.real_inference import DeepSearchLoop
        from navpy.modules.vision.frame_publication import FrameSnapshot

        logger = _RecordingLogger()
        run_state = SimpleNamespace(is_running=True)
        detections = [Detection(4, 5, 2, 3, 0.8, 1), Detection(1, 2, 3, 4, 0.7, 1)]

        detector = Mock()

        def detect(_frame):
            run_state.is_running = False
            return detections

        detector.detect.side_effect = detect

        provider = Mock()
        provider.get_frame_snapshot.return_value = FrameSnapshot(
            frame=np.zeros((12, 16, 3), dtype=np.uint8),
            width=16,
            height=12,
            sequence=9,
            published_at_s=1.0,
        )

        channel = Mock()
        channel.should_run.return_value = True
        channel.publish.return_value = True

        DeepSearchLoop(
            run_state,
            provider,
            detector,
            SimpleNamespace(period=0.05, stale_seconds=0.5),
            channel,
            Mock(),
            logger,
            SimpleNamespace(monotonic=lambda: 1.0, sleep=Mock()),
        ).run()

        self.assertEqual(logger.errors, [])
        self.assertEqual(len(logger.infos), 1)
        call = logger.infos[0]
        _assert_no_routing_metadata(self, call, key="deep_search_candidates_2")
        self.assertIn(f"{len(detections)} candidates", call.msg)


class TestTrackingFallbackLog(unittest.TestCase):
    def test_unknown_task_warning_is_formatted_without_routing_metadata(self):
        logger = _RecordingLogger()
        member = Mock()
        resolver = SimpleNamespace(identity_for_task=lambda _task_id: None)

        TrackingCommandRouter([member], resolver, logger).start_tracking(42)

        member.start_tracking.assert_called_once_with(42)
        self.assertEqual(len(logger.warnings), 1)
        call = logger.warnings[0]
        _assert_no_routing_metadata(self, call)
        self.assertEqual(
            call.msg,
            "DetectionCoordinator: start_tracking(42) is not resolvable "
            "to a source detector; broadcasting (legacy fallback)",
        )


class _FailingHalfYOLO:
    """Raises on FP16 so detect() takes the FP32 fallback path once."""

    def predict(self, **kwargs):
        if kwargs.get("half"):
            raise RuntimeError("fp16 unsupported")
        return [SimpleNamespace(boxes=None)]


if __name__ == "__main__":
    unittest.main()
