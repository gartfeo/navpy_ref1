"""Exact-once cleanup ownership for real-detector model resources."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import threading

from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
from navpy.modules.vision.deep_search import DeepSearchDetector
from navpy.modules.vision.real_detector_ports import CleanupAction, FrameDetector
from navpy.modules.vision.real_detector_state import ConfirmationFrameStore
from navpy.modules.vision.tracker_backends import TrackerBackend


def _raise_cleanup_errors(errors: tuple[BaseException, ...]) -> None:
    if len(errors) == 1:
        raise errors[0]
    if errors:
        if all(isinstance(error, Exception) for error in errors):
            raise ExceptionGroup("detector resource cleanup failed", list(errors))
        raise BaseExceptionGroup(
            "detector resource cleanup failed",
            list(errors),
        )


class DetectorResources:
    """Close every model resource once and retain every cleanup failure."""

    def __init__(
        self,
        tracker: TrackerBackend,
        appearance: AsyncAppearanceEmbedder | None,
        frame_detector: FrameDetector,
        deep_search: DeepSearchDetector | None,
        confirmation_frames: ConfirmationFrameStore,
    ) -> None:
        self._lock = threading.Lock()
        self._pending_actions: tuple[CleanupAction, ...] = tuple(
            action for action in (
                tracker.close,
                None if appearance is None else appearance.close,
                frame_detector.close,
                None if deep_search is None else deep_search.close,
                confirmation_frames.clear,
            )
            if action is not None
        )

    def close(self) -> None:
        with self._lock:
            pending: list[CleanupAction] = []
            errors: list[BaseException] = []
            for action in self._pending_actions:
                try:
                    action()
                except BaseException as error:
                    pending.append(action)
                    errors.append(error)
            self._pending_actions = tuple(pending)
        _raise_cleanup_errors(tuple(errors))


__all__ = ["DetectorResources"]
