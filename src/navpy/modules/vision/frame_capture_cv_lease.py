"""Transactional ownership for raw OpenCV capture handles."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

from types import TracebackType

import cv2


class OpenCvCaptureLease:
    """Release a raw capture unless ownership is explicitly transferred."""

    def __init__(self, capture: cv2.VideoCapture) -> None:
        self.capture = capture
        self._transferred = False

    def __enter__(self) -> OpenCvCaptureLease:
        return self

    def __exit__(
        self,
        _error_type: type[BaseException] | None,
        primary_error: BaseException | None,
        _traceback: TracebackType | None,
    ) -> bool:
        if self._transferred:
            return False
        try:
            self.capture.release()
        except BaseException as cleanup_error:
            if primary_error is None:
                raise
            raise BaseExceptionGroup(
                "OpenCV capture operation and cleanup failed",
                [primary_error, cleanup_error],
            ) from None
        return False

    def transfer(self) -> cv2.VideoCapture:
        self._transferred = True
        return self.capture


__all__ = ["OpenCvCaptureLease"]
