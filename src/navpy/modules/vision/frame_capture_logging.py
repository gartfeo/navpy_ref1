"""Failure-isolated logging adapter for frame-capture resources."""

from __future__ import annotations

from navpy.modules.vision.frame_capture_ports import CaptureLogger


class SafeCaptureLogger:
    """Keep diagnostic failures outside capture lifecycle transactions."""

    def __init__(self, logger: CaptureLogger) -> None:
        self._logger = logger

    def info(self, message: str) -> None:
        self._write(self._logger.info, message)

    def warning(self, message: str) -> None:
        self._write(self._logger.warning, message)

    def error(self, message: str) -> None:
        self._write(self._logger.error, message)

    @staticmethod
    def _write(writer: object, message: str) -> None:
        if not callable(writer):
            return
        try:
            writer(message)
        except Exception:
            pass


__all__ = ["SafeCaptureLogger"]
