"""Retryable ownership of the optional vision debug surface."""

from __future__ import annotations

from typing import Callable, Protocol


class VisionDebugClosePort(Protocol):
    def close(self) -> None: ...


class VisionDebugOwner:
    def __init__(
        self,
        debug: VisionDebugClosePort | None,
        report_error: Callable[[str], None],
    ) -> None:
        self._debug = debug
        self._report_error = report_error
        self._closed = False

    @property
    def is_closed(self) -> bool:
        return self._closed

    def close(self) -> BaseException | None:
        if self._closed:
            return None
        if self._debug is None:
            self._closed = True
            return None
        try:
            self._debug.close()
        except BaseException as close_error:
            self._report_error(
                f"Failed to close vision debug window: {close_error}"
            )
            return close_error
        self._closed = True
        return None


__all__ = ["VisionDebugClosePort", "VisionDebugOwner"]
