"""Small retryable LIFO owner for runtime resources."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

from collections.abc import Callable
from types import TracebackType


CleanupCallback = Callable[[], object]


class CleanupStack:
    """Attempt every cleanup in reverse order and retain only failures."""

    def __init__(self) -> None:
        self._callbacks: list[CleanupCallback] = []
        self._closed = False
        self._running = False

    def __enter__(self) -> CleanupStack:
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        primary: BaseException | None,
        _traceback: TracebackType | None,
    ) -> bool:
        cleanup_errors = self._attempt_close()
        if not cleanup_errors:
            return False
        errors = (
            cleanup_errors
            if primary is None
            else [primary, *cleanup_errors]
        )
        _raise_errors(errors)
        return False  # pragma: no cover - _raise_errors never returns

    def push(self, callback: CleanupCallback) -> CleanupCallback:
        if self._closed or self._running:
            raise RuntimeError("cleanup stack no longer accepts callbacks")
        self._callbacks.append(callback)
        return callback

    def close(self) -> None:
        _raise_errors(self._attempt_close())

    def _attempt_close(self) -> list[BaseException]:
        if self._closed:
            return []
        if self._running:
            raise RuntimeError("cleanup stack is already closing")
        self._running = True
        failed_callbacks: list[CleanupCallback] = []
        errors: list[BaseException] = []
        try:
            for callback in reversed(self._callbacks):
                try:
                    callback()
                except BaseException as error:
                    failed_callbacks.append(callback)
                    errors.append(error)
            self._callbacks = list(reversed(failed_callbacks))
            self._closed = not self._callbacks
            return errors
        finally:
            self._running = False


def _raise_errors(errors: list[BaseException]) -> None:
    if not errors:
        return
    if len(errors) == 1:
        raise errors[0]
    raise BaseExceptionGroup("resource cleanup failed", errors)


__all__ = ["CleanupStack"]
