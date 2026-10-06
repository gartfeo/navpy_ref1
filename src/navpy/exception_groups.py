"""Python 3.10-compatible exception-group types.

Python 3.11 provides native groups.  The small fallback preserves the project
contract used by cleanup transactions on older supported runtimes: a message,
an immutable ``exceptions`` tuple, and distinct Exception/BaseException group
catching behavior.  No code in NavPy uses ``except*`` or subgroup operations.
"""

from __future__ import annotations

from collections.abc import Sequence


class _FallbackBaseExceptionGroup(BaseException):
    """Minimal pre-3.11 group for cleanup failures including BaseException."""

    __slots__ = ("_exceptions", "_message")

    def __new__(
        cls,
        message: str,
        exceptions: Sequence[BaseException],
    ) -> "_FallbackBaseExceptionGroup":
        grouped = tuple(exceptions)
        if (
            cls is _FallbackBaseExceptionGroup
            and grouped
            and all(isinstance(error, Exception) for error in grouped)
        ):
            cls = _FallbackExceptionGroup
        return BaseException.__new__(cls)

    def __init__(
        self,
        message: str,
        exceptions: Sequence[BaseException],
    ) -> None:
        grouped = tuple(exceptions)
        if not grouped:
            raise ValueError("exception group must contain at least one error")
        if not all(isinstance(error, BaseException) for error in grouped):
            raise TypeError("exception group members must be exceptions")
        self._message = str(message)
        self._exceptions = grouped
        BaseException.__init__(self, self._message)

    @property
    def message(self) -> str:
        return self._message

    @property
    def exceptions(self) -> tuple[BaseException, ...]:
        return self._exceptions

    def __str__(self) -> str:
        return f"{self.message} ({len(self.exceptions)} sub-exceptions)"


class _FallbackExceptionGroup(Exception, _FallbackBaseExceptionGroup):
    """Minimal pre-3.11 group whose members must all be Exceptions."""

    def __init__(
        self,
        message: str,
        exceptions: Sequence[Exception],
    ) -> None:
        grouped = tuple(exceptions)
        if not all(isinstance(error, Exception) for error in grouped):
            raise TypeError("ExceptionGroup cannot contain BaseException")
        _FallbackBaseExceptionGroup.__init__(self, message, grouped)


try:
    from builtins import BaseExceptionGroup, ExceptionGroup
except ImportError:  # pragma: no cover - selected only on Python 3.10
    BaseExceptionGroup = _FallbackBaseExceptionGroup
    ExceptionGroup = _FallbackExceptionGroup


__all__ = ["BaseExceptionGroup", "ExceptionGroup"]
