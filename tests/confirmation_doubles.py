"""Shared confirmation-round doubles for tests."""

from __future__ import annotations

import math

from navpy.core.review.confirmation_round_transaction import ConfirmationDeadline


class SettableClock:
    """A monotonic-clock stand-in the test moves by hand; counts its reads."""

    def __init__(self, now_s: float = 0.0) -> None:
        self.now_s = now_s
        self.reads = 0

    def __call__(self) -> float:
        self.reads += 1
        return self.now_s


def open_deadline() -> ConfirmationDeadline:
    """A deadline that never passes and reads no real clock.

    For tests about something other than the response window.
    """
    return ConfirmationDeadline(lambda: 0.0, math.inf)


__all__ = ["SettableClock", "open_deadline"]
