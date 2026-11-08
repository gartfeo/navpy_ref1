"""A send the caller waits for only so long (docs/design/navpy_recovery.md, level 4 and step 6).

``MavTransport.try_call`` bounds taking the send lock, not the write. The watchdog must keep running even when a
send hangs, so each send runs on a short-lived thread and the caller waits for it with a deadline. A send that does
not return is reported and left behind; no new send starts while it is still running (a hung send holds the send
lock, so another would hang too), and that skip is reported once.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Generic, TypeVar

from navpy.core.kernel import runtime_time as rt

#: One MAVLink write takes milliseconds; one still running after this holds the send lock and is hung.
BOUNDED_SEND_TIMEOUT_S = 0.5

T = TypeVar("T")


class BoundedSender(Generic[T]):
    """Called from one thread (the watchdog's)."""

    def __init__(
        self,
        send: Callable[[T], object],
        report: Callable[[str], None],
        timeout_s: float = BOUNDED_SEND_TIMEOUT_S,
    ) -> None:
        self._send = send
        self._report = report
        self._timeout_s = float(timeout_s)
        self._stuck: rt.Thread | None = None
        self._skip_reported = False

    def __call__(self, value: T) -> bool:
        """True when the send returned in time without raising."""
        if self._stuck is not None:
            if self._stuck.is_alive():
                if not self._skip_reported:
                    self._skip_reported = True
                    self._report(f"Send of {value} skipped: an earlier send is still hung")
                return False
            self._stuck = None
            self._skip_reported = False
        outcome: list[BaseException | None] = []

        def run() -> None:
            try:
                self._send(value)
                outcome.append(None)
            except BaseException as error:  # noqa: BLE001 - told to the caller below
                outcome.append(error)

        thread = rt.Thread(target=run, daemon=True)
        thread.start()
        thread.join(timeout=self._timeout_s)
        if thread.is_alive():
            self._stuck = thread
            self._report(f"Send of {value} did not return in {self._timeout_s:g} s; left behind")
            return False
        error = outcome[0] if outcome else None
        if error is not None:
            self._report(f"Send of {value} failed: {error!r}")
            return False
        return True


__all__ = ["BOUNDED_SEND_TIMEOUT_S", "BoundedSender"]
