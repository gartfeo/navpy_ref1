"""NAV recovery's one outstanding mode request (core/flow/nav_recovery.py): confirmed by a newer heartbeat showing
it, repeated every MODE_RETRY_S while NavPy owns the mode, not confirmed after MODE_CONFIRM_S. Times are readings of
recovery's clock (nav_recovery_clock.py)."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.flow.nav_mode_sender import ModeSender
from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership
from navpy.core.ports.flight_mode import FlightMode

#: A mode request NavPy owns must show in a newer heartbeat by then (ArduPlane heartbeats at 1 Hz).
MODE_CONFIRM_S = 3.0
#: Repeat an unconfirmed mode request this often, not every 50 Hz period.
MODE_RETRY_S = 1.0
#: The mode sender waits this long for the link's send lock, then does not write at all (re-submitted after
#: MODE_RETRY_S): a stuck send holding the lock does not keep newer requests behind it (step 6).
MODE_SEND_LOCK_TIMEOUT_S = 0.5


class ModeRequest:
    """Owned by the NAV loop thread, like the recovery that holds it."""

    def __init__(self, sender: ModeSender, ownership: NavOwnership) -> None:
        """``sender`` (nav_mode_sender.py) writes a mode only while the mode it was decided from still shows, and
        only then is it NavPy's request (``take_sent``)."""
        self._sender = sender
        self._ownership = ownership
        self.pending: FlightMode | None = None
        self._since: float | None = None
        self._count = -1
        self._last_sent: float | None = None

    def send(self, mode: FlightMode, observation: ModeObservation, now: float) -> None:
        self.pending = mode
        self._since = now
        self._count = observation.heartbeat_count
        self._send(mode, observation, now)

    def take_sent(self) -> None:
        """Give ownership the writes that went out, at the heartbeat count read just before each, and forget the
        ones that did not: before any heartbeat that could show them is observed."""
        for event in self._sender.take_events():
            if event.heartbeat_count is None:
                self._ownership.unsent(event.mode)
            else:
                self._ownership.requested(event.mode, event.heartbeat_count)

    def clear(self) -> None:
        self.pending = None

    def not_confirmed(self, observation: ModeObservation, now: float) -> bool:
        """True once the pending request has waited MODE_CONFIRM_S; retries it until then."""
        if self.pending is None:
            return False
        if observation.mode is self.pending and observation.heartbeat_count > self._count:
            self.pending = None
            return False
        if self._since is not None and now - self._since >= MODE_CONFIRM_S:
            return True
        self.retry(observation, now)
        return False

    def retry(self, observation: ModeObservation, now: float) -> None:
        if self.pending is None or not self._ownership.may_request_mode():
            return
        if self._last_sent is None or now - self._last_sent >= MODE_RETRY_S:
            self._send(self.pending, observation, now)

    def _send(self, mode: FlightMode, observation: ModeObservation, now: float) -> None:
        self._last_sent = now
        self._sender.submit(mode, observation.mode)


__all__ = ["MODE_CONFIRM_S", "MODE_RETRY_S", "MODE_SEND_LOCK_TIMEOUT_S", "ModeRequest"]
