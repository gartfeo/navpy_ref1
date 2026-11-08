"""NAV recovery's mode writes, on a thread of their own (docs/design/navpy_recovery.md, step 6).

Recovery decides a mode (LOITER, AUTO, RTL) in its step and submits it here; it never writes itself, so a write that
stalls (a blocked serial write, a send lock held by a stuck sender) cannot stall recovery, nor the watchdog that
steps recovery while the NAV loop is stuck.

The write is guarded where it happens, under the link's send lock just before the bytes go out: the heartbeat must
still show the mode recovery decided from, armed. A pilot or failsafe mode that showed meanwhile cancels it. Only a
write that passes the guard is NavPy's request: the guard records it at the heartbeat count it read
(``ModeSendEvent``), which recovery gives its ownership record (nav_ownership.py); a write that never went out
records nothing, so the pilot's later choice of the same mode is never taken for NavPy's. A newer submission replaces
one not yet written; MODE_RETRY_S re-submits.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.flow.nav_ownership import ModeObservation
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.flight_mode import FlightMode

MODE_SENDER_JOIN_TIMEOUT_S = 2.0

#: ``send(mode, guard)``: writes ``mode`` if ``guard()``, called under the link's send lock right before the
#: write, returns True; True when written (vehicle.set_mode(mode, lock_timeout_s=..., guard=guard)).
GuardedSend = Callable[[FlightMode, Callable[[], bool]], bool]


@dataclass(frozen=True)
class ModeSendEvent:
    mode: FlightMode
    #: The heartbeat count read just before the write; None: the request did not go out (guard, lock, error).
    heartbeat_count: int | None


class ModeSender:
    """``submit`` and ``take_events`` from recovery's step; ``send_now`` on the sender thread, or inline."""

    def __init__(
        self,
        send: GuardedSend,
        observe_mode: Callable[[], ModeObservation],
        report: Callable[[str], None],
        *,
        threaded: bool = True,
    ) -> None:
        self._send = send
        self._observe_mode = observe_mode
        self._report = report
        self._threaded = threaded
        self._lock = rt.Lock()
        self._wake = rt.Event()
        self._stop = rt.Event()
        self._pending: tuple[FlightMode, FlightMode | None] | None = None
        self._events: list[ModeSendEvent] = []
        self._thread: rt.Thread | None = None

    def submit(self, mode: FlightMode, shown: FlightMode | None) -> None:
        """Write ``mode`` if the heartbeat still shows ``shown`` (the mode recovery decided from) at the write."""
        if not self._threaded:
            self.send_now(mode, shown)
            return
        with self._lock:
            self._pending = (mode, shown)
        self._wake.set()

    def take_events(self) -> list[ModeSendEvent]:
        with self._lock:
            events, self._events = self._events, []
            return events

    def send_now(self, mode: FlightMode, shown: FlightMode | None) -> None:
        guarded: list[bool] = []

        def guard() -> bool:
            observation = self._observe_mode()
            passed = observation.armed and observation.mode is shown
            guarded.append(passed)
            if passed:
                self._post(ModeSendEvent(mode, observation.heartbeat_count))
            return passed

        try:
            sent = bool(self._send(mode, guard))
        except Exception as error:  # noqa: BLE001 - not written; recovery retries
            self._report(f"NavPy {mode.name} request failed: {error!r}")
            sent = False
        if not sent:
            if guarded == [False]:
                self._report(f"NavPy {mode.name} request not sent: the mode changed to another before the write")
            self._post(ModeSendEvent(mode, None))

    def start(self) -> None:
        if not self._threaded or self._thread is not None:
            return
        self._stop.clear()
        self._thread = rt.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=MODE_SENDER_JOIN_TIMEOUT_S)
            if thread.is_alive():
                self._report("NavPy mode sender did not stop: a mode write is stuck")

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                pending, self._pending = self._pending, None
            if pending is not None and not self._stop.is_set():
                self.send_now(*pending)

    def _post(self, event: ModeSendEvent) -> None:
        with self._lock:
            self._events.append(event)


__all__ = ["GuardedSend", "ModeSendEvent", "ModeSender"]
