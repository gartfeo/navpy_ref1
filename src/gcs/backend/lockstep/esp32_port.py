"""The in-process ESP32 simulator port (``lockstep_slice.md``, "Ports").

It covers start, status, health (the launch session's check too), trigger
results and stop, from the simulator's own state
(``esp32_simulator.Esp32SimulatorState``), with no ``HTTPServer`` and no
port. A client reaches it only while the simulator runs, as the loopback
HTTP one does; a trigger answers at once and holds its channel on a tracked
producer thread, as the HTTP handler does; a cancel (the shutdown's) releases
the channel.
"""

from __future__ import annotations

from typing import Any, Callable

from gcs.backend.esp32_simulator import NUM_CHANNELS, TRIGGER_DURATION_MS, Esp32SimulatorState, _state
from gcs.backend.esp32_trigger import TriggerResult
from gcs.backend.producer_inventory import producers
from navpy.core.kernel import runtime_time as rt


class _Simulator:
    """``Esp32Simulator``'s surface; ``port`` is None (it reports none)."""

    port = None

    def __init__(self, owner: "InProcessEsp32") -> None:
        self._owner = owner

    def start(self) -> None:
        self._owner.running = self

    def stop(self) -> None:
        if self._owner.running is self:
            self._owner.running = None

    def set_trigger_callback(self, callback: Callable[[int], None]) -> None:
        _state.on_trigger_callback = callback

    def get_state(self) -> Esp32SimulatorState:
        return _state


class _Client:
    """``Esp32TriggerClient``'s surface, answered from the simulator's state."""

    def __init__(self, owner: "InProcessEsp32") -> None:
        self._owner = owner

    def health_check(self) -> bool:
        return self._owner.running is not None

    def get_status(self) -> dict | None:
        return _state.get_status() if self.health_check() else None

    def reset_all(self) -> bool:
        if not self.health_check():
            return False
        _state.reset()
        return True

    def trigger_channel(self, channel: int) -> TriggerResult:
        if not self.health_check():
            return TriggerResult(success=False, channel=channel, message="Connection failed")
        if not 1 <= channel <= NUM_CHANNELS:
            return TriggerResult(success=False, channel=channel,
                                 message=f"Invalid channel. Must be 1-{NUM_CHANNELS}")
        producers.start_thread("esp32_simulator", lambda: _hold(channel),
                               name=f"esp32-trigger-{channel}")
        return TriggerResult(success=True, channel=channel, message="Trigger successful",
                             duration_ms=TRIGGER_DURATION_MS)


def _hold(channel: int) -> None:
    """The HTTP handler's trigger thread; a cancel mid-hold releases the channel."""
    try:
        _state.trigger_channel(channel)
    except BaseException:
        with rt.shielded():  # the release completes in the cancelled scope
            _state.reset()
        raise


class InProcessEsp32:
    """The ``Esp32Port`` the backend participant installs."""

    def __init__(self) -> None:
        self.running: _Simulator | None = None

    def simulator(self, port: int) -> _Simulator:
        del port  # no socket: nothing binds
        return _Simulator(self)

    def client(self, host: str, port: Any, timeout: float = 5.0) -> _Client:
        del host, port, timeout
        return _Client(self)


__all__ = ["InProcessEsp32"]
