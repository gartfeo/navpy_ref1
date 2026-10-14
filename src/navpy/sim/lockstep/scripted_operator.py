"""Scripted GCS operators for serial-step SITLs, run under the baton (steps 7, 10d).

:class:`ScriptedFleetOperator` flies N vehicles' scripts
(``scripted_vehicle``) on one baton (the mission modules' ``rt`` is module
state, so one runtime serves them all); its clock starts at the tick it is
admitted. :class:`ScriptedOperator` is step 7's single-SITL operator: it
joins at the boot barrier (the first frame with ``helpers_quiet``), so its
clock starts at that tick's ``source_us``.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from navpy.plugins.vehicle.ardupilot import mission_inbox, mission_upload
from navpy.sim.lockstep.baton import BatonRuntime, LockstepFault
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime, module_port
from navpy.sim.lockstep.scripted_vehicle import ScriptedVehicle, ScriptStep
from navpy.sim.lockstep.step_wire import Frame


class ScriptedFleetOperator:
    """N vehicles' scripts on one baton; each on its own SITL's serial1."""

    def __init__(self, vehicles: Sequence[tuple[int, str]]) -> None:
        self.vehicles = tuple(ScriptedVehicle(self, sysid, home) for sysid, home in vehicles)
        self._patched: list[tuple[Any, str, Any]] = []
        self.runtime: BatonRuntime | None = None
        self.tick = 0

    @property
    def done(self) -> bool:
        return all(vehicle.done for vehicle in self.vehicles)

    def on_tick(self, k: int, now_us: int, egress: Sequence[bytes],
                deadline: float | None = None) -> list[bytes]:
        """Tick k at ``now_us``: each vehicle's serial1 egress in, its serial1
        ingress out (SITL order). The first call starts the operator at
        ``now_us``. ``deadline`` (``time.monotonic()``) bounds every baton
        handoff of the tick, the startup's included."""
        self.tick = k
        if self.runtime is None:
            self._start(now_us, deadline)
        inputs = [(vehicle, data) for vehicle, data in zip(self.vehicles, egress) if data]

        def deliver() -> None:
            for vehicle, data in inputs:
                vehicle.deliver(data)

        self.runtime.tick(now_us, deliver if inputs else None, deadline=deadline)
        return [vehicle.drain() for vehicle in self.vehicles]

    def close(self) -> None:
        """Unwind the baton's threads, then give the mission modules their host
        time back. A runtime poisoned by a holder that still runs (a handoff cut
        by its deadline) refuses the abort: only ownership teardown is left, so
        its threads stay parked until the process ends and the run's fault stands."""
        try:
            if self.runtime is not None:
                self.runtime.abort()
        except LockstepFault:
            pass
        finally:
            while self._patched:
                module, name, original = self._patched.pop()
                setattr(module, name, original)

    def _start(self, now_us: int, deadline: float | None) -> None:
        runtime = BatonRuntime(start_us=now_us)
        self.runtime = runtime
        # The coordinator process stays on the wall runtime (its relay link is
        # host IO), so only the mission modules' ``rt`` is swapped for the run.
        # The operator is ordinal 0; its entropy is unused, so seed 0.
        port = module_port(LockstepRuntime.create(runtime, epoch_us=0, seed=0, ordinal=0))
        for module in (mission_inbox, mission_upload):
            self._patched.append((module, "rt", getattr(module, "rt")))
            setattr(module, "rt", port)
        for vehicle in self.vehicles:
            vehicle.start(runtime)
        runtime.run_until_quiescent(deadline)


class ScriptedOperator:
    """Step 7's operator for one SITL: it joins at the first ``helpers_quiet`` frame."""

    def __init__(self, sysid: int, home: str) -> None:
        self._fleet = ScriptedFleetOperator(((sysid, home),))
        self._vehicle = self._fleet.vehicles[0]

    @property
    def tick(self) -> int:
        return self._fleet.tick

    @property
    def steps(self) -> list[ScriptStep]:
        return self._vehicle.steps

    @property
    def statustexts(self) -> list[ScriptStep]:
        return self._vehicle.statustexts

    @property
    def done(self) -> bool:
        return self._fleet.done

    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        if self._fleet.runtime is None and not frame.helpers_quiet:
            self._fleet.tick = frame.tick
            return b"", b""  # boot barrier: only the SITL ticks
        (serial1,) = self._fleet.on_tick(frame.tick, frame.source_us, (frame.ports[1].egress,))
        return b"", serial1

    def close(self) -> None:
        self._fleet.close()


__all__ = ["ScriptedFleetOperator", "ScriptedOperator"]
