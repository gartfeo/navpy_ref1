"""One fixed physical update and readback publication for SIYI simulation."""

from __future__ import annotations

from dataclasses import replace

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.sim.gimbal.aircraft_step_clock import AircraftStepClock
from navpy.sim.gimbal.gimbal_attitude_reader import (
    AircraftAttitudeReader,
    AircraftClockReader,
)
from navpy.sim.gimbal.siyi.gimbal_angles import (
    MODE_FPV,
    MODE_LOCK,
)
from navpy.sim.gimbal.siyi.gimbal_frame_transform import (
    body_readback,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_state import (
    SiyiSimState,
)


SIYI_PHYSICS_STEP_S = 0.02
_SIYI_PHYSICS_STEP_MS = 20


class SiyiSimStepper:
    """Advance angular and zoom plants in 20 ms steps of aircraft time.

    With an aircraft clock, each call advances by the aircraft time elapsed
    since the last call (zero or more steps). Without one, by one step.
    Commands call ``catch_up`` first, so a new command never acts on aircraft
    time that passed before it was issued.
    """

    def __init__(
        self,
        state: SiyiSimState,
        attitude_reader: AircraftAttitudeReader,
    ) -> None:
        self._state = state
        self._attitude_reader = attitude_reader
        self._clock = AircraftStepClock(_SIYI_PHYSICS_STEP_MS)

    def advance(self) -> None:
        """One runner slot: pay owed aircraft time (bounded per call)."""
        self._step(command=False)

    def catch_up(self) -> None:
        """Before a command: pay all owed aircraft time; never a wall step."""
        self._step(command=True)

    def _step(self, *, command: bool) -> None:
        with self._state.lock:
            # Sample and clock are consumed together, so two callers can
            # never apply the same elapsed time twice or in reverse order.
            vehicle_attitude, boot_ms = self._read()
            if command and boot_ms is None:
                return
            steps = self._clock.steps_for(boot_ms, pay_all=command)
            if vehicle_attitude is not None:
                self._state.angular.set_vehicle_attitude(vehicle_attitude)
            for _ in range(steps):
                self._state.angular.advance(SIYI_PHYSICS_STEP_S)
                self._state.zoom.advance(SIYI_PHYSICS_STEP_S)
            if command:
                # The part-step belongs to the old command, not the new one.
                remainder_s = self._clock.settle_remainder_ms() / 1000.0
                if remainder_s > 0.0:
                    self._state.angular.advance(remainder_s)
                    self._state.zoom.advance(remainder_s)
            # Stamped after integration, under the lock: never goes backwards.
            wall_timestamp_s = rt.time()
            monotonic_s = rt.monotonic()
            angular = self._state.angular.snapshot()
            self._state.zoom_sample_seq += 1
            self._state.zoom_updated_monotonic_s = monotonic_s
            self._state.data = replace(
                self._state.data,
                att=body_readback(angular),
                roll_stabilize=(angular.motion_mode != MODE_FPV),
                pitch_stabilize=True,
                timestamp_s=wall_timestamp_s,
                reference_aircraft_attitude=(
                    angular.vehicle_attitude
                    if angular.motion_mode == MODE_LOCK
                    else None
                ),
            )


    def _read(self) -> tuple[Attitude | None, int | None]:
        if isinstance(self._attitude_reader, AircraftClockReader):
            timed = self._attitude_reader.read_timed()
            if timed is not None:
                return timed
        return self._attitude_reader.read(), None


__all__ = ["SIYI_PHYSICS_STEP_S", "SiyiSimStepper"]
