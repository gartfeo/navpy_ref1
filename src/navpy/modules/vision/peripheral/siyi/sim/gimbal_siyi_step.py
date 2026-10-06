"""One fixed physical update and readback publication for SIYI simulation."""

from __future__ import annotations

import time
from dataclasses import replace

from navpy.modules.vision.gimbal_attitude_reader import AircraftAttitudeReader
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    MODE_FPV,
    MODE_LOCK,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_frame_transform import (
    body_readback,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)


SIYI_PHYSICS_STEP_S = 0.02


class SiyiSimStepper:
    """Advance angular and zoom plants by exactly 20 ms."""

    def __init__(
        self,
        state: SiyiSimState,
        attitude_reader: AircraftAttitudeReader,
    ) -> None:
        self._state = state
        self._attitude_reader = attitude_reader

    def advance(self) -> None:
        vehicle_attitude = self._attitude_reader.read()
        wall_timestamp_s = time.time()
        monotonic_s = time.monotonic()
        with self._state.lock:
            if vehicle_attitude is not None:
                self._state.angular.set_vehicle_attitude(vehicle_attitude)
            self._state.angular.advance(SIYI_PHYSICS_STEP_S)
            self._state.zoom.advance(SIYI_PHYSICS_STEP_S)
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


__all__ = ["SIYI_PHYSICS_STEP_S", "SiyiSimStepper"]
