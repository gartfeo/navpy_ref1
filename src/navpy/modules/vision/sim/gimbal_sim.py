"""Generic fixed-cadence simulated gimbal."""

from __future__ import annotations

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.vision.gimbal_attitude_reader import AircraftAttitudeReader
from navpy.modules.vision.gimbal_cadence import FixedCadenceRunner
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData
from navpy.modules.vision.sim.gimbal_stabilizer import GimbalStabilizer


GIMBAL_SIM_STEP_S = 0.02


class GimbalSim(GimbalAbc):
    """Stabilize a generic mount at the same fixed 50 Hz physics cadence."""

    def __init__(
        self,
        data: GimbalData,
        attitude_reader: AircraftAttitudeReader,
        uas_seq: str = "ZYX",
        scheduler_cadence: SchedulerCadence | None = None,
    ) -> None:
        self._stabilizer = GimbalStabilizer(data, attitude_reader, uas_seq)
        self._runner = FixedCadenceRunner(
            self._stabilizer,
            GIMBAL_SIM_STEP_S,
            scheduler_cadence,
        )

    def start(self) -> None:
        self._runner.start()

    def stop(self) -> bool:
        return self._runner.stop()

    def is_alive(self) -> bool:
        return self._runner.is_running

    def raise_if_failed(self) -> None:
        self._runner.raise_if_failed()

    def get_data(self) -> GimbalData:
        return self._stabilizer.get_data()

    def set_att(self, att: Attitude) -> None:
        self._stabilizer.set_att(att)

__all__ = ["GIMBAL_SIM_STEP_S", "GimbalSim"]
