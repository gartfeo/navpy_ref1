"""Public SIYI ZR10 simulator over focused control/read/lifecycle owners."""

from __future__ import annotations

from dataclasses import replace

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.sim.gimbal.gimbal_attitude_reader import AircraftAttitudeReader
from navpy.sim.gimbal.gimbal_cadence import FixedCadenceRunner
from navpy.core.ports.gimbal_abc import GimbalAbc, GimbalData
from navpy.plugins.gimbal.siyi.siyi_convention import SIYI_GIMBAL_SEQ
from navpy.sim.gimbal.siyi.gimbal_angles import (
    GimbalAngularPlant,
    MODE_FOLLOW,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_control import (
    SiyiSimControl,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_lifecycle import (
    SiyiSimLifecycle,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_readback import (
    SiyiSimReadback,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_state import (
    SiyiSimState,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_step import (
    SIYI_PHYSICS_STEP_S,
    SiyiSimStepper,
)
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import (
    GimbalZoomPlant,
)
from navpy.sim.gimbal.siyi.gimbal_siyi_facets import (
    SiyiSimControlFacet,
    SiyiSimLifecycleFacet,
    SiyiSimPorts,
    SiyiSimReadbackFacet,
)


def build_siyi_sim(
    data: GimbalData,
    attitude_reader: AircraftAttitudeReader,
    logger: ILogger,
    scheduler_cadence: SchedulerCadence | None,
) -> SiyiSimPorts:
    # The mount's yaw is relative to the body. Left in the plant's LOCK, start's switch to FOLLOW
    # reads it as a world yaw and takes the aircraft's heading off it: a NavPy restarted in flight
    # would keep looking off to the side.
    angular = GimbalAngularPlant(data.att.pitch, data.att.yaw)
    angular.set_motion_mode(MODE_FOLLOW)
    state = SiyiSimState(
        data=replace(data, roll_stabilize=True, pitch_stabilize=True, g_seq=SIYI_GIMBAL_SEQ),
        angular=angular,
        zoom=GimbalZoomPlant(),
        lock=rt.Lock(),
    )
    stepper = SiyiSimStepper(state, attitude_reader)
    runner = FixedCadenceRunner(
        stepper,
        SIYI_PHYSICS_STEP_S,
        scheduler_cadence,
    )
    return SiyiSimPorts(
        SiyiSimControl(state, logger, stepper.catch_up),
        SiyiSimReadback(state),
        SiyiSimLifecycle(state, runner, logger, stepper.catch_up),
    )


class GimbalSiyiSim(
    SiyiSimLifecycleFacet,
    SiyiSimReadbackFacet,
    SiyiSimControlFacet,
    GimbalAbc,
):
    """One-field compatibility adapter matching real SIYI capabilities."""

    def __init__(
        self,
        data: GimbalData,
        attitude_reader: AircraftAttitudeReader,
        logger: ILogger,
        scheduler_cadence: SchedulerCadence | None = None,
    ) -> None:
        self._parts = build_siyi_sim(
            data,
            attitude_reader,
            logger,
            scheduler_cadence,
        )

__all__ = ["GimbalSiyiSim", "SiyiSimPorts", "build_siyi_sim"]
