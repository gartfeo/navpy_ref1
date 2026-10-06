"""Public SIYI ZR10 simulator over focused control/read/lifecycle owners."""

from __future__ import annotations

import threading
from dataclasses import replace

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.vision.gimbal_attitude_reader import AircraftAttitudeReader
from navpy.modules.vision.gimbal_cadence import FixedCadenceRunner
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    GimbalAngularPlant,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_control import (
    SiyiSimControl,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_lifecycle import (
    SiyiSimLifecycle,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_readback import (
    SiyiSimReadback,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_step import (
    SIYI_PHYSICS_STEP_S,
    SiyiSimStepper,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_zoom_plant import (
    GimbalZoomPlant,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_facets import (
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
    state = SiyiSimState(
        data=replace(data, roll_stabilize=True, pitch_stabilize=True),
        angular=GimbalAngularPlant(data.att.pitch, data.att.yaw),
        zoom=GimbalZoomPlant(),
        lock=threading.Lock(),
    )
    stepper = SiyiSimStepper(state, attitude_reader)
    runner = FixedCadenceRunner(
        stepper,
        SIYI_PHYSICS_STEP_S,
        scheduler_cadence,
    )
    return SiyiSimPorts(
        SiyiSimControl(state, logger),
        SiyiSimReadback(state),
        SiyiSimLifecycle(state, runner, logger),
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
