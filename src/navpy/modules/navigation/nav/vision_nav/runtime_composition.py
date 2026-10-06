"""Exact adapters and construction for final approach."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass

from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.command_executor import (
    FinalApproachCommandExecutor,
    FinalApproachExecutorPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    FinalApproachCommandFreshness,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    FinalApproachCommandHold,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    FinalApproachCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    FinalApproachCommandReset,
    FinalApproachCommandResetPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    FinalApproachAttitudeActuator,
    FinalApproachCommandTransaction,
)
from navpy.modules.navigation.nav.vision_nav.confirmation import (
    FinalApproachConfirmation,
    FinalApproachConfirmationPorts,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    FinalApproachDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.diagnostics import (
    NavigationFinalApproachDiagnostics,
    FinalApproachDiagnosticReader,
    FinalApproachDiagnosticSnapshot,
)
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.modules.navigation.nav.vision_nav.ingress import (
    FinalApproachIngress,
    FinalApproachIngressPorts,
)
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.navigation.nav.vision_nav.runtime import (
    FinalApproachCommandWorkRuntime,
    FinalApproachSessionRuntime,
    VisionNavRuntime,
)
from navpy.modules.navigation.nav.vision_nav.runtime_state import FinalApproachRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.source_time_adapter import (
    PoseCadenceFinalApproachSourceTimeObserver,
)
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector


@dataclass(frozen=True)
class FinalApproachRuntimeComposition:
    runtime: VisionNavRuntime
    confirmation: FinalApproachConfirmation


class FinalApproachVehicleActuator:
    def __init__(self, set_attitude: Callable[..., None]) -> None:
        self._set_attitude = set_attitude

    def issue(
        self,
        roll_deg: float,
        pitch_deg: float,
        throttle: float | None,
    ) -> None:
        self._set_attitude(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            yaw=None,
            thr=throttle,
        )


class FinalApproachVehicleDiagnosticReader:
    def __init__(
        self,
        attitude: Callable[[], Attitude | None],
        location: Callable[[], Location | None],
    ) -> None:
        self._attitude = attitude
        self._location = location

    def read(self) -> FinalApproachDiagnosticSnapshot:
        return FinalApproachDiagnosticSnapshot(
            attitude=self._read_attitude(),
            location=self._read_location(),
        )

    def _read_attitude(self) -> Attitude | None:
        try:
            return _attitude_or_none(self._attitude())
        except Exception:  # noqa: BLE001 - optional post-command telemetry
            return None

    def _read_location(self) -> Location | None:
        try:
            return _location_or_none(self._location())
        except Exception:  # noqa: BLE001 - optional post-command telemetry
            return None


def compose_final_approach_runtime(
    *,
    lock: threading.RLock,
    slot: NavigationCommandSlot,
    sys_id: int,
    actuator: FinalApproachAttitudeActuator,
    diagnostic_reader: FinalApproachDiagnosticReader,
    law: VisionNavLaw,
    aircraft_roll_deg: Callable[[], float],
    aircraft_sequence: str,
    aircraft_degrees: bool,
    navigation_logger: NavigationLogger,
    wall_period_s: Callable[[float], float],
) -> FinalApproachRuntimeComposition:
    if type(sys_id) is not int:
        raise TypeError("final-approach vehicle target_system must be an integer")
    source_time = PoseCadenceFinalApproachSourceTimeObserver(sys_id)
    liveness = FinalApproachCommandLiveness()
    freshness = FinalApproachCommandFreshness(wall_period_s)
    hold = FinalApproachCommandHold(actuator, source_time, liveness, freshness)
    projector = FinalApproachFrameProjector(
        FinalApproachProjectionConfig(aircraft_sequence, aircraft_degrees)
    )
    epochs = SourceEpochLedger()
    mailbox = FinalApproachDiagnosticMailbox()
    status = FinalApproachRuntimeStatus()
    visual_pass = VisualPassDetector()
    postprocess_fence = FinalApproachPostprocessFence()
    command_reset = FinalApproachCommandReset(FinalApproachCommandResetPorts(
        lock,
        slot,
        mailbox,
        hold,
        law,
        visual_pass,
        status,
        liveness,
        postprocess_fence,
    ))
    ingress = FinalApproachIngress(
        FinalApproachIngressPorts(
            lock,
            slot,
            projector,
            epochs,
            mailbox,
            source_time,
            command_reset,
        )
    )
    confirmation = FinalApproachConfirmation(
        FinalApproachConfirmationPorts(lock, projector, epochs, law, aircraft_roll_deg)
    )
    transaction = FinalApproachCommandTransaction(
        law,
        visual_pass,
        actuator,
    )
    diagnostics = NavigationFinalApproachDiagnostics(
        navigation_logger,
        diagnostic_reader,
    )
    executor = FinalApproachCommandExecutor(
        FinalApproachExecutorPorts(
            slot,
            transaction,
            mailbox,
            diagnostics,
            status,
            source_time,
            hold,
            liveness,
            freshness,
            postprocess_fence,
        )
    )
    runtime = VisionNavRuntime(
        FinalApproachCommandWorkRuntime(slot, ingress, executor, hold),
        FinalApproachSessionRuntime(
            lock,
            epochs,
            command_reset,
            visual_pass,
            status,
            liveness,
        ),
    )
    return FinalApproachRuntimeComposition(runtime, confirmation)


def _attitude_or_none(value: Attitude | None) -> Attitude | None:
    if value is None:
        return None
    values = (value.pitch, value.yaw, value.roll)
    return value if all(math.isfinite(float(item)) for item in values) else None


def _location_or_none(value: Location | None) -> Location | None:
    return value


__all__ = [
    "FinalApproachRuntimeComposition",
    "FinalApproachVehicleActuator",
    "FinalApproachVehicleDiagnosticReader",
    "compose_final_approach_runtime",
]
