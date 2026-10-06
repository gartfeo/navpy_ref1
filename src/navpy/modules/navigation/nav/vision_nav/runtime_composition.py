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
    TerminalCommandExecutor,
    TerminalExecutorPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_freshness import (
    TerminalCommandFreshness,
)
from navpy.modules.navigation.nav.vision_nav.command_hold import (
    TerminalCommandHold,
)
from navpy.modules.navigation.nav.vision_nav.command_liveness import (
    TerminalCommandLiveness,
)
from navpy.modules.navigation.nav.vision_nav.command_postprocess import (
    TerminalPostprocessFence,
)
from navpy.modules.navigation.nav.vision_nav.command_reset import (
    TerminalCommandReset,
    TerminalCommandResetPorts,
)
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    TerminalAttitudeActuator,
    TerminalCommandTransaction,
)
from navpy.modules.navigation.nav.vision_nav.confirmation import (
    TerminalConfirmation,
    TerminalConfirmationPorts,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_mailbox import (
    TerminalDiagnosticMailbox,
)
from navpy.modules.navigation.nav.vision_nav.diagnostics import (
    NavigationTerminalDiagnostics,
    TerminalDiagnosticReader,
    TerminalDiagnosticSnapshot,
)
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    TerminalFrameProjector,
    TerminalProjectionConfig,
)
from navpy.modules.navigation.nav.vision_nav.ingress import (
    TerminalIngress,
    TerminalIngressPorts,
)
from navpy.modules.navigation.nav.vision_nav.law import VisionNavLaw
from navpy.modules.navigation.nav.vision_nav.runtime import (
    TerminalCommandWorkRuntime,
    TerminalSessionRuntime,
    VisionNavRuntime,
)
from navpy.modules.navigation.nav.vision_nav.runtime_state import TerminalRuntimeStatus
from navpy.modules.navigation.nav.vision_nav.source_epoch import SourceEpochLedger
from navpy.modules.navigation.nav.vision_nav.source_time_adapter import (
    PoseCadenceTerminalSourceTimeObserver,
)
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector


@dataclass(frozen=True)
class TerminalRuntimeComposition:
    runtime: VisionNavRuntime
    confirmation: TerminalConfirmation


class TerminalVehicleActuator:
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


class TerminalVehicleDiagnosticReader:
    def __init__(
        self,
        attitude: Callable[[], Attitude | None],
        location: Callable[[], Location | None],
    ) -> None:
        self._attitude = attitude
        self._location = location

    def read(self) -> TerminalDiagnosticSnapshot:
        return TerminalDiagnosticSnapshot(
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


def compose_terminal_runtime(
    *,
    lock: threading.RLock,
    slot: NavigationCommandSlot,
    sys_id: int,
    actuator: TerminalAttitudeActuator,
    diagnostic_reader: TerminalDiagnosticReader,
    law: VisionNavLaw,
    aircraft_roll_deg: Callable[[], float],
    aircraft_sequence: str,
    aircraft_degrees: bool,
    navigation_logger: NavigationLogger,
    wall_period_s: Callable[[float], float],
) -> TerminalRuntimeComposition:
    if type(sys_id) is not int:
        raise TypeError("terminal vehicle target_system must be an integer")
    source_time = PoseCadenceTerminalSourceTimeObserver(sys_id)
    liveness = TerminalCommandLiveness()
    freshness = TerminalCommandFreshness(wall_period_s)
    hold = TerminalCommandHold(actuator, source_time, liveness, freshness)
    projector = TerminalFrameProjector(
        TerminalProjectionConfig(aircraft_sequence, aircraft_degrees)
    )
    epochs = SourceEpochLedger()
    mailbox = TerminalDiagnosticMailbox()
    status = TerminalRuntimeStatus()
    visual_pass = VisualPassDetector()
    postprocess_fence = TerminalPostprocessFence()
    command_reset = TerminalCommandReset(TerminalCommandResetPorts(
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
    ingress = TerminalIngress(
        TerminalIngressPorts(
            lock,
            slot,
            projector,
            epochs,
            mailbox,
            source_time,
            command_reset,
        )
    )
    confirmation = TerminalConfirmation(
        TerminalConfirmationPorts(lock, projector, epochs, law, aircraft_roll_deg)
    )
    transaction = TerminalCommandTransaction(
        law,
        visual_pass,
        actuator,
    )
    diagnostics = NavigationTerminalDiagnostics(
        navigation_logger,
        diagnostic_reader,
    )
    executor = TerminalCommandExecutor(
        TerminalExecutorPorts(
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
        TerminalCommandWorkRuntime(slot, ingress, executor, hold),
        TerminalSessionRuntime(
            lock,
            epochs,
            command_reset,
            visual_pass,
            status,
            liveness,
        ),
    )
    return TerminalRuntimeComposition(runtime, confirmation)


def _attitude_or_none(value: Attitude | None) -> Attitude | None:
    if value is None:
        return None
    values = (value.pitch, value.yaw, value.roll)
    return value if all(math.isfinite(float(item)) for item in values) else None


def _location_or_none(value: Location | None) -> Location | None:
    return value


__all__ = [
    "TerminalRuntimeComposition",
    "TerminalVehicleActuator",
    "TerminalVehicleDiagnosticReader",
    "compose_terminal_runtime",
]
