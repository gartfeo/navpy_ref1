"""Exact adapters and construction for guidance."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.approach.logs.navigation_logger import NavigationLogger
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.command.navigation_command_slot import NavigationCommandSlot
from navpy.core.approach.vision_nav.command_executor import (
    GuidanceCommandExecutor,
    GuidanceExecutorPorts,
)
from navpy.core.approach.vision_nav.command_freshness import (
    GuidanceCommandFreshness,
    GuidanceFreshnessCheck,
)
from navpy.core.approach.vision_nav.command_hold import (
    GuidanceCommandHold,
)
from navpy.core.approach.vision_nav.command_liveness import (
    GuidanceEpisodeFailures,
    GuidanceCommandLiveness,
)
from navpy.core.approach.vision_nav.command_postprocess import (
    GuidancePostprocessFence,
)
from navpy.core.approach.vision_nav.command_reset import (
    GuidanceCommandReset,
    GuidanceCommandResetPorts,
)
from navpy.core.approach.vision_nav.command_transaction import (
    GuidanceAttitudeActuator,
    GuidanceCommandTransaction,
)
from navpy.core.approach.vision_nav.confirmation import (
    GuidanceConfirmation,
    GuidanceConfirmationPorts,
)
from navpy.core.approach.vision_nav.diagnostic_mailbox import (
    GuidanceDiagnosticMailbox,
)
from navpy.core.approach.vision_nav.diagnostics import (
    NavigationGuidanceDiagnostics,
    GuidanceDiagnosticReader,
    GuidanceDiagnosticSnapshot,
)
from navpy.core.approach.vision_nav.frame_projection import (
    GuidanceFrameProjector,
    GuidanceProjectionConfig,
)
from navpy.core.approach.vision_nav.ingress import (
    GuidanceIngress,
    GuidanceIngressPorts,
)
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from navpy.core.approach.vision_nav.runtime import (
    GuidanceCommandWorkRuntime,
    GuidanceSessionRuntime,
    VisionNavRuntime,
)
from navpy.core.approach.vision_nav.runtime_state import GuidanceRuntimeStatus
from navpy.core.approach.vision_nav.source_epoch import SourceEpochLedger
from navpy.core.approach.vision_nav.source_time_adapter import (
    PoseCadenceGuidanceSourceTimeObserver,
)
from navpy.core.approach.vision_nav.dock_outcome import GuidanceDockOutcome
from navpy.core.approach.vision_nav.docked_evidence import GuidanceDockedEvidence
from navpy.core.approach.vision_nav.episode_latch import GuidanceEpisodeLatch, TerminalObserver
from navpy.core.approach.vision_nav.visual_pass import VisualPassDetector


@dataclass(frozen=True)
class GuidanceRuntimeComposition:
    runtime: VisionNavRuntime
    confirmation: GuidanceConfirmation
    dock_outcome: GuidanceDockOutcome


class GuidanceVehicleActuator:
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


class GuidanceVehicleDiagnosticReader:
    def __init__(
        self,
        attitude: Callable[[], Attitude | None],
        location: Callable[[], Location | None],
    ) -> None:
        self._attitude = attitude
        self._location = location

    def read(self) -> GuidanceDiagnosticSnapshot:
        return GuidanceDiagnosticSnapshot(
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


def compose_guidance_runtime(
    *,
    lock: rt.RLock,
    slot: NavigationCommandSlot,
    sys_id: int,
    actuator: GuidanceAttitudeActuator,
    diagnostic_reader: GuidanceDiagnosticReader,
    law: VisionNavLaw,
    aircraft_roll_deg: Callable[[], float],
    aircraft_sequence: str,
    aircraft_degrees: bool,
    navigation_logger: NavigationLogger,
    wall_period_s: Callable[[float], float],
    on_terminal: TerminalObserver | None = None,
    report_stale_hold: Callable[[GuidanceFreshnessCheck], None] = (
        lambda check: None
    ),
    report_stale_command: Callable[[GuidanceFreshnessCheck], None] = (
        lambda check: None
    ),
) -> GuidanceRuntimeComposition:
    if type(sys_id) is not int:
        raise TypeError("guidance vehicle target_system must be an integer")
    source_time = PoseCadenceGuidanceSourceTimeObserver(sys_id)
    liveness = GuidanceCommandLiveness()
    episode = GuidanceEpisodeLatch(on_terminal)
    worker_failures = GuidanceEpisodeFailures(lock, episode, liveness)
    freshness = GuidanceCommandFreshness(wall_period_s)
    hold = GuidanceCommandHold(
        actuator, source_time, worker_failures, freshness,
        report_stale=report_stale_hold,
    )
    projector = GuidanceFrameProjector(
        GuidanceProjectionConfig(aircraft_sequence, aircraft_degrees)
    )
    epochs = SourceEpochLedger()
    mailbox = GuidanceDiagnosticMailbox()
    status = GuidanceRuntimeStatus()
    visual_pass = VisualPassDetector()
    postprocess_fence = GuidancePostprocessFence()
    diagnostics = NavigationGuidanceDiagnostics(
        navigation_logger,
        diagnostic_reader,
    )
    docked_evidence = GuidanceDockedEvidence(
        lock,
        diagnostics.record_docked,
        postprocess_fence,
        diagnostics.record_dock_armed,
        diagnostics.record_dock_disarmed,
    )
    command_reset = GuidanceCommandReset(GuidanceCommandResetPorts(
        lock,
        slot,
        mailbox,
        hold,
        law,
        visual_pass,
        status,
        liveness,
        postprocess_fence,
        docked_evidence,
        episode,
    ))
    ingress = GuidanceIngress(
        GuidanceIngressPorts(
            lock,
            slot,
            projector,
            epochs,
            mailbox,
            source_time,
            command_reset,
        )
    )
    confirmation = GuidanceConfirmation(
        GuidanceConfirmationPorts(lock, projector, epochs, law, aircraft_roll_deg)
    )
    transaction = GuidanceCommandTransaction(
        law,
        visual_pass,
        actuator,
        episode,
    )
    executor = GuidanceCommandExecutor(
        GuidanceExecutorPorts(
            slot,
            transaction,
            mailbox,
            diagnostics,
            status,
            source_time,
            hold,
            worker_failures,
            freshness,
            postprocess_fence,
            report_stale_command,
        )
    )
    runtime = VisionNavRuntime(
        GuidanceCommandWorkRuntime(
            slot, ingress, executor, hold, docked_evidence,
        ),
        GuidanceSessionRuntime(
            lock,
            epochs,
            command_reset,
            episode,
            status,
            liveness,
            docked_evidence,
        ),
    )
    dock_outcome = GuidanceDockOutcome(lock, episode, diagnostics.record_dock_failed)
    return GuidanceRuntimeComposition(runtime, confirmation, dock_outcome)


def _attitude_or_none(value: Attitude | None) -> Attitude | None:
    if value is None:
        return None
    values = (value.pitch, value.yaw, value.roll)
    return value if all(math.isfinite(float(item)) for item in values) else None


def _location_or_none(value: Location | None) -> Location | None:
    return value


__all__ = [
    "GuidanceRuntimeComposition",
    "GuidanceVehicleActuator",
    "GuidanceVehicleDiagnosticReader",
    "compose_guidance_runtime",
]
