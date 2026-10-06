"""Post-fence terminal diagnostics and compact navigation samples."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.logger.navigation_logger import NavigationLogger
from navpy.logger.log_events import LogEvent
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    TerminalCommandOutcome,
    TerminalCommandResult,
    TerminalLawEvidence,
)
from navpy.modules.navigation.nav.vision_nav.diagnostic_capture import (
    body_bearing_deg,
    copy_attitude,
    copy_camera_matrix,
    copy_location,
    debug_camera,
    debug_target,
    law_evidence_payload,
    optional_float,
)
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class TerminalDiagnosticSnapshot:
    attitude: Attitude | None
    location: Location | None


@dataclass(frozen=True)
class TerminalCommandValues:
    roll_deg: float
    pitch_deg: float
    throttle: float | None


@dataclass(frozen=True)
class TerminalCommandDiagnostic:
    """Immutable command-time telemetry prepared for later formatting."""

    frame: TerminalVisionFrame
    command: TerminalCommandValues | None
    outcome: TerminalCommandOutcome | None
    passed: bool
    snapshot: TerminalDiagnosticSnapshot | None
    target_location: Location | None
    camera_location: Location | None
    x_error: float | None
    y_error: float | None
    camera_matrix: np.ndarray | None
    event_timestamp: str
    law_evidence: TerminalLawEvidence | None


class TerminalDiagnosticReader(Protocol):
    def read(self) -> TerminalDiagnosticSnapshot: ...


class TerminalCommandDiagnostics(Protocol):
    def capture(
        self,
        frame: TerminalVisionFrame,
        target: DetectedObject | None,
        result: TerminalCommandResult | None,
    ) -> object: ...

    def record(
        self,
        diagnostic: object,
    ) -> None: ...


class NavigationTerminalDiagnostics:
    """Snapshot mutable telemetry at issue time, then format it off-thread."""

    def __init__(
        self,
        logger: NavigationLogger,
        reader: TerminalDiagnosticReader,
    ) -> None:
        self._logger = logger
        self._reader = reader

    def capture(
        self,
        frame: TerminalVisionFrame,
        target: DetectedObject | None,
        result: TerminalCommandResult | None,
    ) -> TerminalCommandDiagnostic:
        calc = None if result is None else result.calc_data
        target_location = copy_location(debug_target(target))
        camera_location = copy_location(debug_camera(target))
        needs_snapshot = calc is not None or (
            result is not None
            and result.outcome is TerminalCommandOutcome.PASS_SUPPRESSED
            and target_location is not None
        )
        snapshot = self._reader.read() if needs_snapshot else None
        return TerminalCommandDiagnostic(
            frame=frame,
            command=(
                None
                if calc is None
                else TerminalCommandValues(
                    float(calc.cmd_roll),
                    float(calc.cmd_pitch),
                    None if calc.cmd_thr is None else float(calc.cmd_thr),
                )
            ),
            outcome=None if result is None else result.outcome,
            passed=bool(result is not None and result.passed),
            snapshot=_copy_snapshot(snapshot),
            target_location=target_location,
            camera_location=camera_location,
            x_error=None if target is None else optional_float(target.pixel.u_px),
            y_error=None if target is None else optional_float(target.pixel.v_px),
            camera_matrix=copy_camera_matrix(target),
            event_timestamp=self._logger.capture_event_timestamp(),
            law_evidence=None if result is None else result.evidence,
        )

    def record(self, diagnostic: object) -> None:
        if not isinstance(diagnostic, TerminalCommandDiagnostic):
            raise TypeError("invalid terminal command diagnostic")
        if diagnostic.command is None:
            self._record_event(diagnostic)
            if diagnostic.outcome is TerminalCommandOutcome.PASS_SUPPRESSED:
                self._sample_pass_frame(
                    diagnostic.target_location,
                    diagnostic.snapshot,
                )
            return
        snapshot = diagnostic.snapshot or TerminalDiagnosticSnapshot(None, None)
        if self._record_compact(diagnostic, snapshot):
            self._record_event(diagnostic)
            self._record_law_evidence(diagnostic)

    def _sample_pass_frame(
        self,
        target_location: Location | None,
        snapshot: TerminalDiagnosticSnapshot | None,
    ) -> None:
        if target_location is None or snapshot is None:
            return
        self._logger.sample_snap(snapshot.location, target_location)

    def _record_event(
        self,
        diagnostic: TerminalCommandDiagnostic,
    ) -> None:
        frame = diagnostic.frame
        command = diagnostic.command
        self._logger.log_event(
            LogEvent.TERMINAL_CMD,
            {
                "source": frame.source_name,
                "generation": frame.source_generation,
                "task": frame.task_id,
                "obj": frame.obj_id,
                "obs_ts": frame.source_timestamp_s,
                "dt_wall_ms": "",
                "body_bearing_deg": body_bearing_deg(frame),
                "cmd_roll": "" if command is None else command.roll_deg,
                "cmd_pitch": "" if command is None else command.pitch_deg,
                "cmd_thr": "" if command is None else command.throttle,
                "issued": diagnostic.outcome is TerminalCommandOutcome.ISSUED,
                "passed": diagnostic.passed,
            },
            timestamp=diagnostic.event_timestamp,
        )

    def _record_law_evidence(self, diagnostic: TerminalCommandDiagnostic) -> None:
        evidence = diagnostic.law_evidence
        if evidence is None:
            return
        self._logger.log_event(
            LogEvent.TERMINAL_RESPONSE_STATE,
            {
                "source": diagnostic.frame.source_name,
                "obs_ts": diagnostic.frame.source_timestamp_s,
                "cmd_roll_deg": diagnostic.command.roll_deg,
                "cmd_pitch_deg": diagnostic.command.pitch_deg,
                **law_evidence_payload(evidence),
            },
            timestamp=diagnostic.event_timestamp,
        )

    def _record_compact(
        self,
        diagnostic: TerminalCommandDiagnostic,
        snapshot: TerminalDiagnosticSnapshot,
    ) -> bool:
        frame = diagnostic.frame
        target_location = diagnostic.target_location
        command = diagnostic.command
        if command is None:  # pragma: no cover - caller narrows this branch
            return False
        distance = (
            GeoRefCalc.calculate_distance(snapshot.location, target_location)
            if snapshot.location is not None and target_location is not None
            else 0.0
        )
        attitude = snapshot.attitude
        return self._logger.log(
            c_loc=snapshot.location,
            t_loc=target_location,
            distance=distance,
            cmd_roll=command.roll_deg,
            cmd_pitch=command.pitch_deg,
            yaw_error=body_bearing_deg(frame),
            pitch_error=math.degrees(math.atan2(
                frame.control_z,
                math.hypot(frame.control_x, frame.control_y),
            )),
            actual_roll=None if attitude is None else attitude.roll,
            actual_pitch=None if attitude is None else attitude.pitch,
            x_error=diagnostic.x_error,
            y_error=diagnostic.y_error,
            detect_c_loc=diagnostic.camera_location,
            detect_t_loc_debug=target_location,
            k=diagnostic.camera_matrix,
        )


def _copy_snapshot(
    snapshot: TerminalDiagnosticSnapshot | None,
) -> TerminalDiagnosticSnapshot | None:
    if snapshot is None:
        return None
    return TerminalDiagnosticSnapshot(
        attitude=copy_attitude(snapshot.attitude),
        location=copy_location(snapshot.location),
    )


__all__ = [
    "NavigationTerminalDiagnostics",
    "TerminalCommandDiagnostic",
    "TerminalCommandValues",
    "TerminalCommandDiagnostics",
    "TerminalDiagnosticReader",
    "TerminalDiagnosticSnapshot",
]
