"""Target-selection, coordinate, and SNAP episode evidence gates."""

from __future__ import annotations

import re
import time
from datetime import datetime
from pathlib import Path

from eval_navigation_models import (
    EvidenceGateResult,
    SelectionEvidence,
    TargetExpectation,
    TargetLocation,
)
from eval_navigation_scoring import horizontal_distance_m


TARGET_SNAP_BIND_TIMEOUT_S = 2.0
_CATALOG_RE = re.compile(r"T(?P<task>\d+):\s*wp:(?P<wp>\d+)\(seq:(?P<seq>\d+)\)")
_TARGET_RE = re.compile(
    r"TARGET:\s*T(?P<task>\d+)\s*\(tracking obj_id=(?P<obj>-?\d+)\)"
)
_TARGET_LOCATION_RE = re.compile(
    r"t_l\s*\([^)]*\):\s*"
    r"(?P<lat>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<lon>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<alt>[-+]?\d+(?:\.\d+)?)"
)
_LOG_TIME_RE = re.compile(
    r"^(?P<stamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3,6})"
)
_TARGET_SNAP_MISSING = "accepted target episode has no subsequent navigation SNAP"


def parse_selection_evidence(log_text: str) -> SelectionEvidence:
    """Parse the first accepted target episode and its mission catalog row."""
    target = _TARGET_RE.search(log_text)
    catalog = None
    if target is not None:
        selected_task = int(target.group("task"))
        catalog = next(
            (
                candidate
                for candidate in _CATALOG_RE.finditer(log_text, 0, target.start())
                if int(candidate.group("task")) == selected_task
            ),
            None,
        )
    next_target = (
        _TARGET_RE.search(log_text, target.end()) if target is not None else None
    )
    episode_end = next_target.start() if next_target is not None else len(log_text)
    location = (
        _TARGET_LOCATION_RE.search(log_text, target.end(), episode_end)
        if target is not None
        else None
    )
    return SelectionEvidence(
        catalog_wp=int(catalog.group("wp")) if catalog else None,
        catalog_seq=int(catalog.group("seq")) if catalog else None,
        task_id=int(target.group("task")) if target else None,
        obj_id=int(target.group("obj")) if target else None,
        fallback_location_registered="SimT(" in log_text,
        event_wall_time_s=_event_time(log_text, target),
        target_lat_deg=float(location.group("lat")) if location else None,
        target_lon_deg=float(location.group("lon")) if location else None,
        target_abs_alt_m=float(location.group("alt")) if location else None,
    )


def _event_time(log_text: str, target: re.Match[str] | None) -> float | None:
    if target is None:
        return None
    line_start = log_text.rfind("\n", 0, target.start()) + 1
    timestamp = _LOG_TIME_RE.match(log_text[line_start:])
    if timestamp is None:
        return None
    return datetime.strptime(
        timestamp.group("stamp"),
        "%Y-%m-%d %H:%M:%S,%f",
    ).timestamp()


def target_episode_binding_error(
    log_text: str,
    evidence: SelectionEvidence | None,
) -> str | None:
    """Require the accepted target to remain selected through its SNAP."""
    targets = list(_TARGET_RE.finditer(log_text))
    if evidence is None or evidence.task_id is None or evidence.obj_id is None:
        return "SNAP has no accepted target episode"
    if not targets:
        return "accepted target episode is absent from the navigation log"
    accepted = targets[0]
    accepted_identity = (
        int(accepted.group("task")),
        int(accepted.group("obj")),
    )
    if accepted_identity != (evidence.task_id, evidence.obj_id):
        return (
            "accepted target evidence does not match the first logged episode: "
            f"evidence=T{evidence.task_id}/obj{evidence.obj_id}, "
            f"log=T{accepted_identity[0]}/obj{accepted_identity[1]}"
        )
    snap = re.search(r"SNAP\(", log_text[accepted.end() :])
    if snap is None:
        return _TARGET_SNAP_MISSING
    snap_start = accepted.end() + snap.start()
    if len(targets) > 1 and targets[1].start() < snap_start:
        switched = targets[1]
        return (
            "target switched before certified SNAP: "
            f"accepted T{evidence.task_id}/obj{evidence.obj_id}, then "
            f"T{int(switched.group('task'))}/obj{int(switched.group('obj'))}"
        )
    return None


def await_target_snap_binding(
    navigation_path: Path,
    evidence: SelectionEvidence | None,
    *,
    timeout_s: float = TARGET_SNAP_BIND_TIMEOUT_S,
) -> str | None:
    """Wait for the verbose SNAP paired with the compact SNAP flush."""
    deadline = time.time() + timeout_s
    while True:
        try:
            text = navigation_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            text = ""
        error = target_episode_binding_error(text, evidence)
        if error != _TARGET_SNAP_MISSING or time.time() >= deadline:
            return error
        time.sleep(0.01)


def selected_location_from_evidence(
    evidence: SelectionEvidence,
    *,
    home_abs_alt_m: float,
) -> TargetLocation | None:
    """Convert logged absolute target coordinates to the expected record."""
    if (
        evidence.target_lat_deg is None
        or evidence.target_lon_deg is None
        or evidence.target_abs_alt_m is None
    ):
        return None
    return TargetLocation(
        evidence.target_lat_deg,
        evidence.target_lon_deg,
        evidence.target_abs_alt_m - home_abs_alt_m,
        evidence.target_abs_alt_m,
    )


def validate_pre_snap_evidence(
    expectation: TargetExpectation,
    evidence: SelectionEvidence,
    selected_location: TargetLocation | None,
    *,
    configured_rel_alt_m: float,
    coordinate_tolerance_m: float,
    altitude_tolerance_m: float,
) -> EvidenceGateResult:
    """Validate target identity, coordinate, and altitude before scoring SNAP."""
    errors = _identity_errors(expectation, evidence)
    coordinate_error: float | None = None
    altitude_error: float | None = None
    if selected_location is None:
        errors.append("selected target coordinate could not be resolved")
    else:
        coordinate_error = horizontal_distance_m(
            expectation.location.lat_deg,
            expectation.location.lon_deg,
            selected_location.lat_deg,
            selected_location.lon_deg,
        )
        altitude_error = abs(
            expectation.location.abs_alt_m - selected_location.abs_alt_m
        )
        if coordinate_error > coordinate_tolerance_m:
            errors.append(
                f"selected target coordinate differs by {coordinate_error:.3f} m "
                f"(tolerance {coordinate_tolerance_m:.3f} m)"
            )
        if altitude_error > altitude_tolerance_m:
            errors.append(
                f"selected target altitude differs by {altitude_error:.3f} m "
                f"(tolerance {altitude_tolerance_m:.3f} m)"
            )
    configured_error = abs(
        configured_rel_alt_m - expectation.location.rel_alt_m
    )
    if configured_error > altitude_tolerance_m:
        errors.append(
            "configured target relative altitude differs by "
            f"{configured_error:.3f} m"
        )
    return EvidenceGateResult(
        passed=not errors,
        errors=tuple(errors),
        coordinate_error_m=coordinate_error,
        altitude_error_m=altitude_error,
    )


def _identity_errors(
    expectation: TargetExpectation,
    evidence: SelectionEvidence,
) -> list[str]:
    errors: list[str] = []
    if evidence.catalog_wp != expectation.target_wp:
        errors.append(
            f"target catalog WP mismatch: expected {expectation.target_wp}, "
            f"got {evidence.catalog_wp}"
        )
    if evidence.catalog_seq != expectation.mission_seq:
        errors.append(
            "target catalog mission-seq mismatch: expected "
            f"{expectation.mission_seq}, got {evidence.catalog_seq}"
        )
    if evidence.task_id is None or evidence.obj_id is None:
        errors.append("no selected TARGET identity was logged")
    elif evidence.task_id != expectation.expected_task_id:
        errors.append(
            f"selected task mismatch: expected T{expectation.expected_task_id}, "
            f"got T{evidence.task_id}"
        )
    if evidence.obj_id is not None and evidence.obj_id != expectation.expected_obj_id:
        label = "default OOI was selected" if evidence.fallback_location_registered else "selected object mismatch"
        errors.append(f"{label} (obj_id={evidence.obj_id}); expected mission target obj_id=0")
    return errors
