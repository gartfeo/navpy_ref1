"""POI-selection, coordinate, and SNAP episode evidence gates."""

from __future__ import annotations

import re
import time
from datetime import datetime
from pathlib import Path

from eval_navigation_models import (
    EvidenceGateResult,
    SelectionEvidence,
    PoiExpectation,
    PoiLocation,
)
from eval_navigation_scoring import horizontal_distance_m


POI_SNAP_BIND_TIMEOUT_S = 2.0
_CATALOG_RE = re.compile(r"P(?P<task>\d+):\s*wp:(?P<wp>\d+)\(seq:(?P<seq>\d+)\)")
_POI_RE = re.compile(
    r"POI:\s*P(?P<task>\d+)\s*\(tracking obj_id=(?P<obj>-?\d+)\)"
)
_POI_LOCATION_RE = re.compile(
    r"t_l\s*\([^)]*\):\s*"
    r"(?P<lat>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<lon>[-+]?\d+(?:\.\d+)?),\s*"
    r"(?P<alt>[-+]?\d+(?:\.\d+)?)"
)
_LOG_TIME_RE = re.compile(
    r"^(?P<stamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3,6})"
)
_POI_SNAP_MISSING = "accepted POI episode has no subsequent navigation SNAP"


def parse_selection_evidence(log_text: str) -> SelectionEvidence:
    """Parse the first accepted POI episode and its mission catalog row."""
    poi = _POI_RE.search(log_text)
    catalog = None
    if poi is not None:
        selected_task = int(poi.group("task"))
        catalog = next(
            (
                candidate
                for candidate in _CATALOG_RE.finditer(log_text, 0, poi.start())
                if int(candidate.group("task")) == selected_task
            ),
            None,
        )
    next_poi = (
        _POI_RE.search(log_text, poi.end()) if poi is not None else None
    )
    episode_end = next_poi.start() if next_poi is not None else len(log_text)
    location = (
        _POI_LOCATION_RE.search(log_text, poi.end(), episode_end)
        if poi is not None
        else None
    )
    return SelectionEvidence(
        catalog_wp=int(catalog.group("wp")) if catalog else None,
        catalog_seq=int(catalog.group("seq")) if catalog else None,
        task_id=int(poi.group("task")) if poi else None,
        obj_id=int(poi.group("obj")) if poi else None,
        fallback_location_registered="SimT(" in log_text,
        event_wall_time_s=_event_time(log_text, poi),
        poi_lat_deg=float(location.group("lat")) if location else None,
        poi_lon_deg=float(location.group("lon")) if location else None,
        poi_abs_alt_m=float(location.group("alt")) if location else None,
    )


def _event_time(log_text: str, poi: re.Match[str] | None) -> float | None:
    if poi is None:
        return None
    line_start = log_text.rfind("\n", 0, poi.start()) + 1
    timestamp = _LOG_TIME_RE.match(log_text[line_start:])
    if timestamp is None:
        return None
    return datetime.strptime(
        timestamp.group("stamp"),
        "%Y-%m-%d %H:%M:%S,%f",
    ).timestamp()


def poi_episode_binding_error(
    log_text: str,
    evidence: SelectionEvidence | None,
) -> str | None:
    """Require the accepted POI to remain selected through its SNAP."""
    pois = list(_POI_RE.finditer(log_text))
    if evidence is None or evidence.task_id is None or evidence.obj_id is None:
        return "SNAP has no accepted POI episode"
    if not pois:
        return "accepted POI episode is absent from the navigation log"
    accepted = pois[0]
    accepted_identity = (
        int(accepted.group("task")),
        int(accepted.group("obj")),
    )
    if accepted_identity != (evidence.task_id, evidence.obj_id):
        return (
            "accepted POI evidence does not match the first logged episode: "
            f"evidence=T{evidence.task_id}/obj{evidence.obj_id}, "
            f"log=T{accepted_identity[0]}/obj{accepted_identity[1]}"
        )
    snap = re.search(r"SNAP\(", log_text[accepted.end() :])
    if snap is None:
        return _POI_SNAP_MISSING
    snap_start = accepted.end() + snap.start()
    if len(pois) > 1 and pois[1].start() < snap_start:
        switched = pois[1]
        return (
            "POI switched before certified SNAP: "
            f"accepted P{evidence.task_id}/obj{evidence.obj_id}, then "
            f"P{int(switched.group('task'))}/obj{int(switched.group('obj'))}"
        )
    return None


def await_poi_snap_binding(
    navigation_path: Path,
    evidence: SelectionEvidence | None,
    *,
    timeout_s: float = POI_SNAP_BIND_TIMEOUT_S,
) -> str | None:
    """Wait for the verbose SNAP paired with the compact SNAP flush."""
    deadline = time.time() + timeout_s
    while True:
        try:
            text = navigation_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            text = ""
        error = poi_episode_binding_error(text, evidence)
        if error != _POI_SNAP_MISSING or time.time() >= deadline:
            return error
        time.sleep(0.01)


def selected_location_from_evidence(
    evidence: SelectionEvidence,
    *,
    home_abs_alt_m: float,
) -> PoiLocation | None:
    """Convert logged absolute POI coordinates to the expected record."""
    if (
        evidence.poi_lat_deg is None
        or evidence.poi_lon_deg is None
        or evidence.poi_abs_alt_m is None
    ):
        return None
    return PoiLocation(
        evidence.poi_lat_deg,
        evidence.poi_lon_deg,
        evidence.poi_abs_alt_m - home_abs_alt_m,
        evidence.poi_abs_alt_m,
    )


def validate_pre_snap_evidence(
    expectation: PoiExpectation,
    evidence: SelectionEvidence,
    selected_location: PoiLocation | None,
    *,
    configured_rel_alt_m: float,
    coordinate_tolerance_m: float,
    altitude_tolerance_m: float,
) -> EvidenceGateResult:
    """Validate POI identity, coordinate, and altitude before scoring SNAP."""
    errors = _identity_errors(expectation, evidence)
    coordinate_error: float | None = None
    altitude_error: float | None = None
    if selected_location is None:
        errors.append("selected POI coordinate could not be resolved")
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
                f"selected POI coordinate differs by {coordinate_error:.3f} m "
                f"(tolerance {coordinate_tolerance_m:.3f} m)"
            )
        if altitude_error > altitude_tolerance_m:
            errors.append(
                f"selected POI altitude differs by {altitude_error:.3f} m "
                f"(tolerance {altitude_tolerance_m:.3f} m)"
            )
    configured_error = abs(
        configured_rel_alt_m - expectation.location.rel_alt_m
    )
    if configured_error > altitude_tolerance_m:
        errors.append(
            "configured POI relative altitude differs by "
            f"{configured_error:.3f} m"
        )
    return EvidenceGateResult(
        passed=not errors,
        errors=tuple(errors),
        coordinate_error_m=coordinate_error,
        altitude_error_m=altitude_error,
    )


def _identity_errors(
    expectation: PoiExpectation,
    evidence: SelectionEvidence,
) -> list[str]:
    errors: list[str] = []
    if evidence.catalog_wp != expectation.poi_wp:
        errors.append(
            f"POI catalog WP mismatch: expected {expectation.poi_wp}, "
            f"got {evidence.catalog_wp}"
        )
    if evidence.catalog_seq != expectation.mission_seq:
        errors.append(
            "POI catalog mission-seq mismatch: expected "
            f"{expectation.mission_seq}, got {evidence.catalog_seq}"
        )
    if evidence.task_id is None or evidence.obj_id is None:
        errors.append("no selected POI identity was logged")
    elif evidence.task_id != expectation.expected_task_id:
        errors.append(
            f"selected task mismatch: expected P{expectation.expected_task_id}, "
            f"got P{evidence.task_id}"
        )
    if evidence.obj_id is not None and evidence.obj_id != expectation.expected_obj_id:
        label = "default OOI was selected" if evidence.fallback_location_registered else "selected object mismatch"
        errors.append(f"{label} (obj_id={evidence.obj_id}); expected mission POI obj_id=0")
    return errors
