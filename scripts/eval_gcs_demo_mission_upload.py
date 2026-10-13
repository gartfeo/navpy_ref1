"""Upload the scenario's missions and prove the onboard readback."""

from __future__ import annotations

import struct

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_DO_JUMP, MAV_FRAME_GLOBAL

from scripts.eval_gcs_demo_config import TAKEOFF_ALTITUDE_M
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_scenario import load_scenario_manifest
from scripts.eval_gcs_demo_scenario_missions import bind_missions
from scripts.eval_gcs_demo_upload import mission_assignment, upload_mission_assignments

from gcs.backend.mission_validator import raw_mission_item
from gcs.backend.models import VehicleAssignment
from gcs.backend.planner.mission_items import build_mission
from gcs.backend.routes.mission_upload_helpers import assignment_geometry


#: MAVLink carries these as float32; x and y are exact integers.
_FLOAT32_FIELDS = ("param1", "param2", "param3", "param4", "z")
#: ArduPilot stores no frame for a command without a location and downloads it
#: as MAV_FRAME_GLOBAL (observed live: the skip jump goes up as MAV_FRAME_MISSION).
_READBACK_FRAME = {MAV_CMD_DO_JUMP: MAV_FRAME_GLOBAL}


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def expected_items(assignment: dict) -> list[dict]:
    """Every stored row the upload route builds for *assignment*, after seq 0.

    The parsed readback skips takeoff, the skip jump and the metadata rows, so
    only this raw comparison shows they are what the scenario intends.
    """
    model = VehicleAssignment(**assignment)
    track, polygon, backbone, launch_point, hub = assignment_geometry(model)
    loader = build_mission(
        track, model.altitude_m, model.sys_id, model.corridor_count,
        model.search_pattern, polygon, backbone, launch_point,
        model.corridor_altitude_m, hub, TAKEOFF_ALTITUDE_M,
    )
    rows = []
    for seq in range(1, loader.count()):
        row = raw_mission_item(seq, loader.wp(seq))
        row = {**row, **{name: _float32(row[name]) for name in _FLOAT32_FIELDS}}
        row["frame"] = _READBACK_FRAME.get(row["command"], row["frame"])
        rows.append(row)
    return rows


def _require_raw_items(sys_id: int, onboard: object, assignment: dict) -> None:
    actual = onboard.get("items") if isinstance(onboard, dict) else None
    expected = expected_items(assignment)
    if actual == expected:
        return
    if not isinstance(actual, list):
        raise RegressionError(f"vehicle {sys_id} mission readback has no raw items")
    differences = [
        f"seq {e['seq']}: " + ", ".join(
            f"{name} onboard {a.get(name)!r} expected {e[name]!r}"
            for name in e if a.get(name) != e[name]
        )
        for a, e in zip(actual, expected)
        if a != e
    ]
    raise RegressionError(
        f"vehicle {sys_id} onboard mission row differs from the upload "
        f"({len(actual)} rows, {len(expected)} expected): " + "; ".join(differences)
    )


def _track_altitudes(onboard: object) -> list[object]:
    rows = onboard.get("waypoints") if isinstance(onboard, dict) else None
    if not isinstance(rows, list) or not all(isinstance(row, dict) for row in rows):
        raise RegressionError("onboard mission waypoints must be a list of objects")
    return [row.get("alt") for row in rows]


def upload_scenario_missions(context: StackContext) -> None:
    """Make the scenario's missions the onboard missions, then read them back.

    Runs before the companion restart, so the companions start on them. The
    upload route only warns on field mismatches, so this check is the gate.
    """
    expected = bind_missions(load_scenario_manifest().missions, context.sys_ids)
    upload_mission_assignments(context, expected)
    for assignment in expected:
        sys_id = assignment["sys_id"]
        onboard = context.api.get_json(f"/api/vehicles/{sys_id}/mission", timeout=90.0)
        readback = mission_assignment(
            onboard,
            expected_sys_id=sys_id,
            zone_index=assignment["zone_index"],
        )
        # mission_assignment keeps one mission altitude; every track row carries
        # its own, and the scenario flies each at the zone altitude.
        altitudes = _track_altitudes(onboard)
        if readback != assignment or any(alt != assignment["altitude_m"] for alt in altitudes):
            raise RegressionError(
                f"vehicle {sys_id} onboard mission differs from the scenario after "
                f"upload: expected {assignment}, onboard {readback}, "
                f"track altitudes {altitudes}"
            )
        _require_raw_items(sys_id, onboard, assignment)


__all__ = ["expected_items", "upload_scenario_missions"]
