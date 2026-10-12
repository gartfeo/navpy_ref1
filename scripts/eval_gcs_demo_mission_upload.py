"""Upload the scenario's missions and prove the onboard readback."""

from __future__ import annotations

from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_scenario import load_scenario_manifest
from scripts.eval_gcs_demo_scenario_missions import bind_missions
from scripts.eval_gcs_demo_upload import mission_assignment, upload_mission_assignments


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


__all__ = ["upload_scenario_missions"]
