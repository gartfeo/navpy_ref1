from __future__ import annotations

from pathlib import Path
from unittest.mock import Mock

from scripts import eval_siyi_geo_tracking_three_uav as evaluator
from scripts import siyi_geo_track_child as child
from navpy.modules.common.models.location import Location


def test_wait_ready_uses_child_marker_not_profile_specific_console_text(
    tmp_path: Path,
) -> None:
    marker = tmp_path / "ready.marker"
    marker.write_text("SIYI_GEO_READY\n", encoding="utf-8")
    process = Mock()
    process.poll.return_value = None

    evaluator._wait_ready(marker, process, 0.1)


def test_wait_ready_reports_child_exit(tmp_path: Path) -> None:
    process = Mock()
    process.poll.return_value = 7
    process.returncode = 7

    try:
        evaluator._wait_ready(tmp_path / "ready.marker", process, 0.1)
    except RuntimeError as error:
        assert "code=7" in str(error)
    else:
        raise AssertionError("child exit was not reported")


def test_child_commands_target_centered_loiter_at_current_altitude() -> None:
    vehicle = Mock()
    vehicle.location.return_value = Location(1.0, 2.0, 140.0, False)
    vehicle.get_param_or_default.return_value = 90.0
    target = Location(3.0, 4.0, 60.0, True)

    child._command_target_loiter(vehicle, target)

    vehicle.get_param_or_default.assert_called_once_with("WP_LOITER_RAD", 90.0)
    commanded, radius = vehicle.goto_loiter.call_args.args
    assert commanded == Location(3.0, 4.0, 140.0, False)
    assert radius == 90.0


def test_child_rejects_invalid_autopilot_loiter_radius() -> None:
    vehicle = Mock()
    vehicle.location.return_value = Location(1.0, 2.0, 140.0, False)
    vehicle.get_param_or_default.return_value = 0.0

    try:
        child._command_target_loiter(vehicle, Location(3.0, 4.0, 60.0, True))
    except RuntimeError as error:
        assert "WP_LOITER_RAD" in str(error)
    else:
        raise AssertionError("invalid radius was accepted")

    vehicle.goto_loiter.assert_not_called()


def test_child_waits_for_physical_zoom_to_reach_selected_zoom() -> None:
    rig = Mock()
    rig.mount.gimbal.get_zoom_level.side_effect = [3.2, 3.0]

    assert not child._zoom_matches_command(rig, "3")
    assert child._zoom_matches_command(rig, "3")
