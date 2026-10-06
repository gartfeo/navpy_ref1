import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.legacy_final_approach_command import (
    LegacyFinalApproachCommand,
    LegacyFinalApproachCommandPorts,
)
from navpy.modules.navigation.nav.nav_law import NavCommand, NavCommandMode
from tests.detection_factory import make_detected_poi


def _command_fixture(*, adjusted: bool = False):
    current = Location(40.0, 44.0, 1000.0, is_absolute=True)
    locked = Location(40.001, 44.002, 900.0, is_absolute=True)
    mission = Location(41.0, 45.0, 700.0, is_absolute=True)
    vehicle = Mock()
    vehicle.location.return_value = current
    vehicle.attitude = Attitude(-5.0, 20.0, 3.0)
    vehicle.get_mission_item_location.return_value = mission
    geo_ref = Mock()
    geo_ref.calc_yaw_pitch_proj.return_value = (4.0, -2.0)
    geo_ref.calc_pitch_los.return_value = -6.0
    nav = Mock()
    nav.calc.return_value = NavCommand(
        NavCommandMode.ACTUATOR,
        cmd_roll_deg=8.0,
        cmd_pitch_deg=-10.0,
        cmd_thr=0.55,
    )
    parts = SimpleNamespace(
        current=current,
        locked=locked,
        mission=mission,
        vehicle=vehicle,
        geo_ref=geo_ref,
        nav=nav,
        logger=Mock(),
        navigation_logger=Mock(),
        get_locked_poi=Mock(return_value=locked),
        adjust_nav=Mock(return_value=False),
        is_adjusted=Mock(return_value=adjusted),
        calc_nav_poi=Mock(return_value=locked),
    )
    command = LegacyFinalApproachCommand(LegacyFinalApproachCommandPorts(
        vehicle=parts.vehicle,
        geo_ref=parts.geo_ref,
        logger=parts.logger,
        navigation_logger=parts.navigation_logger,
        nav=parts.nav,
        get_locked_poi=parts.get_locked_poi,
        adjust_nav=parts.adjust_nav,
        is_adjusted=parts.is_adjusted,
        calc_nav_poi=parts.calc_nav_poi,
    ))
    return command, parts


def test_adjusted_command_preserves_context_and_command_values():
    command, parts = _command_fixture(adjusted=True)
    poi_ned = np.array([1.0, 0.0, 0.1])

    result = command.execute(poi_ned, None)

    context = parts.nav.calc.call_args.args[0]
    assert context.prev_loc == Location(
        parts.mission.lat,
        parts.mission.lng,
        parts.current.alt,
        is_absolute=True,
    )
    assert context.current_loc is parts.current
    assert context.next_loc is parts.locked
    assert context.target_bearing_cd == 1600.0
    assert context.poi_ned is poi_ned
    assert context.pitch_error == -6.0
    assert context.distance == 100.0
    parts.vehicle.set_attitude.assert_called_once_with(
        math.radians(8.0),
        math.radians(-10.0),
        thr=0.55,
    )
    assert parts.get_locked_poi.call_count == 2
    assert (
        result.yaw,
        result.pitch,
        result.cmd_roll,
        result.cmd_pitch,
        result.cmd_thr,
    ) == (4.0, -6.0, 8.0, -10.0, 0.55)


def test_successful_adjustment_stops_before_nav_actuation_and_logging():
    command, parts = _command_fixture()
    parts.adjust_nav.return_value = True

    result = command.execute(np.array([1.0, 0.0, 0.1]), None)

    assert result is None
    parts.geo_ref.calc_yaw_pitch_proj.assert_called_once()
    parts.geo_ref.calc_pitch_los.assert_called_once()
    parts.is_adjusted.assert_not_called()
    parts.calc_nav_poi.assert_not_called()
    parts.nav.calc.assert_not_called()
    parts.vehicle.set_attitude.assert_not_called()
    parts.navigation_logger.log.assert_not_called()


def test_non_actuator_nav_command_fails_before_actuation_and_logging():
    command, parts = _command_fixture()
    parts.nav.calc.return_value = NavCommand(
        NavCommandMode.ACCELERATION,
        a_cmd_ned=np.array([1.0, 2.0, 3.0]),
    )

    with pytest.raises(
            AssertionError,
            match="Legacy final approach expects ACTUATOR NavCommand",
    ):
        command.execute(np.array([1.0, 0.0, 0.1]), None)

    parts.vehicle.set_attitude.assert_not_called()
    parts.navigation_logger.log.assert_not_called()


def test_actuation_exception_propagates_without_diagnostic_log():
    command, parts = _command_fixture()
    parts.vehicle.set_attitude.side_effect = RuntimeError("actuation failed")

    with pytest.raises(RuntimeError, match="actuation failed"):
        command.execute(np.array([1.0, 0.0, 0.1]), None)

    parts.navigation_logger.log.assert_not_called()


def test_diagnostic_exception_propagates_after_actuation():
    command, parts = _command_fixture()
    parts.navigation_logger.log.side_effect = RuntimeError("logging failed")

    with pytest.raises(RuntimeError, match="logging failed"):
        command.execute(np.array([1.0, 0.0, 0.1]), None)

    parts.vehicle.set_attitude.assert_called_once()


def test_sim_truth_is_used_only_by_post_command_diagnostics():
    command, parts = _command_fixture()
    truth = Location(42.0, 46.0, 500.0, is_absolute=True)
    detection = make_detected_poi(
        t_g_loc_debug=truth,
        is_simulation=True,
    )
    poi_ned = np.array([1.0, 0.0, 0.1])

    command.execute(poi_ned, detection)

    parts.calc_nav_poi.assert_called_once_with(
        parts.current,
        poi_ned,
        100.0,
        parts.locked,
    )
    assert parts.nav.calc.call_args.args[0].poi_ned is poi_ned
    assert parts.get_locked_poi.call_count == 1
    assert (
        parts.navigation_logger.log.call_args.kwargs["detect_t_loc_debug"]
        is truth
    )


@pytest.mark.parametrize(
    ("current_location", "poi_ned", "warning"),
    [
        (None, np.array([1.0, 0.0, 0.1]), "FAILED TO GET CURRENT LOCATION."),
        ("current", None, "POI NED IS NONE."),
    ],
)
def test_invalid_ingress_returns_before_calculation(
        current_location,
        poi_ned,
        warning,
):
    command, parts = _command_fixture()
    if current_location is None:
        parts.vehicle.location.return_value = None

    result = command.execute(poi_ned, None)

    assert result is None
    parts.logger.warning.assert_called_once_with(warning)
    parts.get_locked_poi.assert_called_once_with()
    parts.geo_ref.calc_yaw_pitch_proj.assert_not_called()
    parts.vehicle.set_attitude.assert_not_called()
    parts.navigation_logger.log.assert_not_called()
