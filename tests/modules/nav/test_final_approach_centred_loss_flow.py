"""Flow: a dock lost centred after arming runs the pass path, not NAV_FAIL (a7)."""

from __future__ import annotations

import math
from unittest.mock import Mock

from navpy.core.approach.vision_nav.visual_pass import VisualPassDetector
from navpy.core.flow.nav_state import NavState
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.ports.frame import FinalApproachVisionFrame
from tests.modules.nav.test_nav_controller import (
    _create_controller,
    _create_detected_poi,
    _create_mock_detector,
    _create_mock_navigation,
    _create_mock_vehicle,
)

ZR10_1X_SHORT_HALF_FOV_DEG = math.degrees(math.atan(360.0 / 1407.42))


def _ahead_frame(off_axis_deg: float) -> FinalApproachVisionFrame:
    return FinalApproachVisionFrame(
        "gimbal_0", 0, 1, 1, 1.0,
        1.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        camera_off_axis_deg=off_axis_deg,
        camera_half_fov_deg=ZR10_1X_SHORT_HALF_FOV_DEG,
    )


def _run_loss_tick(detector: VisualPassDetector):
    """One NAV tick with no detection, then one decision step."""
    navigation = _create_mock_navigation()
    navigation.final_approach.is_active = True
    navigation.final_approach.consume_command_liveness_failure = Mock(
        return_value=False
    )
    navigation.final_approach.poi_passed_override = lambda: detector.passed
    # The camera optics are unchanged since the last frame (the reader itself
    # is covered in tests/modules/nav/test_final_approach_optics.py).
    navigation.final_approach.latch_pass_on_detection_loss = (
        lambda _reader: detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG)
    )
    logger = Mock()
    controller = _create_controller(
        vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
        detector=_create_mock_detector(detections=[]),
        navigation=navigation,
        logger=logger,
    )
    controller.confirmation_manager.set_active_poi(_create_detected_poi(obj_id=1))
    controller.sensor.sense()
    controller.phase.current = NavState.NAV

    controller.final_approach_nav.act_nav()
    controller.decision.decide()

    messages = [str(call.args[0]) for call in logger.info.call_args_list if call.args]
    return controller, messages


def test_centred_loss_after_arming_runs_the_pass_path_not_nav_fail() -> None:
    detector = VisualPassDetector()
    detector.commit(detector.plan(_ahead_frame(2.3)))

    controller, messages = _run_loss_tick(detector)

    assert detector.passed
    assert not any("NAV_FAIL" in message for message in messages)
    assert any("PASS: dock lost centred" in message for message in messages)
    assert controller.navigation_task.final_approach_nav_completed is True
    assert controller.phase.is_current(NavState.RESET)
    assert any("RESET: PASSED POI" in message for message in messages)


def test_edge_loss_after_arming_stays_no_detection_failure() -> None:
    detector = VisualPassDetector()
    detector.commit(detector.plan(_ahead_frame(13.0)))

    controller, messages = _run_loss_tick(detector)

    assert not detector.passed
    assert any("NAV_FAIL: no_detection" in message for message in messages)
    assert not controller.navigation_task.final_approach_nav_completed
    assert not any("RESET: PASSED POI" in message for message in messages)
