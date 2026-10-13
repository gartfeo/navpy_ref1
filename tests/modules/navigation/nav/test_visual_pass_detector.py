import math

import pytest

from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.core.approach.vision_nav.visual_pass import (
    PASS_ARM_MAX_OFF_NOSE_DEG,
    PASS_CENTRED_MAX_DEG,
    PASS_CENTRED_MAX_HALF_FOV_FRACTION,
    VisualPassDetector,
)

# ZR10 at 1x (vision_profiles.json fx 1407.42 px, 1280x720): short half-FOV.
ZR10_1X_SHORT_HALF_FOV_DEG = math.degrees(math.atan(360.0 / 1407.42))


def _frame(ts, body_x):
    return FinalApproachVisionFrame(
        "cam", 0, 1, 2, ts,
        body_x, 0.0, 0.0 if body_x else 1.0,
        body_x, 0.0, 0.0 if body_x else 1.0,
    )


def test_initial_aft_is_commanded_without_arming_or_passing():
    detector = VisualPassDetector()
    plan = detector.plan(_frame(1.0, -1.0))
    detector.commit(plan)

    assert not plan.suppress_command
    assert not detector.passed


def test_forward_then_two_aft_frames_suppress_and_latch_pass():
    detector = VisualPassDetector()
    forward = detector.plan(_frame(1.0, 1.0))
    detector.commit(forward)
    first_aft = detector.plan(_frame(2.0, -1.0))
    detector.commit(first_aft)
    second_aft = detector.plan(_frame(3.0, -1.0))
    detector.commit(second_aft)

    assert not forward.suppress_command
    assert first_aft.suppress_command
    assert second_aft.suppress_command
    assert detector.passed


def test_intervening_forward_clears_pending_aft_evidence():
    detector = VisualPassDetector()
    for frame in (_frame(1.0, 1.0), _frame(2.0, -1.0), _frame(3.0, 1.0)):
        detector.commit(detector.plan(frame))
    next_aft = detector.plan(_frame(4.0, -1.0))
    detector.commit(next_aft)

    assert next_aft.suppress_command
    assert not detector.passed


# --- a6: arm only when the dock is clearly ahead ---------------------------


def _los(
    ts: float,
    off_nose_deg: float,
    off_axis_deg: float = 180.0,
    half_fov_deg: float = 0.0,
) -> FinalApproachVisionFrame:
    """Unit LOS `off_nose_deg` from the nose, in the body x-y plane."""
    angle = math.radians(off_nose_deg)
    x, y = math.cos(angle), math.sin(angle)
    return FinalApproachVisionFrame(
        "cam", 0, 1, 2, ts,
        x, y, 0.0,
        x, y, 0.0,
        camera_off_axis_deg=off_axis_deg,
        camera_half_fov_deg=half_fov_deg,
    )


def _run(detector: VisualPassDetector, *frames: FinalApproachVisionFrame) -> list[bool]:
    suppressed = []
    for frame in frames:
        plan = detector.plan(frame)
        detector.commit(plan)
        suppressed.append(plan.suppress_command)
    return suppressed


def test_arm_threshold_is_sixty_degrees_off_the_nose():
    assert PASS_ARM_MAX_OFF_NOSE_DEG == 60.0


@pytest.mark.parametrize("wobble_deg", [85.0, 88.0, 89.9])
def test_abeam_start_wobbling_across_ninety_degrees_never_passes(wobble_deg):
    """a1-measure-b uav_1: dock ~90 deg abeam at NAV entry, then a small turn."""
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, wobble_deg),
        _los(2.0, 180.0 - wobble_deg),
        _los(3.0, 180.0 - wobble_deg),
        _los(4.0, wobble_deg),
        _los(5.0, 180.0 - wobble_deg),
        _los(6.0, 180.0 - wobble_deg),
    )
    assert not any(suppressed)
    assert not detector.passed


def test_clearly_ahead_then_two_aft_frames_passes():
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, PASS_ARM_MAX_OFF_NOSE_DEG - 1.0),
        _los(2.0, 150.0),
        _los(3.0, 150.0),
    )
    assert suppressed == [False, True, True]
    assert detector.passed


def test_ahead_but_outside_the_arm_cone_does_not_arm():
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, PASS_ARM_MAX_OFF_NOSE_DEG + 1.0),
        _los(2.0, 150.0),
        _los(3.0, 150.0),
    )
    assert not any(suppressed)
    assert not detector.passed


def test_arming_persists_through_a_forward_frame_outside_the_cone():
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, 10.0),
        _los(2.0, 75.0),
        _los(3.0, 150.0),
        _los(4.0, 150.0),
    )
    assert suppressed == [False, False, True, True]
    assert detector.passed


# --- a7: an armed dock lost while centred counts as passed -----------------


def test_centred_constants():
    assert PASS_CENTRED_MAX_DEG == 3.0
    assert PASS_CENTRED_MAX_HALF_FOV_FRACTION == 0.5


@pytest.mark.parametrize("off_axis_deg", [0.0, 2.0, 2.3, PASS_CENTRED_MAX_DEG])
def test_armed_dock_lost_centred_latches_pass(off_axis_deg):
    """a1-measure-b uav_2/uav_3: last frames 2.0/2.3 deg off axis, behind_cam."""
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, off_axis_deg, ZR10_1X_SHORT_HALF_FOV_DEG))

    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is True
    assert detector.passed
    after = detector.plan(_los(2.0, 5.0, 0.0, ZR10_1X_SHORT_HALF_FOV_DEG))
    assert after.suppress_command


def test_armed_dock_lost_at_frame_edge_stays_a_failure():
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, 13.0, ZR10_1X_SHORT_HALF_FOV_DEG))

    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is False
    assert not detector.passed


def test_edge_loss_at_high_zoom_is_not_centred_inside_three_degrees():
    """At ZR10 10x the short half-FOV is ~2 deg: the edge is inside 3 deg."""
    half_fov_10x_deg = math.degrees(math.atan(360.0 / 10332.425))
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, 0.95 * half_fov_10x_deg, half_fov_10x_deg))

    assert detector.latch_detection_loss(half_fov_10x_deg) is False
    assert not detector.passed


def test_only_the_last_frame_decides_centred():
    detector = VisualPassDetector()
    _run(
        detector,
        _los(1.0, 5.0, 1.0, ZR10_1X_SHORT_HALF_FOV_DEG),
        _los(2.0, 5.0, 10.0, ZR10_1X_SHORT_HALF_FOV_DEG),
    )
    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is False


def test_unarmed_dock_lost_centred_stays_a_failure():
    """A gimbal centred on an abeam dock is not a pass."""
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 88.0, 0.5, ZR10_1X_SHORT_HALF_FOV_DEG))

    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is False
    assert not detector.passed


def test_unknown_camera_state_never_counts_as_centred():
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0))

    assert detector.latch_detection_loss(0.0) is False


def test_reset_clears_a_loss_latched_pass():
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, 1.0, ZR10_1X_SHORT_HALF_FOV_DEG))
    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is True
    detector.reset()
    assert not detector.passed
    assert detector.latch_detection_loss(ZR10_1X_SHORT_HALF_FOV_DEG) is False


def test_optics_change_after_the_last_frame_blocks_the_loss_pass():
    """A zoom-in after the last frame narrows the FOV: an edge exit could
    look centred under the old optics, so the loss stays a failure."""
    half_fov_10x_deg = math.degrees(math.atan(360.0 / 10332.425))
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, 1.5, ZR10_1X_SHORT_HALF_FOV_DEG))

    assert detector.latch_detection_loss(half_fov_10x_deg) is False
    assert detector.latch_detection_loss(None) is False
    assert not detector.passed


def test_a_frame_with_the_new_optics_restores_eligibility():
    half_fov_2x_deg = math.degrees(math.atan(360.0 / 2066.485))
    detector = VisualPassDetector()
    _run(
        detector,
        _los(1.0, 5.0, 1.5, ZR10_1X_SHORT_HALF_FOV_DEG),
        _los(2.0, 5.0, 1.0, half_fov_2x_deg),
    )

    assert detector.latch_detection_loss(half_fov_2x_deg) is True


def test_zero_half_fov_never_counts_as_centred_even_on_axis():
    """A half-FOV of 0 means unknown optics, not a centred dock."""
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0, 0.0, 0.0))

    assert detector.latch_detection_loss(0.0) is False
    assert not detector.passed
