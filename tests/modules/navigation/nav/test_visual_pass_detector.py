import math

import pytest

from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.core.approach.vision_nav.visual_pass import (
    PASS_ARM_MAX_OFF_NOSE_DEG,
    VisualPassDetector,
)


def _frame(ts, body_x):
    return FinalApproachVisionFrame(
        "cam", 0, 1, 2, ts,
        body_x, 0.0, 0.0 if body_x else 1.0,
        body_x, 0.0, 0.0 if body_x else 1.0,
    )


def test_initial_aft_is_commanded_without_arming_or_missing():
    detector = VisualPassDetector()
    plan = detector.plan(_frame(1.0, -1.0))
    detector.commit(plan)

    assert not plan.suppress_command
    assert not detector.missed


def test_forward_then_two_aft_frames_suppress_and_latch_a_miss():
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
    assert detector.missed


def test_intervening_forward_clears_pending_aft_evidence():
    detector = VisualPassDetector()
    for frame in (_frame(1.0, 1.0), _frame(2.0, -1.0), _frame(3.0, 1.0)):
        detector.commit(detector.plan(frame))
    next_aft = detector.plan(_frame(4.0, -1.0))
    detector.commit(next_aft)

    assert next_aft.suppress_command
    assert not detector.missed


# --- a6: arm only when the dock is clearly ahead ---------------------------


def _los(ts: float, off_nose_deg: float) -> FinalApproachVisionFrame:
    """Unit LOS `off_nose_deg` from the nose, in the body x-y plane."""
    angle = math.radians(off_nose_deg)
    x, y = math.cos(angle), math.sin(angle)
    return FinalApproachVisionFrame(
        "cam", 0, 1, 2, ts,
        x, y, 0.0,
        x, y, 0.0,
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
def test_abeam_start_wobbling_across_ninety_degrees_never_misses(wobble_deg):
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
    assert not detector.missed


def test_clearly_ahead_then_two_aft_frames_is_a_miss():
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, PASS_ARM_MAX_OFF_NOSE_DEG - 1.0),
        _los(2.0, 150.0),
        _los(3.0, 150.0),
    )
    assert suppressed == [False, True, True]
    assert detector.missed


def test_ahead_but_outside_the_arm_cone_does_not_arm():
    detector = VisualPassDetector()
    suppressed = _run(
        detector,
        _los(1.0, PASS_ARM_MAX_OFF_NOSE_DEG + 1.0),
        _los(2.0, 150.0),
        _los(3.0, 150.0),
    )
    assert not any(suppressed)
    assert not detector.missed


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
    assert detector.missed


# --- physical docking: an aft dock is a miss, never a pass ----------------


def test_a_missed_dock_keeps_suppressing_until_reset():
    """ideal_360 keeps an aft dock visible: commands must stay off."""
    detector = VisualPassDetector()
    _run(detector, _los(1.0, 5.0), _los(2.0, 170.0), _los(3.0, 170.0))
    assert detector.missed

    assert _run(detector, _los(4.0, 170.0), _los(5.0, 5.0)) == [True, True]
    detector.reset()
    assert not detector.missed
    assert _run(detector, _los(6.0, 5.0)) == [False]


def test_the_gate_has_no_success_state():
    """The picture never declares success (physical_docking.md rule 1)."""
    detector = VisualPassDetector()
    assert not hasattr(detector, "passed")
    assert not hasattr(detector, "latch_detection_loss")
