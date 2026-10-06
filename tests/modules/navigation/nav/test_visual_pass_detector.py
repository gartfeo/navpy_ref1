from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.visual_pass import VisualPassDetector


def _frame(ts, body_x):
    return TerminalVisionFrame(
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
