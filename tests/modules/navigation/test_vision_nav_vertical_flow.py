import math

import pytest

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.rate_filter import VerticalRateFilter


def _frame(ts, angle_deg, generation=0):
    angle = math.radians(angle_deg)
    ray = (math.cos(angle), 0.0, math.sin(angle))
    return TerminalVisionFrame("cam", generation, 1, 2, ts, *ray, *ray)


def test_filter_uses_raw_source_delta_and_tau():
    estimator = VerticalRateFilter()
    estimator.seed(_frame(1.0, 10.0))
    plan = estimator.plan(_frame(1.2, 12.0), 0.5)
    expected = (1.0 - math.exp(-0.2 / 0.5)) * math.radians(2.0) / 0.2
    assert plan.next_state.filtered_rate_rad_s == pytest.approx(expected)
    # The pitch integrator consumes the mean, while history retains the same
    # endpoint response checked above.
    expected_mean = (1.0 + 0.5 / 0.2 * math.expm1(-0.2 / 0.5)) * math.radians(2.0) / 0.2
    assert plan.rate_rad_s == pytest.approx(expected_mean)


def test_generation_change_resets_rate_without_touching_timestamp():
    estimator = VerticalRateFilter()
    estimator.seed(_frame(100.0, 10.0, 0))
    assert estimator.plan(_frame(1.0, 40.0, 1), 0.5).rate_rad_s == 0.0
