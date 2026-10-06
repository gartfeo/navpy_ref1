"""Mechanism-level tests for the vertical (pitch) command formula.

These exercise `pitch_law.py`'s pure function directly, independent of the
frame/anchor plumbing in `law.py`, so the integrator's defining property is
isolated from everything else in the terminal law.

An `AAS_PITCH_LAW` selector with three alternative formulations was removed on
2026-08-20 after all three were measured and rejected; the tests that covered
mode selection, `PitchHistory`, `telescope` and `memoryless` went with it. See
`pitch_law.py`'s module docstring for why each failed.
"""

import math

import pytest

from navpy.modules.navigation.nav.vision_nav.law import VERTICAL_PN_NAVIGATION_CONSTANT
from navpy.modules.navigation.nav.vision_nav.pitch_law import raw_pitch_current


def test_raw_pitch_current_matches_the_integrator_formula():
    anchor_pitch = -12.0
    rate = math.radians(3.0)
    dt_s = 0.2
    expected = anchor_pitch - math.degrees(
        VERTICAL_PN_NAVIGATION_CONSTANT * rate * dt_s
    )
    assert raw_pitch_current(
        anchor_pitch, VERTICAL_PN_NAVIGATION_CONSTANT, rate, dt_s
    ) == pytest.approx(expected)


# --------------------------------------------------------------------------
# The accumulation property, pinned deliberately rather than guarded against.
#
# A bias living in the RATE estimate -- not in a real target-elevation change
# -- is exactly what a running integral cannot forget, and the drift scales
# with active navigation time. That is the acknowledged price of the disturbance
# rejection this formula exists to provide: rejecting a constant unknown bias
# REQUIRES integral action, which is why the memoryless P+D alternative lost
# every tailwind block of the SITL wind matrix. The mitigation is filtering the
# rate channel (`VERTICAL_RATE_FILTER_TAU_S`), not removing the integrator.
# --------------------------------------------------------------------------


def test_current_accumulates_unbounded_drift_from_a_pure_rate_bias():
    dt_s = 0.2
    rate_bias_rad_s = math.radians(0.05)  # small, persistent, non-elevation
    cycles = 400  # 80 s of active navigation -- beyond the reported 21-55 s SITL range
    pitch = 0.0
    series = []
    for _ in range(cycles):
        pitch = raw_pitch_current(
            pitch, VERTICAL_PN_NAVIGATION_CONSTANT, rate_bias_rad_s, dt_s
        )
        series.append(pitch)

    per_cycle = -math.degrees(
        VERTICAL_PN_NAVIGATION_CONSTANT * rate_bias_rad_s * dt_s
    )
    assert series[-1] == pytest.approx(per_cycle * cycles, rel=1e-9)
    # Monotone and growing with elapsed time -- a longer navigation task strictly
    # means a bigger drift, exactly the "scales with active navigation time" property.
    assert abs(series[-1]) > abs(series[cycles // 2]) > abs(series[cycles // 4]) > 0.0
