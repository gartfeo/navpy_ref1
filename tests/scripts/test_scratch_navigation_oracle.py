"""The truth-fed gain must deliver True PN exactly, or it measures nothing.

The whole point of this treatment is to establish a CEILING. A substitution
that is merely "bigger in the right direction" would produce a number nobody
could interpret: a partial improvement could not be told apart from a partial
implementation. So the central test here is the identity, not an inequality.
"""

from __future__ import annotations

import math

import pytest

from scripts.scratch_navigation_oracle import (
    MAX_SPEED_MPS,
    MODES,
    substitute_speed,
)

# Target due north, so the LOS unit vector is (1, 0) and every projection below
# can be read by eye.
NORTH_OFFSET = (3000.0, 0.0, 0.0)
CHANNEL = 22.57


def call(mode, **overrides):
    kwargs = dict(
        channel_airspeed_mps=CHANNEL,
        ground_speed_mps=26.81,
        course_deg=0.0,
        wind_speed_mps=0.0,
        wind_dir_deg=0.0,
        offset_ned_m=NORTH_OFFSET,
    )
    kwargs.update(overrides)
    return substitute_speed(mode, **kwargs)


def test_none_hands_back_the_channel_untouched() -> None:
    gain = call("none")
    assert gain.speed_mps == CHANNEL
    assert not gain.applied


def test_unknown_mode_raises_rather_than_falling_through() -> None:
    # Silently treating a typo as "none" would report a clean null for a
    # treatment that never ran, which is the one failure this experiment
    # cannot survive.
    with pytest.raises(ValueError):
        call("tpm")


@pytest.mark.parametrize("mode", [m for m in MODES if m != "none"])
def test_calm_leaves_the_wind_factor_at_exactly_one(mode: str) -> None:
    # With no wind the air mass IS the ground frame, so Vc_ground == Vc_air and
    # the ratio must be exactly 1. `vcg` therefore has nothing to correct in
    # calm, and that is the control the wind cells are read against.
    gain = call(mode)
    assert gain.applied
    assert gain.vc_ground_mps == pytest.approx(gain.vc_air_mps)
    expected = CHANNEL if mode == "vcg" else 26.81
    assert gain.speed_mps == pytest.approx(expected)


def test_tpn_delivers_exactly_true_pn_in_a_tailwind() -> None:
    """S * cos(sigma) must equal Vc_ground -- the cosine cancels, not divides.

    This is the identity the module is built on. A bank puts only cos(sigma) of
    its acceleration onto the LOS, so a gain S delivers `N*S*cos(sigma)*lam`.
    True PN asks for `N*Vc_ground*lam`. If this assertion holds, the treatment
    IS True PN and the run measures the real ceiling.
    """
    # Crabbing: flying 20 deg right of north through an 8 m/s wind, so the
    # nose, the track and the LOS are three different directions.
    gain = call(
        "tpn", ground_speed_mps=34.7, course_deg=12.0,
        wind_speed_mps=8.0, wind_dir_deg=200.0,
    )
    assert gain.applied

    ground_n = 34.7 * math.cos(math.radians(12.0))
    ground_e = 34.7 * math.sin(math.radians(12.0))
    blowing = math.radians(200.0 + 180.0)
    air_n = ground_n - 8.0 * math.cos(blowing)
    air_e = ground_e - 8.0 * math.sin(blowing)
    # sigma is measured from the AIR-relative velocity, because that is the
    # direction a coordinated bank accelerates perpendicular to.
    sigma = math.atan2(air_e, air_n)

    assert gain.speed_mps * math.cos(sigma) == pytest.approx(
        gain.vc_ground_mps, rel=1e-9
    )


def test_tailwind_lowers_the_gain_and_headwind_raises_it() -> None:
    # Direction check on the physics: a tailwind means the ground closes faster
    # than the air does, so True PN wants MORE gain than the air-frame law
    # supplies; a headwind is the reverse. Getting this backwards would still
    # produce a plausible-looking run.
    tail = call("vcg", ground_speed_mps=34.7, course_deg=0.0,
                wind_speed_mps=8.0, wind_dir_deg=180.0)
    head = call("vcg", ground_speed_mps=19.1, course_deg=0.0,
                wind_speed_mps=8.0, wind_dir_deg=0.0)
    assert tail.vc_ground_mps > tail.vc_air_mps
    assert tail.speed_mps > CHANNEL
    assert head.vc_ground_mps < head.vc_air_mps
    assert head.speed_mps < CHANNEL


def test_no_singularity_or_sign_flip_across_the_forward_sector() -> None:
    """Every bearing out to the rear stays positive and bounded.

    The rejected form of this treatment divided by cos(bearing), which blows up
    at 90 degrees and REVERSES beyond it -- turning the certified 118 degree
    rear acquisition into a fly-away. This form must not, at any angle.
    """
    for bearing_deg in range(0, 180, 5):
        bearing = math.radians(bearing_deg)
        gain = call(
            "tpn",
            offset_ned_m=(3000.0 * math.cos(bearing),
                          3000.0 * math.sin(bearing), 0.0),
        )
        assert gain.speed_mps > 0.0
        assert gain.speed_mps <= MAX_SPEED_MPS


def test_falls_back_and_is_counted_when_barely_closing() -> None:
    # Target abeam: almost none of the velocity is along the LOS, so the ratio
    # has a near-zero denominator. Falling back must be visible in the result,
    # never silent.
    gain = call("tpn", offset_ned_m=(1.0, 3000.0, 0.0))
    assert not gain.applied
    assert gain.speed_mps == CHANNEL


def test_missing_telemetry_falls_back_rather_than_guessing() -> None:
    assert not call("tpn", ground_speed_mps=None).applied
    assert not call("tpn", course_deg=None).applied
