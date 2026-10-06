"""The truth sensor must report geometry, and must not leak anything else.

The interesting assertions here are the negative ones: that range cannot reach
the law, and that the control frame carries no yaw. Those are the properties the
whole isolated test rests on, so they are pinned rather than trusted.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from scripts.navigation_truth_sensor import (
    ClosestApproach,
    build_frame,
    poi_offset_ned_m,
)

BASE = {
    "truth_pitch_deg": 0.0,
    "truth_roll_deg": 0.0,
    "truth_yaw_deg": 0.0,
    "airspeed_mps": 30.0,
    "yaw_rate_rad_s": 0.0,
    "source_timestamp_s": 1.0,
}


def test_poi_dead_ahead_is_straight_down_the_nose():
    frame = build_frame(offset_ned_m=np.array([1000.0, 0.0, 0.0]), **BASE)
    assert frame.body_x == pytest.approx(1.0)
    assert frame.body_y == pytest.approx(0.0, abs=1e-12)
    assert frame.body_z == pytest.approx(0.0, abs=1e-12)


def test_range_cannot_reach_the_law():
    """Ten metres and ten kilometres in the same direction are one frame.

    This is the structural guarantee: the law is handed a direction, so it
    cannot recover range, POI position or altitude no matter what it does
    with the numbers.
    """
    near = build_frame(offset_ned_m=np.array([10.0, 3.0, -2.0]), **BASE)
    far = build_frame(offset_ned_m=np.array([10_000.0, 3000.0, -2000.0]), **BASE)
    assert near.body_ray == pytest.approx(far.body_ray)
    assert near.control_ray == pytest.approx(far.control_ray)


def test_yaw_rotates_the_body_ray_because_the_nose_moved():
    """Turning the aircraft must change where the POI appears."""
    ahead = build_frame(offset_ned_m=np.array([1000.0, 0.0, 0.0]), **BASE)
    turned = build_frame(
        offset_ned_m=np.array([1000.0, 0.0, 0.0]), **{**BASE, "truth_yaw_deg": 30.0}
    )
    assert ahead.body_y == pytest.approx(0.0, abs=1e-12)
    # Nose swung right, so the POI now sits to the LEFT of the nose.
    assert turned.body_y == pytest.approx(-math.sin(math.radians(30.0)), abs=1e-9)


def test_control_ray_is_yaw_free():
    """The command frame must not move when only the heading datum moves.

    frame_projection.py:53 builds the control ray with yaw=0.0. Here the whole
    problem is rotated in yaw -- aircraft and POI together -- so the geometry
    relative to the nose is unchanged. A yaw-free control frame must return the
    identical ray; anything else means compass heading reached the command path.
    """
    reference = build_frame(offset_ned_m=np.array([1000.0, 0.0, -50.0]), **BASE)
    for heading in (15.0, 90.0, 200.0, 355.0):
        angle = math.radians(heading)
        rotated_offset = np.array([
            1000.0 * math.cos(angle), -1000.0 * math.sin(angle), -50.0,
        ])
        rotated = build_frame(
            offset_ned_m=rotated_offset,
            **{**BASE, "truth_yaw_deg": -heading},
        )
        assert rotated.control_ray == pytest.approx(reference.control_ray, abs=1e-9)


def test_control_ray_is_stabilised_against_pitch_and_roll():
    """Attitude must not smear the POI's direction.

    Working the rotations through for the ZYX sequence, the control ray reduces
    to Rz(-yaw) applied to the NED direction: the pitch and roll used to leave
    the body frame are undone on the way into the control frame. That is what
    makes it a horizon-stabilised frame, and it is why the aircraft pitching or
    rolling does not by itself move the POI in the command frame.

    Pitch and roll are not lost -- they reach the law as their own scalar
    fields, where the law can use them as the project rule permits.
    """
    level = build_frame(offset_ned_m=np.array([1000.0, 0.0, -80.0]), **BASE)
    for pitch, roll in ((-20.0, 0.0), (0.0, 25.0), (-35.0, -15.0)):
        manoeuvring = build_frame(
            offset_ned_m=np.array([1000.0, 0.0, -80.0]),
            **{**BASE, "truth_pitch_deg": pitch, "truth_roll_deg": roll},
        )
        assert manoeuvring.control_ray == pytest.approx(
            level.control_ray, abs=1e-9
        ), "the POI did not move, so the stabilised frame must not either"
        assert manoeuvring.aircraft_pitch_deg == pytest.approx(pitch)
        assert manoeuvring.aircraft_roll_deg == pytest.approx(roll)
        # The BODY ray does move -- that is the raw camera view.
        assert manoeuvring.body_ray != pytest.approx(level.body_ray)


def test_control_ray_follows_the_poi_across_the_nose():
    """Stabilised is not blind: a POI off to one side must read as off to one side."""
    left = build_frame(offset_ned_m=np.array([1000.0, -400.0, 0.0]), **BASE)
    right = build_frame(offset_ned_m=np.array([1000.0, 400.0, 0.0]), **BASE)
    assert left.control_y < 0.0 < right.control_y


def test_rays_are_unit_length():
    frame = build_frame(offset_ned_m=np.array([700.0, -300.0, 120.0]),
                        **{**BASE, "truth_pitch_deg": -12.0, "truth_roll_deg": 8.0,
                           "truth_yaw_deg": 47.0})
    assert np.linalg.norm(frame.body_ray) == pytest.approx(1.0)
    assert np.linalg.norm(frame.control_ray) == pytest.approx(1.0)


def test_zero_offset_is_refused_not_guessed():
    with pytest.raises(ValueError):
        build_frame(offset_ned_m=np.array([0.0, 0.0, 0.0]), **BASE)


def test_offset_north_east_and_down_have_the_right_signs():
    offset = poi_offset_ned_m(
        lat_deg=40.0, lon_deg=44.0, alt_m=1000.0,
        poi_lat_deg=40.01, poi_lon_deg=44.01, poi_alt_m=900.0,
    )
    assert offset[0] > 0.0, "POI further north"
    assert offset[1] > 0.0, "POI further east"
    assert offset[2] == pytest.approx(100.0), "POI 100 m BELOW, down positive"


def test_offset_uses_latitude_dependent_radii():
    """A degree of longitude shrinks with latitude; a degree of latitude does not.

    Using one spherical radius would bias east against north by a fraction of a
    percent, which would read as a steady lateral navigation error.
    """
    at_equator = poi_offset_ned_m(
        lat_deg=0.0, lon_deg=0.0, alt_m=0.0,
        poi_lat_deg=0.0, poi_lon_deg=0.01, poi_alt_m=0.0,
    )
    at_sixty = poi_offset_ned_m(
        lat_deg=60.0, lon_deg=0.0, alt_m=0.0,
        poi_lat_deg=60.0, poi_lon_deg=0.01, poi_alt_m=0.0,
    )
    assert at_sixty[1] == pytest.approx(at_equator[1] * 0.5, rel=0.01)


def test_closest_approach_finds_the_point_between_samples():
    """The miss is almost never at a sample, so reading samples alone overstates it."""
    approach = ClosestApproach()
    # Flies straight past 5 m to the side, sampled either side of the crossing.
    approach.observe(0.0, np.array([100.0, 5.0, 0.0]))
    approach.observe(1.0, np.array([-100.0, 5.0, 0.0]))
    assert approach.miss_m == pytest.approx(5.0)
    assert approach.at_t_s == pytest.approx(0.5)


def test_closest_approach_is_not_the_nearest_sample():
    approach = ClosestApproach()
    approach.observe(0.0, np.array([30.0, 0.0, 0.0]))
    approach.observe(1.0, np.array([-30.0, 0.0, 0.0]))
    nearest_sample = 30.0
    assert approach.miss_m < nearest_sample
    assert approach.miss_m == pytest.approx(0.0, abs=1e-9)


def test_closest_approach_does_not_invent_a_future_pass():
    """Once the POI is behind, the miss must not improve on later segments."""
    approach = ClosestApproach()
    approach.observe(0.0, np.array([10.0, 0.0, 0.0]))
    approach.observe(1.0, np.array([-10.0, 0.0, 0.0]))
    assert approach.miss_m == pytest.approx(0.0, abs=1e-9)
    approach.observe(2.0, np.array([-500.0, 0.0, 0.0]))
    approach.observe(3.0, np.array([-1000.0, 0.0, 0.0]))
    assert approach.miss_m == pytest.approx(0.0, abs=1e-9)
    assert approach.at_t_s == pytest.approx(0.5)


def test_closest_approach_handles_a_repeated_sample():
    approach = ClosestApproach()
    approach.observe(0.0, np.array([7.0, 0.0, 0.0]))
    approach.observe(0.1, np.array([7.0, 0.0, 0.0]))
    assert approach.miss_m == pytest.approx(7.0)


def test_the_law_is_given_the_estimate_not_the_truth():
    """Truth aims the ray; the estimate is what the aircraft knows about itself.

    A real aircraft has no truth attitude. If the frame reported truth, the law
    would fly on information the flying article does not have, and a pass here
    would not transfer to it.
    """
    frame = build_frame(
        offset_ned_m=np.array([1000.0, 200.0, -50.0]),
        truth_pitch_deg=-10.0, truth_roll_deg=4.0, truth_yaw_deg=20.0,
        est_pitch_deg=-9.1, est_roll_deg=3.4,
        airspeed_mps=30.0, yaw_rate_rad_s=0.12, source_timestamp_s=2.0,
    )
    assert frame.aircraft_pitch_deg == pytest.approx(-9.1), "estimate"
    assert frame.aircraft_roll_deg == pytest.approx(3.4), "estimate"
    assert frame.aircraft_pitch_deg != pytest.approx(-10.0), "not truth"
    assert frame.aircraft_yaw_rate_rad_s == pytest.approx(0.12)


def test_truth_still_aims_the_ray_when_the_estimate_disagrees():
    """An attitude estimate error must not move where the POI really is."""
    honest = build_frame(
        offset_ned_m=np.array([1000.0, 0.0, 0.0]),
        truth_pitch_deg=0.0, truth_roll_deg=0.0, truth_yaw_deg=25.0,
        est_pitch_deg=0.0, est_roll_deg=0.0,
        airspeed_mps=30.0, yaw_rate_rad_s=0.0, source_timestamp_s=1.0,
    )
    biased = build_frame(
        offset_ned_m=np.array([1000.0, 0.0, 0.0]),
        truth_pitch_deg=0.0, truth_roll_deg=0.0, truth_yaw_deg=25.0,
        est_pitch_deg=-6.0, est_roll_deg=11.0,
        airspeed_mps=30.0, yaw_rate_rad_s=0.0, source_timestamp_s=1.0,
    )
    assert biased.body_ray == pytest.approx(honest.body_ray)
    assert biased.aircraft_pitch_deg == pytest.approx(-6.0)


def test_coordinates_round_trip_through_the_offset():
    """The inverse must return the offset it was given, not approximate it.

    The placement path resolves range/bearing into fixed coordinates, and the
    scoring interval is then flown against those. If the two conversions disagreed,
    every run would start at a slightly different geometry from the one asked
    for, and a systematic placement error reads exactly like a navigation bias.
    """
    from scripts.navigation_truth_sensor import coordinates_from_offset_ned
    origin = {"lat_deg": 40.3117414, "lon_deg": 44.4552111, "alt_m": 1694.86}
    for offset in (
        np.array([2000.0, 0.0, 400.0]),
        np.array([-1500.0, 900.0, -120.0]),
        np.array([0.0, -3000.0, 0.0]),
    ):
        lat, lon, alt = coordinates_from_offset_ned(**origin, offset_ned_m=offset)
        back = poi_offset_ned_m(
            **origin, poi_lat_deg=lat, poi_lon_deg=lon, poi_alt_m=alt,
        )
        assert back == pytest.approx(offset, abs=1e-6)


def test_an_attitude_estimate_error_moves_the_control_ray():
    """The gap between truth and estimate must survive into the command frame.

    The body ray is aimed by truth. If the control frame de-rotated it by truth
    as well, the two would cancel exactly and no attitude error could ever move
    the frame the law commands in -- the harness would report a perfect frame
    from a badly mis-estimating aircraft. The real projector de-rotates by
    `observation.aircraft_pitch_deg`/`aircraft_roll_deg`
    (frame_projection.py:50-53), which are estimates, so this must too.
    """
    honest = build_frame(
        offset_ned_m=np.array([1000.0, 0.0, -50.0]),
        truth_pitch_deg=-10.0, truth_roll_deg=4.0, truth_yaw_deg=20.0,
        est_pitch_deg=-10.0, est_roll_deg=4.0,
        airspeed_mps=30.0, yaw_rate_rad_s=0.0, source_timestamp_s=1.0,
    )
    biased = build_frame(
        offset_ned_m=np.array([1000.0, 0.0, -50.0]),
        truth_pitch_deg=-10.0, truth_roll_deg=4.0, truth_yaw_deg=20.0,
        est_pitch_deg=-16.0, est_roll_deg=15.0,
        airspeed_mps=30.0, yaw_rate_rad_s=0.0, source_timestamp_s=1.0,
    )
    # The world did not move, so the camera did not either.
    assert biased.body_ray == pytest.approx(honest.body_ray)
    # But where the aircraft THINKS it is pointing did, so the command frame did.
    assert biased.control_ray != pytest.approx(honest.control_ray)

    # How far it moves depends on where the POI sits relative to the rotation
    # axes -- a roll error barely moves a POI that is dead ahead -- so no
    # fixed number is asserted. What must hold is that a WORSE estimate gives a
    # WORSE frame, monotonically, and never more error than was put in.
    def separation(pitch_error, roll_error):
        wrong = build_frame(
            offset_ned_m=np.array([1000.0, 0.0, -50.0]),
            truth_pitch_deg=-10.0, truth_roll_deg=4.0, truth_yaw_deg=20.0,
            est_pitch_deg=-10.0 + pitch_error, est_roll_deg=4.0 + roll_error,
            airspeed_mps=30.0, yaw_rate_rad_s=0.0, source_timestamp_s=1.0,
        )
        return math.degrees(math.acos(float(np.clip(
            np.dot(wrong.control_ray, honest.control_ray), -1.0, 1.0))))

    errors = [(1.0, 2.0), (3.0, 6.0), (6.0, 11.0), (12.0, 20.0)]
    moved = [separation(pitch, roll) for pitch, roll in errors]
    assert moved[0] > 1e-9, "any estimate error must move the frame"
    assert moved == sorted(moved), f"not monotonic in the estimate error: {moved}"
    for (pitch, roll), angle in zip(errors, moved):
        assert angle <= math.hypot(pitch, roll) + 1e-9, (
            "the frame cannot be more wrong than the estimate that built it"
        )


def test_a_perfect_estimate_still_gives_the_stabilised_frame():
    """With no estimate error the pitch/roll cancellation must still hold.

    This is the property the previous test replaces truth with estimate for, so
    it is pinned separately: the fix must not have broken the case it was right
    about.
    """
    level = build_frame(offset_ned_m=np.array([1000.0, 0.0, -80.0]), **BASE)
    for pitch, roll in ((-20.0, 0.0), (0.0, 25.0), (-35.0, -15.0)):
        manoeuvring = build_frame(
            offset_ned_m=np.array([1000.0, 0.0, -80.0]),
            **{**BASE, "truth_pitch_deg": pitch, "truth_roll_deg": roll,
               "est_pitch_deg": pitch, "est_roll_deg": roll},
        )
        assert manoeuvring.control_ray == pytest.approx(level.control_ray, abs=1e-9)


def test_the_miss_names_its_channel():
    """A scalar miss cannot say whether roll or pitch missed.

    A pass 1 m directly above the POI is a pitch-law miss with a perfect
    roll law; 1 m abeam is the reverse. The two need different fixes, so the
    closest approach records the north-east magnitude and the down component
    separately -- taken AT the interpolated closest point, not at a sample.
    """
    import numpy as np

    approach = ClosestApproach()
    # Straight pass 3 m east of the POI, 4 m above it: crosses from ahead
    # to behind between the two samples, closest point between them.
    approach.observe(0.0, np.array([10.0, 3.0, 4.0]))
    approach.observe(1.0, np.array([-10.0, 3.0, 4.0]))
    assert approach.miss_m == pytest.approx(5.0)
    assert approach.miss_horizontal_m == pytest.approx(3.0)
    assert approach.miss_vertical_m == pytest.approx(4.0)

    # Found in review: the horizontal must be its OWN minimisation, not the
    # horizontal component at the 3D closest point. This dive crosses the
    # POI's ground position exactly, but the 3D closest point sits near
    # the low end where the horizontal component reads 9.6 m -- scoring the
    # roll law with the pitch law's timing.
    dive = ClosestApproach()
    dive.observe(0.0, np.array([10.0, 0.0, 100.0]))
    dive.observe(1.0, np.array([-10.0, 0.0, 0.0]))
    assert dive.miss_horizontal_m == pytest.approx(0.0, abs=1e-9)
    assert dive.miss_m == pytest.approx(9.806, abs=0.01)
