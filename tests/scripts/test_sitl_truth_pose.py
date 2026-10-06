"""The pose gates must REJECT bad feeds, not merely accept good ones.

Each test here builds a feed that is wrong in one specific way and asserts the
gate refuses it. A gate that only ever passes is indistinguishable from no gate.
"""

from __future__ import annotations

import math

import pytest

from scripts.sitl_truth_pose import (
    MAX_POSE_GAP_S,
    MIN_POSE_SAMPLES,
    TruthPoseStream,
)


class _Message:
    def __init__(self, kind: str, **fields) -> None:
        self._kind = kind
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self._kind


def _attitude(t_s: float, *, est_pitch_deg: float = -9.0,
              est_roll_deg: float = 2.0, yawspeed: float = 0.11) -> _Message:
    """ATTITUDE as ArduPilot sends it: radians, and the ESTIMATE not the truth.

    The estimate deliberately differs from the truth values in _truth below, so
    any test that confuses the two fails instead of silently agreeing.
    """
    return _Message(
        "ATTITUDE",
        time_boot_ms=int(round(t_s * 1000.0)),
        roll=math.radians(est_roll_deg), pitch=math.radians(est_pitch_deg),
        yaw=math.radians(123.0),
        rollspeed=0.01, pitchspeed=0.02, yawspeed=yawspeed,
    )


def _truth(yaw_deg: float = 0.0, lat_deg: float = 40.0,
           lon_deg: float = 44.0) -> _Message:
    """SIM_STATE as ArduPilot sends it: attitude in RADIANS, lat/lon in DEGREES.

    The mixed units are the message's, not a convenience of this fake. Writing
    lat/lon as radians here would let a decoder that converts them pass, which
    is exactly the bug that reached the aircraft at latitude 2291 degrees.
    """
    return _Message(
        "SIM_STATE",
        lat=lat_deg, lon=lon_deg, alt=1000.0,
        lat_int=0, lon_int=0,
        roll=0.0, pitch=math.radians(-10.0), yaw=math.radians(yaw_deg),
        xgyro=0.0, ygyro=0.0, zgyro=0.25,
    )


def _feed(stream: TruthPoseStream, count: int, *, step_s: float = 0.02,
          truth_every: int = 1):
    """Interleave the two streams. `truth_every` starves SIM_STATE alone.

    The first ATTITUDE can never complete a pose: with no earlier ATTITUDE there
    is no interval to bound the truth sample's time against.
    """
    poses = []
    for index in range(count):
        if index % truth_every == 0:
            stream.absorb(_truth())
        pose = stream.absorb(_attitude(index * step_s))
        if pose is not None:
            poses.append(pose)
    return poses


def test_healthy_feed_certifies():
    stream = TruthPoseStream()
    poses = _feed(stream, 61)
    assert len(poses) == 60
    assert stream.certification_error() is None
    assert stream.observed_rate_hz == pytest.approx(50.0, rel=1e-6)


def test_too_few_poses_is_rejected():
    stream = TruthPoseStream()
    _feed(stream, MIN_POSE_SAMPLES - 1)
    error = stream.certification_error()
    assert error is not None and "insufficient pose cadence" in error


def test_starved_truth_stream_is_rejected_though_each_pose_is_valid():
    """The exact silent failure this gate exists for.

    ATTITUDE was raised to 50 Hz but SIM_STATE was not, so every pose that does
    come out is tightly paired and perfectly well formed -- only sparse. This is
    what "the request was accepted but only one stream was honoured" looks like
    from the receiving end, and nothing about an individual pose reveals it.
    """
    stream = TruthPoseStream()
    poses = _feed(stream, 400, truth_every=10)
    assert len(poses) >= MIN_POSE_SAMPLES, "poses are individually valid"
    assert max(pose.skew_s for pose in poses) == pytest.approx(0.02), (
        "each pair is tight; only the rate is wrong"
    )
    error = stream.certification_error()
    assert error is not None and "pose gap" in error


def test_skewed_pair_is_dropped_not_emitted():
    """Truth from one instant must never be stamped with a distant clock."""
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.0))
    stream.absorb(_truth())
    assert stream.absorb(_attitude(0.5)) is None
    assert stream.pose_count == 0
    assert stream.summary()["rejected_for_skew"] >= 1


def test_stale_truth_is_not_carried_into_a_later_interval():
    """One truth sample may complete one pose, never two."""
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.00))
    stream.absorb(_truth())
    assert stream.absorb(_attitude(0.02)) is not None
    assert stream.absorb(_attitude(0.04)) is None
    assert stream.pose_count == 1


def test_attitude_without_truth_emits_nothing():
    stream = TruthPoseStream()
    for index in range(10):
        assert stream.absorb(_attitude(index * 0.02)) is None
    assert stream.pose_count == 0


def test_lat_lon_pass_through_as_degrees():
    """SIM_STATE lat/lon are already DEGREES, unlike its radian attitude.

    Converting them as if they were radians put the aircraft at latitude 2291,
    which is not merely wrong but outside the range latitude can take -- and it
    survived because the test fake had been written with the same mistake.
    """
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.0))
    stream.absorb(_truth(lat_deg=40.311741, lon_deg=44.455211))
    pose = stream.absorb(_attitude(0.02))
    assert pose is not None
    assert pose.lat_deg == pytest.approx(40.311741, abs=1e-9)
    assert pose.lon_deg == pytest.approx(44.455211, abs=1e-9)
    assert -90.0 <= pose.lat_deg <= 90.0
    assert -180.0 <= pose.lon_deg <= 180.0


def test_attitude_is_radians_even_though_coordinates_are_not():
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.0))
    stream.absorb(_truth(yaw_deg=-30.0))
    pose = stream.absorb(_attitude(0.02))
    assert pose is not None
    assert pose.yaw_deg == pytest.approx(330.0, abs=1e-9)
    assert pose.pitch_deg == pytest.approx(-10.0, abs=1e-9)


def test_truth_and_estimate_are_kept_apart():
    """The law may read the estimate; only the sensor may read the truth.

    Truth pitch is -10 and estimated pitch is -9 here, so a mix-up cannot pass
    unnoticed. Body rates likewise come from ATTITUDE, not from SIM_STATE's
    gyros, because a real aircraft has no truth rate to read.
    """
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.0))
    stream.absorb(_truth())
    pose = stream.absorb(_attitude(0.02))
    assert pose is not None
    assert pose.pitch_deg == pytest.approx(-10.0, abs=1e-9), "truth"
    assert pose.est_pitch_deg == pytest.approx(-9.0, abs=1e-9), "estimate"
    assert pose.est_roll_deg == pytest.approx(2.0, abs=1e-9)
    assert pose.est_yaw_rate_rad_s == pytest.approx(0.11), "from ATTITUDE"
    assert pose.est_yaw_rate_rad_s != pytest.approx(0.25), "not SIM_STATE zgyro"


def test_deg_e7_integer_fields_win_when_present():
    """Some builds carry the coordinates in lat_int/lon_int as degE7 instead.

    sim_state_coordinates_deg prefers those when they are non-zero, which is why
    this module defers to it rather than reading message.lat directly.
    """
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.0))
    stream.absorb(_Message(
        "SIM_STATE", lat=0.0, lon=0.0, alt=1000.0,
        lat_int=403117410, lon_int=444552110,
        roll=0.0, pitch=0.0, yaw=0.0, xgyro=0.0, ygyro=0.0, zgyro=0.0,
    ))
    pose = stream.absorb(_attitude(0.02))
    assert pose is not None
    assert pose.lat_deg == pytest.approx(40.311741, abs=1e-6)
    assert pose.lon_deg == pytest.approx(44.455211, abs=1e-6)


def test_time_is_the_autopilot_clock_not_the_host():
    """A pose must carry simulated time, so host stalls cannot move it."""
    stream = TruthPoseStream()
    stream.absorb(_attitude(12.325))
    stream.absorb(_truth())
    pose = stream.absorb(_attitude(12.345))
    assert pose is not None
    assert pose.t_s == pytest.approx(12.345)


def test_repeated_clock_does_not_emit_duplicate_poses():
    stream = TruthPoseStream()
    stream.absorb(_attitude(0.98))
    stream.absorb(_truth())
    assert stream.absorb(_attitude(1.0)) is not None
    stream.absorb(_truth())
    assert stream.absorb(_attitude(1.0)) is None
    assert stream.pose_count == 1


def test_gap_threshold_is_the_boundary_it_claims():
    """Held just inside and just outside MAX_POSE_GAP_S, nothing else changed.

    ATTITUDE stays at 50 Hz in both, so the skew stays tight and the ONLY
    difference is how often truth arrives: every 4th attitude is a 0.08 s pose
    gap and passes, every 8th is 0.16 s and fails.
    """
    inside = TruthPoseStream()
    _feed(inside, 400, truth_every=4)
    assert inside.certification_error() is None
    assert inside.summary()["worst_gap_s"] == pytest.approx(0.08)

    outside = TruthPoseStream()
    _feed(outside, 400, truth_every=8)
    error = outside.certification_error()
    assert error is not None and "pose gap" in error


def test_attitude_interval_is_what_bounds_the_skew():
    """SIM_STATE has no clock, so slow ATTITUDE alone destroys the pose.

    Even with truth arriving before every single attitude, a 10 Hz attitude
    stream can only place that truth within a 100 ms window -- which is the
    250 ms-class staleness this gate exists to refuse.
    """
    stream = TruthPoseStream()
    poses = _feed(stream, 200, step_s=0.1)
    assert poses == []
    assert stream.summary()["rejected_for_skew"] > 0
    assert stream.certification_error() is not None


def test_truth_stall_after_minimum_samples_is_rejected():
    """A stall AFTER the minimum sample count must not certify.

    Gaps are only recorded when a pose is emitted, so a truth stream that dies
    records no further gap and leaves the last one looking healthy. The run then
    flew mostly blind on held commands while its own record claimed a good feed.
    Only the clock advancing past the last pose reveals it.
    """
    stream = TruthPoseStream()
    _feed(stream, MIN_POSE_SAMPLES + 1)
    assert stream.certification_error() is None, "healthy prefix certifies"
    last = stream.pose_count
    # Truth dies; the autopilot keeps talking for another 10 simulated seconds.
    for index in range(500):
        stream.absorb(_attitude((MIN_POSE_SAMPLES + 1 + index) * 0.02))
    assert stream.pose_count == last, "no new poses were produced"
    error = stream.certification_error()
    assert error is not None and "stall" in error


def test_short_trailing_stall_still_certifies():
    """The stall gate must not reject an ordinary end-of-run tail.

    A run stops between messages, so a little simulated time after the last pose
    is normal. Only a stall longer than one permitted gap is a fault.
    """
    stream = TruthPoseStream()
    _feed(stream, MIN_POSE_SAMPLES + 1)
    end = (MIN_POSE_SAMPLES + 1) * 0.02
    stream.absorb(_attitude(end + MAX_POSE_GAP_S * 0.5))
    assert stream.certification_error() is None


def test_one_truth_sample_is_measured_against_one_attitude_only():
    """The arrival offset diagnoses the pairing, so it must not inflate itself.

    The two streams are not guaranteed to alternate --
    `test_starved_truth_stream_is_rejected_though_each_pose_is_valid` above is
    built on them not doing so. When several ATTITUDE messages follow one
    SIM_STATE, only the first closes its interval; measuring the later ones
    against the same truth sample adds samples that grow with the gap and drags
    the median and p95 upward, which is precisely backwards for a statistic
    whose whole job is to reveal how far apart the streams really are.
    """
    stream = TruthPoseStream()
    stream.absorb(_truth())
    for index in range(4):
        stream.absorb(_attitude(0.02 * (index + 1)))
    offsets = stream.summary()["arrival_offset_s"]
    assert offsets is not None
    assert offsets["samples"] == 1, (
        "one truth sample must contribute exactly one arrival measurement"
    )


def _truth_degE7(lat_deg: float = 40.3149435,
                 lon_deg: float = 44.4396165) -> _Message:
    """SIM_STATE from a link that DOES populate lat_int/lon_int.

    The module-level `_truth` helper leaves them zero, which is the DEGRADED
    float32 path -- so a test that wants the good path has to say so.
    """
    message = _truth(lat_deg=lat_deg, lon_deg=lon_deg)
    message.lat_int = int(round(lat_deg * 1e7))
    message.lon_int = int(round(lon_deg * 1e7))
    return message


def test_the_coordinate_decode_path_is_recorded():
    """Which decode path ran decides whether a miss is measurable at all.

    `sim_state_coordinates_deg` prefers degE7 lat_int/lon_int and falls back to
    SIM_STATE's float32 lat/lon. At this latitude float32 quantises to 0.42 m
    north and 0.32 m east -- large enough to manufacture miss differences of
    0.05-0.17 m, which is the size of the effects this harness measures. A
    MAVLink1 link or an older firmware is enough to take that path, and nothing
    in a result would otherwise reveal it.
    """
    good = TruthPoseStream()
    for index in range(6):
        good.absorb(_truth_degE7())
        good.absorb(_attitude(index * 0.02))
    assert good.summary()["coordinate_source"] == "degE7"

    degraded = TruthPoseStream()
    for index in range(6):
        degraded.absorb(_truth())
        degraded.absorb(_attitude(index * 0.02))
    assert degraded.summary()["coordinate_source"] == "float32"

    # A link that changes mid-run leaves the scoring interval with two different
    # position qualities, which is neither of the clean answers and must not be
    # reported as one.
    changed = TruthPoseStream()
    for index in range(6):
        changed.absorb(_truth_degE7() if index < 3 else _truth())
        changed.absorb(_attitude(index * 0.02))
    assert changed.summary()["coordinate_source"].startswith("mixed")

    assert TruthPoseStream().summary()["coordinate_source"] is None
