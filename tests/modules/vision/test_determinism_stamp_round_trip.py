"""Is the slot key exact, through the conversions production actually uses?

The trace does NOT carry the raw ``time_boot_ms`` / ``time_us`` integers; it
reconstructs whole microseconds from the float seconds the pose plumbing already
produced. That is a deliberate deferral, and the condition attached to it was
that the invariant be ENFORCED rather than assumed. This file is that
enforcement.

Two conversions stand between a MAVLink field and a slot key, and both are
production code -- ``pose_telemetry`` for the read, ``determinism_slots`` for
the key. Asserting the arithmetic in isolation would not notice a clock-domain
offset added to ``time_boot_s`` later, which is exactly the failure the
deferral risks. So every case here starts from a message object and ends at the
integer a row would carry, through:

    ATTITUDE.time_boot_ms
      -> attitude_sample_from_message  (float seconds)
      -> AssociatedPose.attitude_timestamp_s
      -> source_seconds_of / source_microseconds  (integer microseconds)

    SIM_STATE.time_us
      -> simulator_truth_source_time_s  (float seconds)
      -> source_microseconds  (integer microseconds)

Ranges are chosen from the campaign, not arbitrarily: an archived vehicle-case
runs a few minutes, and an 11-hour SITL session is ~4e7 ms / ~4e10 us.

What is established here is a VERIFIED-SAFE range, not a first failure. An
earlier version of this file named a specific first inexact stamp; the review
produced a counterexample below it, and so did a closer search of my own
(4882812500006 ms, against the 10000000000005 ms claimed). The original scan
only swept narrow windows above each power of ten, so it could not have found
a first value and should never have claimed one. Finding the true first would
need an exhaustive sweep of ~5e12 values. So the assertions below are: the
operating range round-trips exactly, and a KNOWN inexact value sits ~1e5x
above it. Where exactness ends between those two points is not established,
and nothing here depends on it.
"""

from __future__ import annotations

import random
from types import SimpleNamespace

import pytest

from navpy.modules.vehicle.pose_telemetry import (
    attitude_sample_from_message,
    simulator_truth_source_time_s,
)
from navpy.modules.vision.sim.determinism_slots import (
    slot_index,
    source_microseconds,
    source_seconds_of,
)


SCHEDULER_PERIOD_US = 20_000
# An 11-hour SITL session, in each unit. The operating range this must cover.
ELEVEN_HOURS_MS = 40_000_000
ELEVEN_HOURS_US = 40_000_000_000
# The SMALLEST inexact stamp found, by halving down from 1e16 with dense
# windows. NOT the first -- see the module docstring. ~155 years of uptime.
KNOWN_INEXACT_MS = 4_882_812_500_006
KNOWN_INEXACT_US = 3_906_250_000_000_007


def _attitude(time_boot_ms: int) -> SimpleNamespace:
    return SimpleNamespace(
        pitch=0.0,
        yaw=0.0,
        roll=0.0,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
        time_boot_ms=time_boot_ms,
    )


def _recorded_attitude_us(time_boot_ms: int) -> int | None:
    """What a row would carry, through every production conversion."""
    sample = attitude_sample_from_message(_attitude(time_boot_ms), 100.0)
    associated = SimpleNamespace(attitude_timestamp_s=sample.time_boot_s)
    return source_microseconds(source_seconds_of(associated))


def _recorded_truth_us(time_us: int) -> int | None:
    seconds = simulator_truth_source_time_s(SimpleNamespace(time_us=time_us))
    return source_microseconds(seconds)


@pytest.mark.parametrize(
    "time_boot_ms",
    [1, 7, 999, 1_000, 12_510, 60_000, 157_003, 1_800_000, 39_999_999],
)
def test_named_attitude_stamps_reach_the_row_exactly(time_boot_ms: int) -> None:
    assert _recorded_attitude_us(time_boot_ms) == time_boot_ms * 1_000


def test_every_millisecond_of_the_first_half_hour_round_trips() -> None:
    """Exhaustive, not sampled: a scored leg lives inside this range."""
    bad = [
        ms
        for ms in range(0, 2_000_000)
        if _recorded_attitude_us(ms) != ms * 1_000
    ]
    assert bad == []


def test_eleven_hours_of_random_attitude_stamps_round_trip() -> None:
    """A seeded sample, so a failure names the same value on every run."""
    generator = random.Random(12345)
    bad = [
        ms
        for ms in (
            generator.randrange(0, 40_000_000) for _ in range(200_000)
        )
        if _recorded_attitude_us(ms) != ms * 1_000
    ]
    assert bad == []


def test_eleven_hours_of_random_truth_stamps_round_trip() -> None:
    generator = random.Random(54321)
    bad = [
        us
        for us in (
            generator.randrange(1, 40_000_000_000) for _ in range(200_000)
        )
        if _recorded_truth_us(us) != us
    ]
    assert bad == []


def test_the_float_path_is_not_exact_forever_and_the_break_is_far_away() -> (
    None
):
    """"Exact today" is not "guaranteed", so pin a value that already fails.

    These are the smallest inexact stamps FOUND, not the first that exist. The
    point they make is a margin, not a boundary: if a future clock-domain
    offset ever moves the working range within ~4 orders of magnitude of
    them, this margin is gone and the raw-integer plumbing is due.
    """
    assert _recorded_attitude_us(KNOWN_INEXACT_MS) != KNOWN_INEXACT_MS * 1_000
    assert _recorded_truth_us(KNOWN_INEXACT_US) != KNOWN_INEXACT_US
    # The MEASURED margins over an 11-hour session, stated rather than
    # rounded up to a nicer number: 122070x and 97656x.
    assert KNOWN_INEXACT_MS // ELEVEN_HOURS_MS == 122_070
    assert KNOWN_INEXACT_US // ELEVEN_HOURS_US == 97_656


def test_no_inexact_stamp_in_range_moves_a_scheduler_slot() -> None:
    """The key is a SLOT, so exactness only matters where it changes one."""
    moved = [
        ms
        for ms in range(0, 400_000)
        if slot_index(_recorded_attitude_us(ms), SCHEDULER_PERIOD_US)
        != slot_index(ms * 1_000, SCHEDULER_PERIOD_US)
    ]
    assert moved == []


def test_the_integer_key_removes_the_ulp_hazard_the_float_axis_documents() -> (
    None
):
    """``truth_pose_time_axis`` records ~40% float disagreement. Measure both.

    7 * 1e-3 is 0.007 but 7000 * 1e-6 is 0.006999999999999999. That is why slot
    keys are re-derived as real integers instead of reusing the axis helper's
    whole-microsecond-valued floats.
    """
    float_disagreements = sum(
        1
        for ms in range(1, 100_000)
        if float(ms) * 1e-3 != float(ms * 1_000) * 1e-6
    )
    assert float_disagreements > 30_000

    integer_disagreements = sum(
        1
        for ms in range(1, 100_000)
        if _recorded_attitude_us(ms) != _recorded_truth_us(ms * 1_000)
    )
    assert integer_disagreements == 0
