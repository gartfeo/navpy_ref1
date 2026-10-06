from __future__ import annotations

from types import SimpleNamespace

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.sim.truth_pose_history import (
    TIME_AXIS_RECEIPT,
    TIME_AXIS_SOURCE,
    TRUTH_POSE_TIME_AXIS_ENV,
    TruthPoseHistory,
    resolve_truth_pose_time_axis,
    wrap_degrees,
)


def _pose(
    receipt_s: float,
    *,
    yaw: float = 0.0,
    lat: float = 40.0,
    alt: float = 1000.0,
    source_s: float | None = None,
) -> SimpleNamespace:
    return SimpleNamespace(
        location=Location(lat, 44.0, alt, is_absolute=True),
        attitude=Attitude(pitch=-4.0, yaw=yaw, roll=3.0),
        receipt_time_s=receipt_s,
        source_time_s=source_s,
    )


def _history(
    span_s: float = 0.05, time_axis: str = TIME_AXIS_RECEIPT
) -> tuple[TruthPoseHistory, list]:
    poses: list = []
    history = TruthPoseHistory(
        truth_pose=lambda: poses[-1] if poses else None,
        max_span_s=lambda: span_s,
        time_axis=time_axis,
    )
    return history, poses


def _note(history: TruthPoseHistory, poses: list, pose: SimpleNamespace) -> bool:
    poses.append(pose)
    return history.note_current()


def test_two_samples_interpolate_the_pose_at_the_target_time() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=100.0, lat=40.0, alt=1000.0))
    _note(history, poses, _pose(100.025, yaw=101.0, lat=40.001, alt=999.0))

    interpolated = history.pose_at(100.0125)

    assert interpolated is not None
    location, attitude = interpolated
    assert attitude.yaw == pytest.approx(100.5)
    assert attitude.pitch == pytest.approx(-4.0)
    assert location.lat == pytest.approx(40.0005)
    assert location.alt == pytest.approx(999.5)
    assert location.is_absolute


def test_a_clock_past_the_newest_sample_is_refused() -> None:
    """The contract is interpolation only -- never a forward guess.

    Carrying the pair's rate past the newest sample leaves a residual
    whenever the aircraft is angularly ACCELERATING, and the navigation law
    differentiates bearing, so that residual comes back as line-of-sight
    rate noise. The caller drops such a clock and renders the next one.
    """
    history, poses = _history(span_s=0.05)
    _note(history, poses, _pose(100.0, yaw=0.0))
    _note(history, poses, _pose(100.025, yaw=1.0))

    assert history.pose_at(100.0251) is None
    assert history.diagnostics["forward_refused"] == 1.0
    # The newest sample itself is still inside the pair.
    assert history.pose_at(100.025)[1].yaw == pytest.approx(1.0)


def test_the_previous_sample_is_the_far_edge_of_the_bracket() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=100.0))
    _note(history, poses, _pose(100.025, yaw=101.0))

    assert history.pose_at(100.0)[1].yaw == pytest.approx(100.0)


def test_a_missing_pose_reports_that_nothing_was_recorded() -> None:
    """The caller renders on the sample that closes a bracket.

    A truth message whose pose cannot be read adds no sample, so the pair
    still ENDS before the pending clock. If this returned True the caller
    would ask for a pose past the newest sample -- the extrapolation the
    class refuses -- so the answer has to be reported, not assumed.
    """
    history, poses = _history()
    assert _note(history, poses, _pose(100.0)) is True
    assert _note(history, poses, _pose(100.025)) is True

    poses.append(None)
    assert history.note_current() is False
    # The pair is untouched, so the old bracket still answers.
    assert history.pose_at(100.0125) is not None


def test_time_inside_the_bracket_interpolates() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=100.0))
    _note(history, poses, _pose(100.025, yaw=101.0))

    advanced = history.pose_at(100.0125)

    assert advanced is not None
    assert advanced[1].yaw == pytest.approx(100.5)


def test_yaw_interpolates_through_the_wrap_without_a_jump() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=179.0))
    _note(history, poses, _pose(100.025, yaw=-179.0))

    interpolated = history.pose_at(100.0125)

    assert interpolated is not None
    # Halfway across a 2 deg step that crosses 180, not 358 deg the other way.
    assert interpolated[1].yaw == pytest.approx(180.0) or interpolated[
        1
    ].yaw == pytest.approx(-180.0)


def test_single_sample_cannot_support_a_rate() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0))

    assert history.pose_at(100.02) is None


def test_over_wide_sample_gap_is_refused() -> None:
    history, poses = _history(span_s=0.05)
    _note(history, poses, _pose(100.0))
    _note(history, poses, _pose(100.06))

    assert history.pose_at(100.07) is None


def test_target_time_past_the_newest_sample_is_refused() -> None:
    history, poses = _history(span_s=0.05)
    _note(history, poses, _pose(100.0))
    _note(history, poses, _pose(100.025))

    assert history.pose_at(100.025 + 0.051) is None


def test_target_time_before_the_bracket_is_refused() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0))
    _note(history, poses, _pose(100.025))

    assert history.pose_at(99.9) is None


def test_equal_receipt_replaces_content_but_is_not_a_second_point() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=1.0))
    _note(history, poses, _pose(100.0, yaw=2.0))

    assert history.pose_at(100.0) is None

    _note(history, poses, _pose(100.025, yaw=3.0))
    interpolated = history.pose_at(100.0125)

    assert interpolated is not None
    # The replaced content (yaw 2), not the original (yaw 1), is the far edge.
    assert interpolated[1].yaw == pytest.approx(2.5)


def test_equal_receipt_after_a_clean_pair_drops_the_pair() -> None:
    """Two distinct packets can share one coarse wall-clock quantum.

    Keeping the older base under the replaced content would measure two
    sample steps of pose change over one receipt step of time and double the
    rate. The history must fail closed instead.
    """
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=1.0))
    _note(history, poses, _pose(100.025, yaw=2.0))
    _note(history, poses, _pose(100.025, yaw=3.0))

    assert history.pose_at(100.05) is None


def test_regressing_receipt_fails_closed() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0, yaw=1.0))
    _note(history, poses, _pose(100.025, yaw=2.0))
    _note(history, poses, _pose(100.010, yaw=3.0))

    assert history.pose_at(100.03) is None


def test_a_tick_wide_gap_still_positions_the_clock_exactly() -> None:
    """A 1 ms receipt gap must interpolate on its OWN gap, unclamped.

    Two sends drained together land stamps one clock tick apart. While the
    gap was a rate denominator carried FORWARD, dividing a full pose step by
    microseconds fabricated kilometres, so a floor was clamped under it.
    Inside a bracket that floor is a bug, not a guard: it shrinks the
    fraction and answers a different clock than the one asked for (review
    probe: the midpoint of this pair read 0.92 deg instead of 0.5). The
    answer cannot run away, because it stays between the two samples.
    """
    history, poses = _history(span_s=0.05)
    _note(history, poses, _pose(100.0, yaw=0.0, lat=40.0))
    _note(history, poses, _pose(100.001, yaw=1.0, lat=40.00001))

    midpoint = history.pose_at(100.0005)

    assert midpoint is not None
    assert midpoint[1].yaw == pytest.approx(0.5)
    assert midpoint[0].lat == pytest.approx(40.000005)
    # And the far edge is the previous sample itself, not a walked-back guess.
    assert history.pose_at(100.0)[1].yaw == pytest.approx(0.0)


def test_a_wide_gap_still_inside_the_span_interpolates_on_its_own_gap() -> None:
    """A ~40 ms gap is 40 ms of emission spacing, not a dropped sample.

    Scored-leg diagnostics (2026-08-24, speedup 1) showed no truth drops --
    wide gaps form one continuous spacing tail. A clock 25 ms after the
    older sample therefore sits 25/40 of the way along the step.
    """
    history, poses = _history(span_s=0.05)
    _note(history, poses, _pose(100.0, yaw=1.0))
    _note(history, poses, _pose(100.04, yaw=2.0))

    interpolated = history.pose_at(100.025)

    assert interpolated is not None
    assert interpolated[1].yaw == pytest.approx(1.625)


def test_missing_truth_pose_is_ignored() -> None:
    history, _poses = _history()

    assert history.note_current() is False

    assert history.pose_at(100.0) is None


def test_diagnostics_classify_every_refusal_and_acceptance() -> None:
    history, poses = _history(span_s=0.05)
    assert history.pose_at(100.0) is None                    # unpaired
    _note(history, poses, _pose(100.0, yaw=0.0))
    assert history.pose_at(100.0) is None                    # still unpaired
    _note(history, poses, _pose(100.058, yaw=1.0))
    assert history.pose_at(100.06) is None                   # wide (past gate)
    _note(history, poses, _pose(100.083, yaw=2.0))
    assert history.pose_at(100.14) is None                   # past the newest
    assert history.pose_at(100.07) is not None               # accepted

    diagnostics = history.diagnostics
    assert diagnostics["unpaired"] == 2.0
    assert diagnostics["gap_wide"] == 1.0
    assert diagnostics["gap_wide_max_ms"] == pytest.approx(58.0)
    assert diagnostics["gap_wide_min_ms"] == pytest.approx(58.0)
    assert diagnostics["forward_refused"] == 1.0
    assert diagnostics["accepted"] == 1.0


def test_reset_diagnostics_restarts_the_tallies() -> None:
    history, poses = _history()
    assert history.pose_at(100.0) is None
    _note(history, poses, _pose(100.0))
    _note(history, poses, _pose(100.025))
    assert history.pose_at(100.0125) is not None

    history.reset_diagnostics()

    diagnostics = history.diagnostics
    assert diagnostics["accepted"] == 0.0
    assert diagnostics["unpaired"] == 0.0
    assert diagnostics["gap_wide_min_ms"] == 0.0


def test_reset_pair_drops_the_held_pair() -> None:
    history, poses = _history()
    _note(history, poses, _pose(100.0))
    _note(history, poses, _pose(100.025))
    assert history.pose_at(100.0125) is not None

    history.reset_pair()

    assert history.pose_at(100.0125) is None


def test_wrap_degrees_maps_onto_the_half_open_range() -> None:
    assert wrap_degrees(180.0) == -180.0
    assert wrap_degrees(-180.0) == -180.0
    assert wrap_degrees(358.0) == pytest.approx(-2.0)
    assert wrap_degrees(-358.0) == pytest.approx(2.0)


# --------------------------------------------------------------------------
# source time axis (SIM_STATE ``time_us`` navlink extension)
# --------------------------------------------------------------------------


def test_source_axis_interpolates_on_source_stamps_not_receipts() -> None:
    """Jittered receipts must not move the answer when source stamps exist.

    The receipts here are deliberately skewed (a 30 ms receipt gap over a
    25 ms source gap): on the receipt axis the query below would land at a
    different fraction. The source axis must position purely by ``time_us``.
    """
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=100.0, source_s=50.000))
    _note(history, poses, _pose(100.030, yaw=101.0, source_s=50.025))

    interpolated = history.pose_at(50.0125)

    assert interpolated is not None
    assert interpolated[1].yaw == pytest.approx(100.5)
    # Receipt-axis clocks are meaningless on this axis.
    assert history.pose_at(100.015) is None


def test_source_axis_without_stamps_falls_back_to_receipt_sticky() -> None:
    """Stock firmware sends no ``time_us``: degrade to receipts, once.

    The fallback must be sticky — flipping per sample would put the two
    ends of a pair on different clocks — and must be visible in the
    diagnostics so a scored A/B arm cannot silently fly the wrong axis.
    """
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=1.0))
    _note(history, poses, _pose(100.025, yaw=2.0))

    assert history.axis_in_use == TIME_AXIS_RECEIPT
    assert history.pose_at(100.0125)[1].yaw == pytest.approx(1.5)
    assert history.diagnostics["source_axis_fallback"] == 1.0

    # Later stamped samples do NOT flip back: still the receipt axis.
    _note(history, poses, _pose(100.050, yaw=3.0, source_s=50.050))
    assert history.axis_in_use == TIME_AXIS_RECEIPT
    assert history.pose_at(100.0375)[1].yaw == pytest.approx(2.5)
    assert history.diagnostics["source_axis_fallback"] == 1.0


def test_source_axis_fallback_drops_the_stamped_pair() -> None:
    """An axis flip mid-run must not leave a pair straddling two clocks."""
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=1.0, source_s=50.000))
    _note(history, poses, _pose(100.025, yaw=2.0, source_s=50.025))
    assert history.pose_at(50.0125) is not None

    _note(history, poses, _pose(100.050, yaw=3.0))  # no stamp: fallback

    # The old pair is gone; only a fresh receipt-ordered pair answers.
    assert history.pose_at(100.0375) is None
    _note(history, poses, _pose(100.075, yaw=4.0))
    assert history.pose_at(100.0625)[1].yaw == pytest.approx(3.5)


def test_source_axis_regressing_source_stamp_fails_closed() -> None:
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=1.0, source_s=50.000))
    _note(history, poses, _pose(100.025, yaw=2.0, source_s=50.025))
    _note(history, poses, _pose(100.050, yaw=3.0, source_s=50.010))

    assert history.pose_at(50.030) is None


def test_source_axis_same_tick_query_is_not_refused_by_float_rounding() -> None:
    """A query on the SAME autopilot tick as the newest stamp must land ON it.

    One integer clock reaches this class through two float conversions: the
    query as ATTITUDE ``time_boot_ms * 1e-3`` and the stamp as SIM_STATE
    ``time_us * 1e-6``. For ~40% of tick values the products round to
    different doubles (12.515 vs 12.514999999999999 here), and a raw float
    compare called that one-ulp excess extrapolation and dropped the frame.
    """
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=1.0, source_s=12_490_000 * 1e-6))
    _note(history, poses, _pose(100.030, yaw=2.0, source_s=12_515_000 * 1e-6))

    same_tick_query = 12_515 * 1e-3
    # The rounding gap this test exists for must actually be present.
    assert same_tick_query > 12_515_000 * 1e-6

    interpolated = history.pose_at(same_tick_query)

    assert interpolated is not None
    assert interpolated[1].yaw == pytest.approx(2.0)
    assert history.diagnostics["forward_refused"] == 0.0


def test_reset_diagnostics_keeps_the_fallback_count() -> None:
    """A fallback is a run-level configuration fact, not a leg tally."""
    history, poses = _history(time_axis=TIME_AXIS_SOURCE)
    _note(history, poses, _pose(100.000, yaw=1.0))
    assert history.diagnostics["source_axis_fallback"] == 1.0

    history.reset_diagnostics()

    assert history.diagnostics["source_axis_fallback"] == 1.0


def test_unknown_time_axis_raises_at_construction() -> None:
    with pytest.raises(ValueError):
        TruthPoseHistory(
            truth_pose=lambda: None,
            max_span_s=lambda: 0.05,
            time_axis="bogus",
        )


def test_resolver_defaults_normalizes_and_rejects() -> None:
    # Default flipped to source after the 2026-08-27 confirmation A/B.
    assert resolve_truth_pose_time_axis({}) == TIME_AXIS_SOURCE
    assert (
        resolve_truth_pose_time_axis({TRUTH_POSE_TIME_AXIS_ENV: "source"})
        == TIME_AXIS_SOURCE
    )
    assert (
        resolve_truth_pose_time_axis({TRUTH_POSE_TIME_AXIS_ENV: " Receipt "})
        == TIME_AXIS_RECEIPT
    )
    with pytest.raises(ValueError):
        resolve_truth_pose_time_axis({TRUTH_POSE_TIME_AXIS_ENV: "wall"})
