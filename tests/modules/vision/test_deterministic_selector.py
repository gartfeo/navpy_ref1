"""Literal source-time oracles and fail-closed lifetime checks."""
from dataclasses import FrozenInstanceError

import pytest

from navpy.modules.vision.sim.deterministic_selection_types import (
    SelectionConfig, SelectionStream, SlotKey, SourceSample,
)
from navpy.modules.vision.sim.deterministic_selector import DeterministicSelector

A, T, V = SelectionStream


def s(stamp: int, payload: bytes = b"value", epoch: int = 3) -> SourceSample:
    return SourceSample(epoch, stamp, payload)


def kernel(*, prime: bool = True, **overrides: int) -> DeterministicSelector:
    config = dict(boot_epoch=3, period_us=10, start_slot=1,
                  max_truth_gap_us=10, capacity_per_stream=32, end_slot_exclusive=1000)
    config.update(overrides)
    result = DeterministicSelector(SelectionConfig(**config))
    if prime:
        for stream in SelectionStream:
            assert result.offer(stream, s(0, epoch=config["boot_epoch"]))
    return result


def offer(target: DeterministicSelector, stream: SelectionStream, *stamps: int) -> None:
    for stamp in stamps:
        assert target.offer(stream, s(stamp))


def test_half_open_selection_decimation_and_first_closing_samples() -> None:
    target = kernel()
    offer(target, A, 10, 11, 19)
    offer(target, T, 10, 20, 30)
    offer(target, V, 10, 18, 19, 40)
    assert target.poll() is None
    offer(target, A, 20, 40)
    decision = target.poll()
    assert decision.slot == SlotKey(3, 1)
    assert decision.outcome == "fresh"
    assert decision.attitude == s(19)
    assert decision.decimated == (s(10), s(11))
    assert decision.truth == (s(10), s(20))
    assert decision.airspeed == s(19)
    assert decision.closing.attitude == s(20)
    assert decision.closing.truth == s(20)
    assert decision.closing.airspeed == s(19)
    following = target.poll()
    assert following.slot == SlotKey(3, 2)
    assert following.attitude == s(20)
    assert following.truth == (s(20), s(20))
    assert following.airspeed == s(19)
    assert following.closing.airspeed == s(40)
    assert following.decimated == ()


@pytest.mark.parametrize("missing", [T, V])
def test_each_stream_closes_its_own_selection(missing: SelectionStream) -> None:
    target = kernel()
    offer(target, A, 17, 100)
    offer(target, T, 10, 16 if missing == T else 20)
    offer(target, V, 10, 16 if missing == V else 100)
    assert target.poll() is None
    assert target.violation is None
    offer(target, missing, 17)
    assert target.poll().attitude == s(17)


def test_exact_truth_endpoint_needs_no_future_truth_or_gap() -> None:
    target = kernel()
    offer(target, A, 17, 20)
    offer(target, T, 17)
    offer(target, V, 17)
    decision = target.poll()
    assert decision.truth == (s(17), s(17))
    assert decision.airspeed == s(17)
    assert target.violation is None


def test_empty_slot_is_distinct_from_pending_association() -> None:
    target = kernel()
    offer(target, A, 20)
    decision = target.poll()
    assert decision.slot == SlotKey(3, 1)
    assert decision.outcome == "empty"
    assert (decision.attitude, decision.truth, decision.airspeed) == (None, None, None)
    assert decision.decimated == ()
    assert decision.closing.attitude == s(20)
    assert decision.closing.truth is None and decision.closing.airspeed is None
    offer(target, A, 30)
    assert target.poll() is None
    assert target.violation is None


@pytest.mark.parametrize("stream", [A, T, V])
@pytest.mark.parametrize("first", [10, 11, 20])
def test_late_feed_cannot_masquerade_as_empty_or_missing_predecessor(
    stream: SelectionStream, first: int,
) -> None:
    target = kernel(prime=False)
    assert not target.offer(stream, s(first))
    assert target.violation.reason == "feed_started_late"
    assert target.poll() is None


def test_empty_slot_still_waits_for_all_streams_to_prove_start_coverage() -> None:
    target = kernel(prime=False)
    offer(target, A, 0, 20)
    offer(target, T, 0)
    assert target.poll() is None
    offer(target, V, 0)
    assert target.poll().outcome == "empty"


def test_latest_candidate_is_not_replaced_when_truth_gap_is_excessive() -> None:
    target = kernel()
    offer(target, A, 10, 19, 20)
    offer(target, T, 10, 21)
    offer(target, V, 20)
    assert target.poll() is None
    assert target.violation.reason == "truth_gap"
    # 10 had an exact truth match; silently falling back to it would conceal 19.


@pytest.mark.parametrize("limit,accepted", [(20_000, False), (40_000, True)])
def test_real_scheduler_units_and_two_period_truth_gap(limit: int, accepted: bool) -> None:
    target = kernel(period_us=20_000, max_truth_gap_us=limit)
    offer(target, A, 20_000, 40_000)
    offer(target, T, 40_000)
    offer(target, V, 40_000)
    decision = target.poll()
    if accepted:
        assert decision.truth == (s(0), s(40_000))
    else:
        assert decision is None
        assert target.violation.reason == "truth_gap"


def test_prestart_retention_preserves_latest_predecessor_without_cruise_buildup() -> None:
    target = kernel(start_slot=100, capacity_per_stream=2)
    for stamp in range(1, 1000):
        for stream in SelectionStream:
            assert target.offer(stream, s(stamp))
    offer(target, A, 1001, 1010)
    offer(target, T, 1005)
    offer(target, V, 1005)
    decision = target.poll()
    assert decision.slot == SlotKey(3, 100)
    assert decision.attitude == s(1001)
    assert decision.decimated == ()
    assert decision.truth == (s(999), s(1005))
    assert decision.airspeed == s(999)


def test_predecessors_survive_many_slots_with_bounded_storage() -> None:
    target = kernel(capacity_per_stream=3)
    offer(target, A, 17, 20)
    offer(target, T, 10, 20)
    offer(target, V, 10, 20)
    assert target.poll().attitude == s(17)
    for boundary in range(30, 1000, 10):
        for stream in SelectionStream:
            offer(target, stream, boundary)
        decision = target.poll()
        assert decision.attitude == s(boundary - 10)
        assert decision.truth == (s(boundary - 10), s(boundary - 10))
        assert decision.airspeed == s(boundary - 10)
    assert target.violation is None


def test_truth_predecessor_before_boundary_is_needed_in_the_next_slot() -> None:
    target = kernel()
    offer(target, T, 18, 22)
    offer(target, V, 25)
    offer(target, A, 19, 21, 30)
    assert target.poll().truth == (s(18), s(22))
    following = target.poll()
    assert following.slot == SlotKey(3, 2)
    assert following.attitude == s(21)
    assert following.truth == (s(18), s(22))
    assert following.airspeed == s(0)
    assert following.closing.attitude == s(30)
    assert following.closing.truth == s(22)
    assert following.closing.airspeed == s(25)


def test_attitude_does_not_retain_an_unneeded_predecessor_against_capacity() -> None:
    target = kernel(capacity_per_stream=2)
    offer(target, T, 10)
    offer(target, V, 10)
    offer(target, A, 10, 20)
    assert target.poll().attitude == s(10)
    offer(target, T, 20)
    offer(target, V, 20)
    offer(target, A, 30)
    assert target.poll().attitude == s(20)
    assert target.violation is None


@pytest.mark.parametrize("stream", [A, T, V])
@pytest.mark.parametrize("stamp,payload,reason", [
    (9, b"value", "source_regressed"), (10, b"value", "source_repeated"),
    (10, b"changed", "conflicting_duplicate"),
])
def test_order_faults_latch_first_invalidity(
    stream: SelectionStream, stamp: int, payload: bytes, reason: str,
) -> None:
    target = kernel()
    offer(target, stream, 10)
    assert not target.offer(stream, s(stamp, payload))
    violation = target.violation
    assert violation.reason == reason
    assert violation.stream == stream
    assert not target.offer(A, s(100))
    assert target.poll() is None
    assert target.violation is violation
    assert target.seal().violation is violation


@pytest.mark.parametrize("stream", [A, T, V])
def test_capacity_never_silently_discards(stream: SelectionStream) -> None:
    target = kernel(capacity_per_stream=2)
    offer(target, stream, 10)
    if stream == A:  # Pre-start ATTITUDE was discarded; other streams keep it.
        offer(target, stream, 11)
    assert not target.offer(stream, s(12))
    assert target.violation.reason == "capacity_exceeded"


class IntSubclass(int):
    pass


@pytest.mark.parametrize("stamp", [None, True, False, -1, 1.0, "1", IntSubclass(1)])
def test_bad_source_stamp_invalidates(stamp: object) -> None:
    target = kernel()
    assert not target.offer(T, s(stamp))
    assert target.violation.reason == "invalid_sample"


@pytest.mark.parametrize("epoch", [True, None, -1, 3.0, IntSubclass(3), 2, 4])
def test_epoch_mismatch_never_infers_a_reboot(epoch: object) -> None:
    target = kernel()
    assert not target.offer(T, s(1, epoch=epoch))
    expected = "wrong_epoch" if type(epoch) is int and epoch >= 0 else "invalid_sample"
    assert target.violation.reason == expected
    replacement = kernel(boot_epoch=4)
    assert replacement.offer(T, s(1, epoch=4))


class BytesSubclass(bytes):
    pass


@pytest.mark.parametrize("payload", [None, bytearray(b"x"), memoryview(b"x"),
                                     "x", BytesSubclass(b"x")])
def test_payload_requires_exact_immutable_bytes(payload: object) -> None:
    target = kernel()
    assert not target.offer(T, s(1, payload))
    assert target.violation.reason == "invalid_sample"


@pytest.mark.parametrize("stream", ["attitude", None, {}])
def test_invalid_stream_latches_instead_of_raising(stream: object) -> None:
    target = kernel()
    assert not target.offer(stream, s(1))
    assert target.violation.reason == "invalid_stream"
    assert not target.seal().valid


@pytest.mark.parametrize("field", ["boot_epoch", "period_us", "start_slot",
                                   "max_truth_gap_us", "capacity_per_stream", "end_slot_exclusive"])
@pytest.mark.parametrize("value", [True, 1.0, -1, None, IntSubclass(1)])
def test_config_rejects_inexact_integers(field: str, value: object) -> None:
    with pytest.raises(ValueError):
        kernel(**{field: value})


@pytest.mark.parametrize("field,value", [("period_us", 0), ("start_slot", 0),
    ("max_truth_gap_us", 0), ("capacity_per_stream", 0), ("capacity_per_stream", 1)])
def test_config_lower_bounds(field: str, value: int) -> None:
    with pytest.raises(ValueError):
        kernel(**{field: value})


def test_large_integer_slot_key_and_immutable_result() -> None:
    index = 2**54 + 1
    target = kernel(start_slot=index, end_slot_exclusive=index + 1)
    offer(target, A, index * 10 + 7, (index + 1) * 10)
    offer(target, T, index * 10 + 7)
    offer(target, V, index * 10 + 7)
    decision = target.poll()
    assert decision.slot == SlotKey(3, index)
    with pytest.raises(FrozenInstanceError):
        decision.attitude = None


def test_seal_finalizes_configured_window_and_rejects_later_operations() -> None:
    target = kernel(end_slot_exclusive=2)
    offer(target, A, 20)
    assert target.poll().outcome == "empty"
    verdict = target.seal()
    assert verdict.valid
    assert verdict.end_slot_exclusive == 2
    assert target.seal() is verdict
    with pytest.raises(RuntimeError):
        target.offer(A, s(30))
    with pytest.raises(RuntimeError):
        target.poll()


def test_seal_cannot_claim_unconsumed_decisions() -> None:
    target = kernel(end_slot_exclusive=3)
    offer(target, A, 20)
    target.poll()
    verdict = target.seal()
    assert not verdict.valid
    assert verdict.violation.reason == "incomplete_window"


@pytest.mark.parametrize("end", [None, True, 1.0, 0, 1, IntSubclass(2)])
def test_configuration_rejects_invalid_window_end(end: object) -> None:
    with pytest.raises(ValueError):
        kernel(end_slot_exclusive=end)


def test_poll_never_checks_faulty_association_after_fixed_window_end() -> None:
    for greedy in (False, True):
        target = kernel(end_slot_exclusive=2)
        offer(target, A, 10, 27, 30)
        offer(target, T, 10, 100)
        offer(target, V, 100)
        assert target.poll().truth == (s(10), s(10))
        if greedy:
            assert target.poll() is None  # Slot 2 would have a 90-us truth gap.
            assert target.poll() is None
        assert target.seal().valid


def test_post_window_fault_fixture_really_fails_when_that_slot_is_included() -> None:
    target = kernel(end_slot_exclusive=3)
    offer(target, A, 10, 27, 30)
    offer(target, T, 10, 100)
    offer(target, V, 100)
    assert target.poll().truth == (s(10), s(10))
    assert target.poll() is None
    assert target.violation.reason == "truth_gap"
    assert target.violation.slot == SlotKey(3, 2)
    assert not target.seal().valid
