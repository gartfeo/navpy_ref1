"""W4 compares complete decisions, including source-defined closing samples."""
from collections.abc import Iterator
import random

from navpy.modules.vision.sim.deterministic_selection_types import (
    ClosingSamples, SelectionConfig, SelectionDecision, SelectionStream, SlotKey,
    SourceSample,
)
from navpy.modules.vision.sim.deterministic_selector import DeterministicSelector

STREAMS = tuple(SelectionStream)
PACKETS = (
    ((11, b"a11"), (17, b"a17"), (30, b"a30")),
    ((10, b"t10"), (20, b"t20"), (30, b"t30")),
    ((10, b"v10"), (15, b"v15"), (20, b"v20")),
)


def interleavings(remaining: tuple[int, ...] = (3, 3, 3)) -> Iterator[tuple[int, ...]]:
    if not any(remaining):
        yield ()
    for stream, count in enumerate(remaining):
        if count:
            rest = tuple(value - (index == stream)
                         for index, value in enumerate(remaining))
            for suffix in interleavings(rest):
                yield (stream, *suffix)


def drain(target: DeterministicSelector, answers: list[SelectionDecision]) -> None:
    while (decision := target.poll()) is not None:
        answers.append(decision)


def primed(capacity: int = 8, end: int = 3) -> DeterministicSelector:
    target = DeterministicSelector(SelectionConfig(3, 10, 1, 10, capacity, end))
    for stream in STREAMS:
        assert target.offer(stream, SourceSample(3, 0, b"prestart"))
    return target


def test_all_cross_stream_interleavings_and_poll_schedules_have_literal_choices() -> None:
    def s(t: int, b: bytes) -> SourceSample:
        return SourceSample(3, t, b)
    expected = [
        SelectionDecision(SlotKey(3, 1), "fresh", s(17, b"a17"),
            (s(11, b"a11"),), (s(10, b"t10"), s(20, b"t20")), s(15, b"v15"),
            ClosingSamples(s(30, b"a30"), s(20, b"t20"), s(20, b"v20"))),
        SelectionDecision(SlotKey(3, 2), "empty", None, (), None, None,
            ClosingSamples(s(30, b"a30"), None, None)),
    ]
    count = 0
    for schedule in interleavings():
        count += 1
        for poll_every in (1, 2, 9):
            target = primed()
            cursors, answers = [0, 0, 0], []
            for step, stream in enumerate(schedule, 1):
                stamp, payload = PACKETS[stream][cursors[stream]]
                cursors[stream] += 1
                assert target.offer(STREAMS[stream], s(stamp, payload))
                if step % poll_every == 0:
                    drain(target, answers)
            drain(target, answers)
            assert answers == expected, (schedule, poll_every)
            assert target.seal().valid, (schedule, poll_every)
    assert count == 1680


def batch_oracle(streams: tuple[tuple[SourceSample, ...], ...],
                 end: int) -> list[SelectionDecision]:
    # Independent full-history reference: never prunes or advances a watermark.
    attitude, truth, speed = [sorted(items, key=lambda item: item.time_us) for items in streams]
    expected = []
    for slot in range(1, end):
        candidates = [item for item in attitude if slot * 10 <= item.time_us < (slot + 1) * 10]
        closing = next(item for item in attitude if item.time_us >= (slot + 1) * 10)
        if not candidates:
            expected.append(SelectionDecision(SlotKey(3, slot), "empty", None, (), None,
                                              None, ClosingSamples(closing, None, None)))
            continue
        selected = candidates[-1]
        lower = max((item for item in truth if item.time_us <= selected.time_us), key=lambda item: item.time_us)
        upper = next(item for item in truth if item.time_us >= selected.time_us)
        air = max((item for item in speed if item.time_us <= selected.time_us), key=lambda item: item.time_us)
        air_close = next(item for item in speed if item.time_us >= selected.time_us)
        expected.append(SelectionDecision(SlotKey(3, slot), "fresh", selected,
            tuple(candidates[:-1]), (lower, upper), air, ClosingSamples(closing, upper, air_close)))
    return expected


def test_retention_matches_full_history_oracle_with_empty_slots_and_delayed_airspeed() -> None:
    streams = tuple(tuple(SourceSample(3, t, f"{index}:{t}".encode()) for t in stamps)
        for index, stamps in enumerate((
            [0, *[t for t in range(10, 401, 5) if t // 10 % 5 != 2]],
            [0, *range(8, 419, 10)], range(0, 451, 50),
        )))
    expected = batch_oracle(streams, 40)
    for seed in range(20):
        rng = random.Random(seed)
        target = DeterministicSelector(SelectionConfig(3, 10, 1, 10, 128, 40))
        cursors, answers = [0, 0, 0], []
        while available := [i for i in range(3) if cursors[i] < len(streams[i])]:
            index = rng.choice(available)
            assert target.offer(STREAMS[index], streams[index][cursors[index]])
            cursors[index] += 1
            if rng.randrange(3) == 0:
                drain(target, answers)
        drain(target, answers)
        assert answers == expected
        assert target.seal().valid


def test_late_fault_after_last_decision_remains_in_final_verdict() -> None:
    target = primed()
    for stream, packets in zip(STREAMS, PACKETS):
        for stamp, payload in packets:
            assert target.offer(stream, SourceSample(3, stamp, payload))
    answers = []
    drain(target, answers)
    assert len(answers) == 2
    assert not target.offer(SelectionStream.ATTITUDE, SourceSample(3, 18, b"late"))
    assert not target.seal().valid
    assert target.violation.reason == "source_regressed"


def test_overshoot_does_not_change_finalized_prefix_verdict() -> None:
    verdicts = []
    for greedy in (False, True):
        target = primed(end=2)
        assert target.offer(SelectionStream.ATTITUDE, SourceSample(3, 30, b"later"))
        assert target.poll().slot == SlotKey(3, 1)
        if greedy:
            assert target.poll() is None
        verdicts.append(target.seal())
    assert verdicts[0] == verdicts[1]
    assert verdicts[0].valid


def test_overload_is_invalidity_not_a_silent_hold() -> None:
    for slow_poll, stalled_speed in ((False, False), (True, False), (False, True)):
        target = DeterministicSelector(SelectionConfig(3, 10, 1, 10, 3, 9))
        answers = []
        for stamp in range(0, 100, 10):
            for stream in STREAMS:
                if stalled_speed and stream == SelectionStream.AIRSPEED and stamp:
                    continue
                target.offer(stream, SourceSample(3, stamp, b"value"))
            if not slow_poll:
                drain(target, answers)
        if slow_poll or stalled_speed:
            assert target.violation.reason == "capacity_exceeded"
            assert not target.seal().valid
        else:
            assert target.seal().valid
