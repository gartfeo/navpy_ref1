"""Reader lag: a NavPy's readers fall behind while it runs (``deterministic_dev_sim.md`` step 9)."""

from __future__ import annotations

import random

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.baton_sync import BatonEvent, sleep
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.mavlink_wire import encoder
from navpy.sim.lockstep.participant_budget import PARTICIPANT_MODELS, participant_models, reader_lag
from navpy.sim.lockstep.reader_lag import READER_LAG, LagEpisode, ReaderLag
from navpy.sim.lockstep.sim_profile import CATALOGUE, model_stream, parse_profile, realize
from tests.modules.lockstep.test_lockstep_bus import SERIAL0, _lockstep

TICK_US = 20_000
FIXED = {"min_peak_ticks": 50, "max_peak_ticks": 50, "min_rise_s": 4.0, "max_rise_s": 4.0,
         "min_drain_s": 6.0, "max_drain_s": 6.0, "min_gap_s": 30.0, "max_gap_s": 30.0}


def test_an_episodes_delay_rises_to_its_peak_then_drains_to_zero():
    episode = LagEpisode(start_us=10_000_000, peak_us=1_000_000, rise_us=4_000_000, drain_us=6_000_000)
    at = {t_s: episode.delay_us(round(t_s * 1e6)) for t_s in (9.0, 10.0, 12.0, 14.0, 17.0, 20.0, 21.0)}
    assert at == {9.0: 0, 10.0: 0, 12.0: 500_000, 14.0: 1_000_000, 17.0: 500_000, 20.0: 0, 21.0: 0}
    # Rising up to the peak, never past it.
    assert episode.delay_us(13_999_000) == 999_750
    assert max(episode.delay_us(t) for t in range(10_000_000, 20_000_000, 500)) == 1_000_000


def test_episodes_are_due_a_gap_after_the_last_due_time_and_never_overlap():
    lag = ReaderLag(FIXED, random.Random(1))
    assert [lag.delay_us(round(t_s * 1e6)) for t_s in (29.0, 32.0, 34.0, 37.0, 62.0, 64.0)] == [
        0, 500_000, 1_000_000, 500_000, 500_000, 1_000_000]
    # A gap shorter than an episode: the next one starts when this one ends (here 2 s after its due time).
    short = ReaderLag({**FIXED, "min_gap_s": 8.0, "max_gap_s": 8.0}, random.Random(1))
    assert short.delay_us(18_000_000) == 0 and short.delay_us(20_000_000) == 500_000


def test_the_schedule_is_one_function_of_sim_time_whatever_order_it_is_asked_in():
    params = {**FIXED, "min_peak_ticks": 20, "min_rise_s": 1.0, "min_drain_s": 2.0, "min_gap_s": 5.0}
    times = [round(t_s * 1e6) for t_s in range(0, 400, 3)]
    forward = ReaderLag(params, model_stream(7, READER_LAG, "participant.1.1"))
    backward = ReaderLag(params, model_stream(7, READER_LAG, "participant.1.1"))
    late_first = [backward.delay_us(t) for t in reversed(times)][::-1]
    assert [forward.delay_us(t) for t in times] == late_first
    assert any(late_first) and forward.due_us(times[40]) == times[40] + late_first[40]


def test_the_stream_draws_a_gap_a_peak_a_rise_and_a_drain_per_episode():
    params = {**FIXED, "min_peak_ticks": 10, "max_peak_ticks": 60, "min_rise_s": 1.0, "min_drain_s": 2.0,
              "min_gap_s": 5.0}
    stream = model_stream(3, READER_LAG, "participant.2.1")
    gap = 5.0 + 25.0 * stream.random()
    peak = 10 + int(stream.random() * 51)
    rise = 1.0 + 3.0 * stream.random()
    lag = reader_lag({READER_LAG: params}, 3, 2, 1)
    start_us = round(1e6 * gap)
    assert lag.delay_us(start_us + round(1e6 * rise)) == peak * TICK_US
    assert lag.delay_us(start_us) == 0 and lag.delay_us(start_us + TICK_US) > 0


def test_peaks_reach_both_ends_of_their_range():
    lag = ReaderLag({**FIXED, "min_peak_ticks": 1, "max_peak_ticks": 2}, random.Random(5))
    lag.delay_us(2_000_000_000)
    assert {episode.peak_us for episode in lag._episodes} == {TICK_US, 2 * TICK_US}


def test_a_restarted_navpy_draws_a_schedule_of_its_own():
    params = {**FIXED, "min_gap_s": 5.0}
    first, restarted = reader_lag({READER_LAG: params}, 1, 1, 1), reader_lag({READER_LAG: params}, 1, 1, 2)
    times = range(0, 300_000_000, 1_000_000)
    assert [first.delay_us(t) for t in times] != [restarted.delay_us(t) for t in times]


@pytest.mark.parametrize("change, message", [
    ({"min_peak_ticks": 2.0}, "min_peak_ticks must be an int"),
    ({"min_peak_ticks": 0}, "min_peak_ticks must be a finite number above 0"),
    ({"min_rise_s": 5.0}, "min_rise_s 5.0 is above max_rise_s 4.0"),
    ({"max_gap_s": float("inf")}, "max_gap_s must be a finite number above 0"),
    ({"min_drain_s": True}, "min_drain_s must be a finite number above 0"),
])
def test_a_bad_reader_lag_is_refused(change, message):
    with pytest.raises(ValueError, match=message):
        ReaderLag({**FIXED, **change}, random.Random(1))


def test_reader_lag_is_a_navpy_model_and_off_by_default():
    assert READER_LAG in PARTICIPANT_MODELS and READER_LAG in CATALOGUE
    on = parse_profile("t", {"models": {READER_LAG: {"p_on": 1.0, "params": FIXED}}}, CATALOGUE)
    assert participant_models(realize(on, 1)) == {READER_LAG: FIXED}
    assert reader_lag({}, 1, 1, 1) is None


class _DueAfter:
    """A stub lag: packets delivered from ``from_tick`` on wait ``ticks`` ticks."""

    def __init__(self, from_tick: int, ticks: int) -> None:
        self._from_us, self._delay_us = from_tick * TICK_US, ticks * TICK_US

    def due_us(self, delivered_us: int) -> int:
        return delivered_us + (self._delay_us if delivered_us >= self._from_us else 0)


def _heartbeat() -> bytes:
    link, sink = encoder(1, 1)
    link.heartbeat_send(mavlink.MAV_TYPE_FIXED_WING, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
    return sink.drain()


def test_a_lagged_reader_dispatches_each_packet_at_its_due_tick_in_order_while_others_run():
    with _lockstep(None) as (baton, _rng):
        baton.runnable.scheduling.reader_lag = _DueAfter(from_tick=3, ticks=4)
        link = LockstepLink(baton, SERIAL0)
        got: list[int] = []
        ran: list[int] = []
        done = BatonEvent(baton)

        class Receiver:
            def feed_message(self, _message) -> None:
                got.append(baton.now_us // TICK_US)
                if len(got) == 6:
                    done.set()

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(Receiver())
            done.wait()
            link.close()

        def other() -> None:
            while not done.is_set():
                ran.append(baton.now_us // TICK_US)
                sleep(baton, TICK_US / 2e6)  # due mid-tick: wakes on the next tick

        baton.spawn(consumer, name="consumer")
        baton.spawn(other, name="other")
        baton.run_until_quiescent()
        for k in range(1, 13):
            baton.tick(k * TICK_US, (lambda: link.deliver(_heartbeat())) if k <= 6 else None)
        assert not baton.threads.alive()
    # Ticks 1, 2 pass at once; from tick 3 each waits 4 ticks, and none passes an earlier one.
    assert got == [1, 2, 7, 8, 9, 10]
    assert ran[:10] == list(range(0, 10))  # the NavPy's other threads kept running every tick


class _DueAt:
    """A stub lag: every packet is due at ``tick`` at the earliest."""

    def __init__(self, tick: int) -> None:
        self._due_us = tick * TICK_US

    def due_us(self, delivered_us: int) -> int:
        return max(delivered_us, self._due_us)


@pytest.mark.parametrize("lagged, woken_at", [(True, [4]), (False, [1, 2, 3, 4])])
def test_a_lagged_reader_drains_the_packets_due_together_in_one_turn(lagged, woken_at):
    """As the wall reader does: a consumer the first packet wakes runs once the reader blocks, not between
    packets. Without the lag the lockstep reader still yields after each packet (``none``, ``hw``)."""
    with _lockstep(None) as (baton, _rng):
        if lagged:
            baton.runnable.scheduling.reader_lag = _DueAt(tick=6)
        link = LockstepLink(baton, SERIAL0)
        got: list[int] = []
        seen: list[int] = []
        woke = BatonEvent(baton)

        class Receiver:
            def feed_message(self, _message) -> None:
                got.append(baton.now_us // TICK_US)
                woke.set()

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(Receiver())
            while len(got) < 4:
                woke.wait()
                woke.clear()
                seen.append(len(got))
            link.close()

        baton.spawn(consumer, name="consumer")
        baton.run_until_quiescent()
        baton.tick(TICK_US, lambda: link.deliver(_heartbeat() * 4))
        for k in range(2, 10):
            baton.tick(k * TICK_US)
        assert not baton.threads.alive()
    assert got == ([6] * 4 if lagged else [1] * 4) and seen == woken_at
