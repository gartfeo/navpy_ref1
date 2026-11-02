"""Seeded thread order, reader budget and timers first (``deterministic_dev_sim.md`` step 6)."""

from __future__ import annotations

from collections import Counter
from queue import Empty

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import RESPONSE_HISTORY_DEPTH
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_parts import ReaderBudget, Scheduling
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import BatonEvent, sleep, yield_now
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.mavlink_wire import encoder
from navpy.sim.lockstep.participant_budget import PARTICIPANT_MODELS, participant_models
from navpy.sim.lockstep.scheduling_models import (
    READER_BUDGET,
    SCHEDULING_MODELS,
    THREAD_ORDER,
    TIMERS_FIRST,
    apply_scheduling,
    check_scheduling,
    scheduling_subset,
)
from navpy.sim.lockstep.sim_profile import CATALOGUE, load_profile, model_stream, parse_profile, realize
from tests.modules.lockstep.lockstep_autopilot import ReferenceAutopilot
from tests.modules.lockstep.test_lockstep_bus import SERIAL0, _Harness, _lines, _lockstep, _navpy_vehicle

TICK_US = 20_000
WORKERS = 6


def _first_tick_order(models: dict, seed: int, ordinal: int = 1, generation: int = 1) -> list[str]:
    """The order in which ``WORKERS`` threads, runnable at one tick, each run twice."""
    runtime = BatonRuntime()
    apply_scheduling(runtime, models, seed, ordinal, generation)
    order: list[str] = []

    def worker(name: str) -> None:
        order.append(name)
        yield_now(runtime)
        order.append(name)

    for index in range(WORKERS):
        runtime.spawn(lambda name=f"w{index}": worker(name), name=f"w{index}")
    runtime.tick(0)
    return order


def test_without_thread_order_runnable_threads_run_first_in_first_out():
    names = [f"w{index}" for index in range(WORKERS)]
    assert _first_tick_order({}, 1) == names + names


def test_thread_order_differs_by_seed_and_repeats_for_one_seed():
    models = {THREAD_ORDER: {}}
    one = _first_tick_order(models, 1)
    assert one == _first_tick_order(models, 1)
    assert sorted(one) == sorted(_first_tick_order({}, 1))  # every run happens, only the order moves
    others = [_first_tick_order(models, seed) for seed in range(2, 12)]
    assert all(order != one for order in others)
    # Its stream is the participant's own: another ordinal or generation orders differently.
    assert _first_tick_order(models, 1, ordinal=2) != one
    assert _first_tick_order(models, 1, generation=2) != one


def test_thread_order_picks_every_runnable_thread_first_about_equally():
    firsts = Counter(_first_tick_order({THREAD_ORDER: {}}, seed)[0] for seed in range(600))
    assert set(firsts) == {f"w{index}" for index in range(WORKERS)}
    assert all(70 < count < 130 for count in firsts.values()), firsts  # 100 each when uniform


def test_the_thread_order_stream_is_the_participants_model_stream():
    stream = model_stream(5, THREAD_ORDER, "participant.3.2")
    runnable, ran, expected = [f"w{index}" for index in range(WORKERS)], set(), []
    while runnable:  # one draw per pick among two or more; a yield goes to the back
        index = min(len(runnable) - 1, int(stream.random() * len(runnable))) if len(runnable) > 1 else 0
        name = runnable.pop(index)
        expected.append(name)
        if name not in ran:
            ran.add(name)
            runnable.append(name)
    assert _first_tick_order({THREAD_ORDER: {}}, 5, ordinal=3, generation=2) == expected


def _reply_on_its_timeout_tick(models: dict) -> list[object]:
    """A waiter's reply is delivered at the tick its 40 ms timeout is due (tick 2)."""
    runtime = BatonRuntime()
    apply_scheduling(runtime, models, 1, 1, 1)
    inbox = BatonQueue(runtime)
    outcome: list[object] = []

    def waiter() -> None:
        try:
            outcome.append(inbox.get(timeout=0.04))
        except Empty:
            outcome.append("timeout")

    runtime.spawn(waiter, name="waiter")
    runtime.run_until_quiescent()
    runtime.tick(TICK_US)
    runtime.tick(2 * TICK_US, lambda: inbox.deliver("reply"))
    return [*outcome, inbox.qsize()]


def test_a_reply_on_its_timeout_tick_wins_by_default_and_loses_with_timers_first():
    assert _reply_on_its_timeout_tick({}) == ["reply", 0]
    # The timeout fires before the tick's inputs: the waiter gives up, the reply stays queued.
    assert _reply_on_its_timeout_tick({TIMERS_FIRST: {}}) == ["timeout", 1]


def test_timers_first_runs_a_due_timer_before_the_inputs_and_shares_the_handoff_cap():
    runtime = BatonRuntime()
    apply_scheduling(runtime, {TIMERS_FIRST: {}}, 1, 1, 1)
    inbox = BatonQueue(runtime)
    trace: list[str] = []

    def sleeper() -> None:
        sleep(runtime, 0.02)
        trace.append("timer")

    def reader() -> None:
        trace.append(f"input {inbox.get()}")

    runtime.spawn(sleeper, name="sleeper")
    runtime.spawn(reader, name="reader")
    runtime.run_until_quiescent()
    # A cap of one handoff: the timer takes it, the input waits for the next tick.
    assert runtime.tick(TICK_US, lambda: inbox.deliver("x"), handoffs=1) == 1
    assert trace == ["timer"]
    runtime.tick(2 * TICK_US)
    assert trace == ["timer", "input x"]


def test_with_timers_first_a_timed_out_waiter_the_cap_leaves_runnable_takes_a_reply_first():
    """As CPython's ``Queue.get``: a waiter whose timeout fired re-checks the queue when it runs,
    so a reply that comes before it gets the CPU wins, as on a starved wall thread."""
    runtime = BatonRuntime()
    apply_scheduling(runtime, {TIMERS_FIRST: {}}, 1, 1, 1)
    inbox = BatonQueue(runtime)
    trace: list[str] = []

    def sleeper() -> None:
        sleep(runtime, 0.02)
        trace.append("timer")

    def waiter() -> None:
        try:
            trace.append(inbox.get(timeout=0.02))
        except Empty:
            trace.append("timeout")

    runtime.spawn(sleeper, name="sleeper")
    runtime.spawn(waiter, name="waiter")
    runtime.run_until_quiescent()
    # Both timers fire first; the cap of one handoff runs only the sleeper before the reply.
    runtime.tick(TICK_US, lambda: inbox.deliver("reply"), handoffs=1)
    assert trace == ["timer"]
    runtime.tick(2 * TICK_US)
    assert trace == ["timer", "reply"]


def _param_values(count: int) -> bytes:
    link, sink = encoder(1, 1)
    for index in range(count):
        link.param_value_send(f"P{index:04d}".encode(), float(index), mavlink.MAV_PARAM_TYPE_REAL32, count, index)
    return sink.drain()


class _Receiver:
    def __init__(self, baton: BatonRuntime) -> None:
        self.baton = baton
        self.received: list[tuple[int, int]] = []
        self.event = BatonEvent(baton)

    def feed_message(self, message) -> None:
        self.received.append((self.baton.now_us // TICK_US, message.param_index))
        self.event.set()


def _served(budget: ReaderBudget | None, packets: int, ticks: int) -> tuple[list[tuple[int, int]], list[int]]:
    """One tick's burst of ``packets`` through a lockstep bus: each packet's (tick, index), and
    how many packets a consumer woken by each one had seen when it ran."""
    with _lockstep(None) as (baton, _rng):
        baton.runnable.scheduling.reader_budget = budget
        link = LockstepLink(baton, SERIAL0)
        receiver = _Receiver(baton)
        seen: list[int] = []

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(receiver)
            while len(receiver.received) < packets:
                receiver.event.wait()
                receiver.event.clear()
                seen.append(len(receiver.received))
            link.close()

        baton.spawn(consumer, name="consumer")
        baton.run_until_quiescent()
        baton.tick(TICK_US, lambda: link.deliver(_param_values(packets)))
        for k in range(2, ticks + 1):
            baton.tick(k * TICK_US)
        assert not baton.threads.alive()
    return receiver.received, seen


def test_the_reader_yields_after_each_packet_without_a_budget():
    received, seen = _served(None, 30, 2)
    assert received == [(1, index) for index in range(30)]
    assert seen == list(range(1, 31))


def test_a_reader_budget_drains_up_to_its_packets_per_tick_and_carries_the_rest_over():
    received, seen = _served(ReaderBudget(packets_per_tick=12), 30, 4)
    assert [index for _tick, index in received] == list(range(30))  # byte order kept
    assert Counter(tick for tick, _index in received) == {1: 12, 2: 12, 3: 6}
    # A woken consumer runs only where the reader blocks: at the budget, or on drained input.
    assert seen == [12, 24, 30]
    received, seen = _served(ReaderBudget(packets_per_tick=4), 10, 4)
    assert Counter(tick for tick, _index in received) == {1: 4, 2: 4, 3: 2}
    assert seen == [4, 8, 10]
    # A backlog several times the budget carries over the same way.
    received, _seen = _served(ReaderBudget(packets_per_tick=4), 18, 6)
    assert [index for _tick, index in received] == list(range(18))
    assert Counter(tick for tick, _index in received) == {1: 4, 2: 4, 3: 4, 4: 4, 5: 2}


def _capped_delivery(budget: ReaderBudget | None, packets: int) -> tuple[int, int, list[str]]:
    """One capped tick (4 handoffs) with ``packets`` for an idle reader and a timer due in it:
    the handoffs taken, the packets dispatched, and whether the timer's thread ran."""
    with _lockstep(None) as (baton, _rng):
        baton.runnable.scheduling.reader_budget = budget
        link = LockstepLink(baton, SERIAL0)
        receiver = _Receiver(baton)
        ran: list[str] = []

        def timer() -> None:
            sleep(baton, TICK_US / 1e6)
            ran.append("timer")

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(receiver)
            sleep(baton, 0.1)
            link.close()

        baton.spawn(timer, name="timer")
        baton.spawn(consumer, name="consumer")
        baton.run_until_quiescent()
        taken = baton.tick(TICK_US, lambda: link.deliver(_param_values(packets)), handoffs=4)
        dispatched, ran_in_tick = len(receiver.received), list(ran)
        for k in range(2, 40):
            baton.tick(k * TICK_US)
        assert not baton.threads.alive()
    return taken, dispatched, ran_in_tick


@pytest.mark.parametrize("packets", [3, 16])
def test_a_drained_delivery_takes_one_reader_handoff_and_a_due_timer_runs_in_that_tick(packets):
    # 16 is exactly the budget: no extra yield after the last allowed packet.
    taken, dispatched, ran = _capped_delivery(ReaderBudget(packets_per_tick=16), packets)
    assert taken == 2 and dispatched == packets and ran == ["timer"]


def test_a_per_packet_reader_spends_the_cap_and_the_due_timer_waits():
    taken, dispatched, ran = _capped_delivery(None, 16)
    assert taken == 4 and dispatched == 4 and ran == []  # todo 65's starvation, with no reader budget


def test_a_close_wakes_a_reader_waiting_out_its_budget_in_the_same_tick():
    with _lockstep(None) as (baton, _rng):
        baton.runnable.scheduling.reader_budget = ReaderBudget(packets_per_tick=2)
        link = LockstepLink(baton, SERIAL0)
        receiver = _Receiver(baton)

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(receiver)
            receiver.event.wait()
            link.close()  # the reader waits for the next tick, with 3 packets left

        baton.spawn(consumer, name="consumer")
        baton.run_until_quiescent()
        baton.tick(TICK_US, lambda: link.deliver(_param_values(5)))
        assert not baton.threads.alive()
        assert receiver.received == [(1, 0), (1, 1)]


def _through_reader(
    models: dict, plan: list[tuple[int, int | None]], *, seed: int = 1, busy: int = 0, sleepers: int = 0
) -> list[int]:
    """Each packet's tick, in dispatch order, after ``plan``: per tick, the packets delivered and
    the handoff cap. ``busy`` threads yield through the plan and ``sleepers`` re-arm a one-tick
    timer; then uncapped ticks until all end.
    Checks that no packet is lost, byte order holds, nothing hangs and a second run is equal."""
    ticks = _one_pass_through_reader(models, plan, seed, busy, sleepers)
    assert _one_pass_through_reader(models, plan, seed, busy, sleepers) == ticks
    return ticks


def _one_pass_through_reader(
    models: dict, plan: list[tuple[int, int | None]], seed: int, busy: int, sleepers: int
) -> list[int]:
    total = sum(packets for packets, _cap in plan)
    with _lockstep(None) as (baton, _rng):
        apply_scheduling(baton, models, seed, 1, 1)
        link = LockstepLink(baton, SERIAL0)
        receiver = _Receiver(baton)
        planned = [True]

        def consumer() -> None:
            link.open(SERIAL0).reserve(1).attach(receiver)
            while len(receiver.received) < total:
                sleep(baton, TICK_US / 1e6)
            link.close()

        def spinner() -> None:
            while planned[0]:
                yield_now(baton)

        def sleeper() -> None:
            while planned[0]:
                sleep(baton, TICK_US / 1e6)

        baton.spawn(consumer, name="consumer")
        baton.run_until_quiescent()
        for index in range(busy):
            baton.spawn(spinner, name=f"busy{index}")
        for index in range(sleepers):
            baton.spawn(sleeper, name=f"sleeper{index}")
        tick = 0
        for packets, cap in plan:
            tick += 1
            deliver = (lambda count=packets: link.deliver(_param_values(count))) if packets else None
            baton.tick(tick * TICK_US, deliver, handoffs=cap)
        planned[0] = False
        for tick in range(tick + 1, tick + 200):
            if not baton.threads.alive():
                break
            baton.tick(tick * TICK_US)
        assert not baton.threads.alive()
    expected: list[int] = []
    for packets, _cap in plan:
        expected.extend(range(packets))
    assert [index for _tick, index in receiver.received] == expected
    return [tick for tick, _index in receiver.received]


@pytest.mark.parametrize(("budget", "ticks"), [(4, {1: 4, 2: 4, 3: 2}), (16, {1: 10})])
def test_a_reader_budget_with_timers_first_keeps_every_packet_in_order(budget, ticks):
    models = {READER_BUDGET: {"packets_per_tick": budget}, TIMERS_FIRST: {}}
    assert Counter(_through_reader(models, [(10, None)])) == ticks


def test_timers_first_fires_a_reader_past_its_budget_before_the_other_threads_use_up_the_cap():
    models = {READER_BUDGET: {"packets_per_tick": 4}, TIMERS_FIRST: {}}
    plan = [(10, 4)] + [(0, 4)] * 6
    assert Counter(_through_reader(models, plan, busy=3)) == {2: 4, 4: 4, 6: 2}


def test_a_reader_budget_with_seeded_order_keeps_every_packet_in_order_and_one_result_per_seed():
    models = {READER_BUDGET: {"packets_per_tick": 4}, THREAD_ORDER: {}}
    plan = [(10, 5)] + [(0, 5)] * 7
    schedules = [_through_reader(models, plan, seed=seed, busy=3) for seed in range(1, 7)]
    assert len({tuple(schedule) for schedule in schedules}) > 1  # the seed moves when packets are served


def test_a_reader_past_its_budget_gets_nothing_while_other_threads_use_up_the_cap():
    """Todo 200, today's behaviour: the reader waits on a timer, a delivery does not wake it, and
    timers fire only after the input phase, so a used-up cap keeps it parked. Its packets survive."""
    plan = [(10, 4)] + [(1, 4)] * 6
    ticks = _through_reader({READER_BUDGET: {"packets_per_tick": 4}}, plan, busy=3)
    assert Counter(tick for tick in ticks if tick <= len(plan)) == {1: 4}


def test_a_reader_past_its_budget_competes_for_picks_once_its_timer_fires():
    """Todo 200, today's behaviour: threads that re-arm a timer leave handoffs after the input
    phase, so the reader's timer fires, but the cap then serves it only on some ticks: every other
    tick with 6 such threads under a cap of 3, every tick with 2. Tick 1 goes to the 6 new threads."""
    plan = [(64, 3)] + [(0, 3)] * 15
    budget = {READER_BUDGET: {"packets_per_tick": 4}}
    ticks = _through_reader(budget, plan, sleepers=6)
    assert sorted({tick for tick in ticks if tick <= len(plan)}) == [3, 5, 7, 9, 11, 13, 15]
    ticks = _through_reader(budget, plan, sleepers=2)
    assert sorted({tick for tick in ticks if tick <= len(plan)}) == list(range(1, 17))


def test_a_reader_below_its_budget_is_woken_by_a_delivery_under_the_same_cap():
    plan = [(4, 4), (3, 4)] + [(0, 4)] * 5
    ticks = _through_reader({READER_BUDGET: {"packets_per_tick": 16}}, plan, busy=3)
    assert ticks == [1] * 4 + [2] * 3


def test_a_freeze_holds_a_reader_past_its_budget_and_resume_keeps_the_order_with_a_fresh_budget():
    plan = [(10, None), (3, 0), (0, 0)]  # 4 served, then a 2-tick freeze with 3 more queued
    assert Counter(_through_reader({READER_BUDGET: {"packets_per_tick": 4}}, plan)) == {1: 4, 4: 4, 5: 4, 6: 1}


def _burst_read(budget: ReaderBudget | None) -> list[str]:
    with _lockstep(None) as (baton, rng):
        baton.runnable.scheduling.reader_budget = budget
        harness = _Harness(baton, ReferenceAutopilot(param_burst=RESPONSE_HISTORY_DEPTH))

        def navpy() -> None:
            vehicle = _navpy_vehicle(harness)
            try:
                harness.note("navpy", f"named read: {vehicle.get_parameter_fresh('SCHED_LOOP_RATE')}")
            finally:
                vehicle.close()

        harness.spawn("navpy", navpy)
        harness.run(100, rng)
        assert harness.alive() == []
    return _lines(harness, "navpy")


def test_a_burst_in_one_reader_batch_evicts_the_reply_and_the_read_fails_closed():
    """The wall race of ``reader_batch.md``: the burst behind the answer runs before the woken
    waiter, so the answer is evicted. The product raises; it never reads a wrong value."""
    assert _burst_read(None)[-1] == "named read: 50.0"
    whole = RESPONSE_HISTORY_DEPTH + 1  # the answer and the full burst in one turn
    lines = _burst_read(ReaderBudget(packets_per_tick=whole))
    assert lines[-1].startswith("raised MessageHistoryOverrun"), lines[-1]


def test_profiles_draw_the_scheduling_models_in_their_ranges():
    for name in SCHEDULING_MODELS:
        assert name in CATALOGUE and name in PARTICIPANT_MODELS
    hw = [realize(load_profile("hw"), seed) for seed in range(1, 401)]
    for realization in hw:
        assert not realization.model(READER_BUDGET)  # off in hw
    order_on = sum(r.model(THREAD_ORDER).on for r in hw) / len(hw)
    timers_on = sum(r.model(TIMERS_FIRST).on for r in hw) / len(hw)
    assert 0.42 < order_on < 0.58 and 0.18 < timers_on < 0.32
    draws = [realize(load_profile("stress"), seed).model(READER_BUDGET) for seed in range(1, 401)]
    stress = [draw for draw in draws if draw.on]
    assert len(stress) == len(draws)  # the CPU budget is on in every stress seed and needs it
    packets = [draw.params["packets_per_tick"] for draw in stress]
    # Calibrated from measured demand (section 2, "Reader budget calibration").
    assert min(packets) == 6 and max(packets) == 16
    assert load_profile("none").models == {}


def test_a_profile_whose_cpu_budget_is_never_on_needs_no_reader_budget():
    models = {"cpu_budget": {"p_on": 0.0, "params": {"handoffs": 4}}}
    assert parse_profile("x", {"models": models}, CATALOGUE).models["cpu_budget"].p_on == 0.0


@pytest.mark.parametrize(
    ("cap_on", "reader"),
    [
        (1.0, None),
        (0.5, None),
        (1.0, {"p_on": 0.5, "params": {"packets_per_tick": 4}}),
        (1.0, {"p_on": 0.75, "params": {"packets_per_tick": 4}}),
    ],
)
def test_a_profile_with_a_cpu_budget_needs_the_reader_budget_on(cap_on, reader):
    models = {"cpu_budget": {"p_on": cap_on, "params": {"handoffs": 4}}}
    if reader:
        models["reader_budget"] = reader
    with pytest.raises(ValueError, match="cpu_budget needs reader_budget"):
        parse_profile("x", {"models": models}, CATALOGUE)
    models["reader_budget"] = {"p_on": 1.0, "params": {"packets_per_tick": 4}}
    assert parse_profile("x", {"models": models}, CATALOGUE).models["reader_budget"].p_on == 1.0


def test_the_backend_carries_only_the_scheduling_models():
    realization = realize(load_profile("stress"), 24)  # draws the reader budget
    models = participant_models(realization)
    assert set(SCHEDULING_MODELS) - {TIMERS_FIRST} <= set(models)
    assert set(scheduling_subset(models)) == set(SCHEDULING_MODELS) & set(models)
    assert "cpu_budget" in models and "cpu_budget" not in scheduling_subset(models)


def test_apply_sets_only_the_on_models_and_refuses_a_bad_budget():
    baton = BatonRuntime()
    apply_scheduling(baton, {}, 1, 1, 1)
    assert baton.runnable.scheduling == Scheduling()
    apply_scheduling(baton, {READER_BUDGET: {"packets_per_tick": 20}}, 1, 1, 1)
    assert baton.runnable.scheduling == Scheduling(reader_budget=ReaderBudget(20))
    for bad in ({"packets_per_tick": 0}, {"packets_per_tick": 20.0}):
        with pytest.raises(ValueError):
            check_scheduling({READER_BUDGET: bad})


def test_the_participant_process_sets_its_role_configs_scheduling_on_its_baton():
    from types import SimpleNamespace

    from navpy.sim.lockstep.participant_budget import SIM_MODELS_KEY
    from navpy.sim.lockstep.participant_main import apply_sim_models

    models = {THREAD_ORDER: {}, TIMERS_FIRST: {}, READER_BUDGET: {"packets_per_tick": 30},
              "cpu_budget": {"handoffs": 5}}
    record = SimpleNamespace(config={SIM_MODELS_KEY: models}, seed=4, ordinal=2, generation=3)
    baton = BatonRuntime()
    camera, budget = apply_sim_models(record, SimpleNamespace(runtime=SimpleNamespace(baton=baton)))
    scheduling = baton.runnable.scheduling
    assert scheduling.timers_first and scheduling.reader_budget == ReaderBudget(30)
    expected = model_stream(4, THREAD_ORDER, "participant.2.3")
    assert [scheduling.order.random() for _ in range(3)] == [expected.random() for _ in range(3)]
    assert camera is None and budget.handoffs(TICK_US) == 5
