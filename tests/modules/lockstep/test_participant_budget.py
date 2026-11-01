"""Process freeze and CPU budget: the handoff cap per tick (``deterministic_dev_sim.md`` step 4)."""

from __future__ import annotations

import math

import pytest

from navpy.sim.lockstep.participant_budget import (
    CPU_BUDGET,
    PROCESS_FREEZE,
    SIM_MODELS_KEY,
    FreezeSchedule,
    TickBudget,
    participant_models,
)
from navpy.sim.lockstep.sim_profile import CATALOGUE, load_profile, model_stream, parse_profile, realize

TICK_US = 20_000
HW_FREEZE = {"min_ticks": 2, "max_ticks": 6, "min_gap_s": 5.0, "max_gap_s": 60.0}


def fixed(ticks: int, gap_s: float) -> dict:
    return {"min_ticks": ticks, "max_ticks": ticks, "min_gap_s": gap_s, "max_gap_s": gap_s}


def frozen_ticks(budget: TickBudget, ticks: int, first: int = 1) -> list[int]:
    """The ticks (of ``TICK_US``) whose cap is 0, asked once per tick in order."""
    return [k for k in range(first, first + ticks) if budget.handoffs(k * TICK_US) == 0]


def freezes(ticks: list[int]) -> list[tuple[int, int]]:
    """Runs of consecutive frozen ticks as (first tick, length)."""
    runs: list[tuple[int, int]] = []
    for k in ticks:
        if runs and runs[-1][0] + runs[-1][1] == k:
            runs[-1] = (runs[-1][0], runs[-1][1] + 1)
        else:
            runs.append((k, 1))
    return runs


def test_freezes_are_due_a_gap_after_the_last_due_time_and_last_their_ticks():
    # 1.01 s is not a whole number of ticks: a gap counted from the actual start tick would drift.
    budget = TickBudget({PROCESS_FREEZE: fixed(3, 1.01)}, seed=1, ordinal=1, generation=1)
    runs = freezes(frozen_ticks(budget, 260))
    # The first tick (20 ms) anchors it: due at 1.03 s, 2.04 s, 3.05 s, 4.06 s, 5.07 s.
    assert runs == [(52, 3), (102, 3), (153, 3), (203, 3), (254, 3)]


def test_the_cpu_budget_caps_every_tick_and_a_freeze_caps_it_at_zero():
    assert [TickBudget({}, 1, 1, 1).handoffs(k * TICK_US) for k in range(1, 5)] == [None] * 4
    cpu = TickBudget({CPU_BUDGET: {"handoffs": 7}}, 1, 1, 1)
    assert {cpu.handoffs(k * TICK_US) for k in range(1, 500)} == {7}
    both = TickBudget({CPU_BUDGET: {"handoffs": 7}, PROCESS_FREEZE: fixed(2, 0.1)}, 1, 1, 1)
    caps = [both.handoffs(k * TICK_US) for k in range(1, 13)]
    assert caps == [7, 7, 7, 7, 7, 0, 0, 7, 7, 7, 0, 0]  # due at 0.12 s and 0.22 s


def test_drawn_freezes_stay_in_the_hw_ranges_and_reach_both_length_ends():
    hour_ticks = 3600 * 1_000_000 // TICK_US
    runs = freezes(frozen_ticks(TickBudget({PROCESS_FREEZE: HW_FREEZE}, 1, 2, 1), hour_ticks))
    lengths = [length for _start, length in runs]
    assert set(lengths) == {2, 3, 4, 5, 6}
    gaps_s = [(b - a) * TICK_US / 1e6 for (a, _), (b, _) in zip(runs, runs[1:])]
    # Start to start, each start on the first tick at or after its due time.
    assert all(5.0 - 0.02 < gap < 60.0 + 0.02 for gap in gaps_s)
    assert 25.0 < sum(gaps_s) / len(gaps_s) < 40.0  # uniform over 5-60 s: mean 32.5 s
    assert min(gaps_s) < 10.0 and max(gaps_s) > 55.0


def test_each_participant_has_its_own_stream_keyed_by_ordinal_and_generation():
    params = {PROCESS_FREEZE: HW_FREEZE}
    one = frozen_ticks(TickBudget(params, 3, 1, 1), 30_000)
    assert one == frozen_ticks(TickBudget(params, 3, 1, 1), 30_000)
    for other in (TickBudget(params, 3, 2, 1), TickBudget(params, 3, 1, 2), TickBudget(params, 4, 1, 1)):
        assert frozen_ticks(other, 30_000) != one
    # The stream is the model stream "participant.<ordinal>.<generation>": a gap, then a length.
    stream = model_stream(3, PROCESS_FREEZE, "participant.2.1")
    first_gap_us = round(1e6 * (5.0 + 55.0 * stream.random()))
    length = 2 + math.floor(stream.random() * 5)
    first = freezes(frozen_ticks(TickBudget(params, 3, 2, 1), 30_000))[0]
    assert first == (math.ceil((TICK_US + first_gap_us) / TICK_US), length)


@pytest.mark.parametrize("models, message", [
    ({"freeze": HW_FREEZE}, "unknown participant models"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "min_ticks": 0}}, "min_ticks must be an int of at least 1"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "max_ticks": 2.0}}, "max_ticks must be an int"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "max_ticks": True}}, "max_ticks must be an int"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "min_ticks": 7}}, "freeze ticks: the minimum 7 is above the maximum 6"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "min_gap_s": 0.0}}, "min_gap_s must be a finite number above 0"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "max_gap_s": math.inf}}, "max_gap_s must be a finite number above 0"),
    ({PROCESS_FREEZE: {**HW_FREEZE, "min_gap_s": 61.0}}, "freeze gap: the minimum 61.0 is above the maximum 60.0"),
    ({CPU_BUDGET: {"handoffs": 0}}, "handoffs must be an int of at least 1"),
])
def test_a_bad_model_entry_is_refused(models, message):
    with pytest.raises(ValueError, match=message):
        TickBudget(models, 1, 1, 1)


def test_participant_models_carry_only_the_on_models_and_refuse_a_bad_draw():
    catalogue = {name: CATALOGUE[name] for name in (CPU_BUDGET, PROCESS_FREEZE, "wind")}
    entry = parse_profile("t", {"models": {
        CPU_BUDGET: {"p_on": 1.0, "params": {"handoffs": [50, 200]}},
        PROCESS_FREEZE: {"p_on": 0.0, "params": HW_FREEZE},
        "wind": {"p_on": 1.0, "params": {"speed_mps": 1.0, "direction_deg": 0.0, "turbulence_mps": 0.0}},
    }}, catalogue)
    models = participant_models(realize(entry, 5))
    assert list(models) == [CPU_BUDGET] and 50 <= models[CPU_BUDGET]["handoffs"] <= 200
    bad = parse_profile("t", {"models": {CPU_BUDGET: {"p_on": 1.0, "params": {"handoffs": 0}}}}, catalogue)
    with pytest.raises(ValueError, match="handoffs"):
        participant_models(realize(bad, 5))
    assert SIM_MODELS_KEY == "sim_models"


def test_the_profiles_give_the_designed_ranges():
    for name in ("quiet", "hw"):
        models = load_profile(name).models
        assert CPU_BUDGET not in models  # off in hw
        freeze = models[PROCESS_FREEZE]
        assert freeze.p_on == 0.5 and dict(freeze.params) == HW_FREEZE
    stress = load_profile("stress").models
    assert stress[CPU_BUDGET].p_on == 1.0
    assert (stress[CPU_BUDGET].params["handoffs"].lo, stress[CPU_BUDGET].params["handoffs"].hi) == (4, 12)
    assert stress[PROCESS_FREEZE].p_on == 1.0
    assert dict(stress[PROCESS_FREEZE].params) == {**HW_FREEZE, "min_ticks": 5, "max_ticks": 12}
    assert participant_models(realize(load_profile("none"), 1)) == {}
    for seed in range(1, 41):  # every draw of every profile is a valid budget
        for name in ("quiet", "hw", "stress"):
            participant_models(realize(load_profile(name), seed))


def test_a_freeze_schedule_asked_out_of_its_first_tick_anchors_there():
    schedule = FreezeSchedule(fixed(1, 0.05), model_stream(1, PROCESS_FREEZE, "participant.1.1"))
    assert [schedule.frozen(t) for t in (500_000, 520_000, 540_000, 560_000)] == [False, False, False, True]
