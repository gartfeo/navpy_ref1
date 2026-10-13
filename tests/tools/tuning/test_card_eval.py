"""Tests for the test-card tracking score (tools/tuning/card_eval.py)."""
import numpy as np
import pytest

from tools.tuning.card_eval import ideal_response, score_steps


def _card(dt=0.005, hold=3.0):
    t_s = np.arange(0, 4 * hold, dt)
    demand = np.where(t_s < hold, 0.0, np.where(t_s < 2 * hold, 25.0, np.where(t_s < 3 * hold, -25.0, 0.0)))
    return t_s, demand


def test_ideal_response_follows_rate_limit_then_time_constant():
    t_s, demand = _card()
    a = ideal_response(t_s, demand, 0.0, tconst=0.25, rmax_up=60.0, rmax_dn=60.0)
    k = np.searchsorted(t_s, 3.2)
    # rate-limited at 60 deg/s right after the step: 0.2 s gives about 12 deg
    assert a[k] == pytest.approx(12.0, abs=0.6)
    assert a[np.searchsorted(t_s, 5.9)] == pytest.approx(25.0, abs=0.01)


def test_ideal_tracking_scores_zero_and_lag_scores_more():
    t_s, demand = _card()
    t_us = t_s * 1e6
    ideal = ideal_response(t_s, demand, 0.0, 0.25, 60.0, 60.0)
    perfect = score_steps(t_us, demand, ideal, "roll", 0.25, 60.0, 60.0)
    assert len(perfect) == 3
    assert max(s.mae_deg for s in perfect) < 0.01     # each step restarts from the measured angle
    # a response 80 ms late tracks worse; one that overshoots shows it
    lag = np.interp(t_s - 0.08, t_s, ideal)
    late = score_steps(t_us, demand, lag, "roll", 0.25, 60.0, 60.0)
    assert min(s.mae_deg for s in late) > 0.5
    over = ideal + 3.0 * np.exp(-((t_s - 4.0) / 0.3) ** 2)
    first = score_steps(t_us, demand, over, "roll", 0.25, 60.0, 60.0)[0]
    assert first.overshoot == pytest.approx(3.0 / 25.0, abs=0.02)


def test_small_demand_changes_are_not_steps():
    t_s = np.arange(0, 6, 0.005)
    demand = 3.0 * np.sin(t_s)
    assert score_steps(t_s * 1e6, demand, demand, "roll", 0.25, 60.0, 60.0) == []


def test_steps_at_a_mode_change_are_not_scored():
    t_s, demand = _card()
    t_us = t_s * 1e6
    ideal = ideal_response(t_s, demand, 0.0, 0.25, 60.0, 60.0)
    all_steps = score_steps(t_us, demand, ideal, "roll", 0.25, 60.0, 60.0)
    skipped = score_steps(t_us, demand, ideal, "roll", 0.25, 60.0, 60.0, mode_change_us=np.array([5.9e6]))
    assert len(all_steps) == 3 and len(skipped) == 2
    assert skipped[0].t_us == pytest.approx(all_steps[0].t_us)
