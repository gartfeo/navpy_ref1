"""Step 9's wall-bug tool: stalls and failures from wall logs, the profile fit and the seed search
(no runs launched)."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import pytest

from navpy.sim.lockstep.sim_profile import CATALOGUE, PROFILES_FILE, parse_profile
from scripts import wall_bug_fit as fit
from scripts.wall_bug_logs import (
    GAP_GATE_MS,
    STALL_MIN_MS,
    Stall,
    UavArrivals,
    failure_kinds,
    span_s,
    stalls,
    uav_arrivals,
    wall_failures,
)

T0 = 1_792_000_000.0
#: The wall speedup of the logged runs: 10 ms of wall is 100 ms of aircraft time.
SPEEDUP = 10


def _rows(boots: list[int], walls: list[float]) -> list[tuple[float, int]]:
    return list(zip(walls, boots))


def _uav(stall_at_ms: int | None = None, stall_ms: int = 0, *, offset_ms: int = 120, jitter_ms: int = 0,
         seconds: int = 60) -> UavArrivals:
    """ATTITUDE every 20 ms of aircraft time; the GCS gets each at its time, the NavPy ``offset_ms``
    of aircraft time earlier, and a NavPy stall holds its packets from ``stall_at_ms`` for ``stall_ms``."""
    boots = list(range(0, seconds * 1000, 20))
    ground = [(T0 + b / 1000 / SPEEDUP, b) for b in boots]
    navpy = []
    for i, b in enumerate(boots):
        aircraft = b - offset_ms + (jitter_ms if i % 2 else 0)
        if stall_at_ms is not None and stall_at_ms <= b < stall_at_ms + stall_ms:
            aircraft = stall_at_ms + stall_ms - offset_ms  # held, then read in one burst
        navpy.append((T0 + aircraft / 1000 / SPEEDUP, b))
    return UavArrivals(1, navpy, ground)


# --- stalls ---


def test_a_navpy_stall_is_found_at_its_start_with_its_length_above_the_paths_own_offset():
    found = stalls(_uav(stall_at_ms=10_000, stall_ms=700))
    # A burst: the first held packet is the latest, and the lag drains to the threshold over 500 ms.
    assert found == [Stall(1, 10_000, 700, rise_ms=0, drain_ms=500)] and not found[0].slow


def _slow(start_ms: int, peak_ms: int, rise_ms: int, drain_ms: int, seconds: int = 60) -> UavArrivals:
    """A reader that falls behind while it keeps receiving: lag rises linearly to ``peak_ms`` over
    ``rise_ms`` of aircraft time, then drains over ``drain_ms``."""
    boots = list(range(0, seconds * 1000, 20))
    ground = [(T0 + b / 1000 / SPEEDUP, b) for b in boots]

    def extra(b: int) -> float:
        if start_ms <= b < start_ms + rise_ms:
            return peak_ms * (b - start_ms) / rise_ms
        if start_ms + rise_ms <= b < start_ms + rise_ms + drain_ms:
            return peak_ms * (start_ms + rise_ms + drain_ms - b) / drain_ms
        return 0.0
    navpy = [(T0 + (b + extra(b)) / 1000 / SPEEDUP, b) for b in boots]
    return UavArrivals(1, navpy, ground)


def test_a_lag_held_at_its_peak_rises_to_the_first_peak_packet():
    uav = _slow(10_000, peak_ms=1_000, rise_ms=4_000, drain_ms=4_000)
    navpy = [(T0 + (b + 1_000) / 1000 / SPEEDUP, b) if 14_000 <= b < 16_000 else (w, b)
             for w, b in uav.navpy]  # flat at the peak for 2 s
    found = stalls(UavArrivals(1, navpy, uav.ground))
    # The drain runs from the first peak packet (14 s) to the last late one (17.2 s), not from the last peak.
    assert abs(found[0].rise_ms - 3_200) <= 40 and abs(found[0].drain_ms - 3_200) <= 40


def test_a_slow_lag_measures_its_rise_and_drain_from_the_threshold():
    found = stalls(_slow(10_000, peak_ms=1_000, rise_ms=8_000, drain_ms=4_000))
    assert len(found) == 1 and found[0].slow and found[0].length_ms == 1_000
    # 200 ms of the 1000 ms peak lie under the threshold: 4/5 of each slope is seen.
    assert abs(found[0].rise_ms - 6_400) <= 40 and abs(found[0].drain_ms - 3_200) <= 40


def test_jitter_under_the_threshold_and_a_constant_offset_are_no_stall():
    assert stalls(_uav(jitter_ms=STALL_MIN_MS - 20)) == []
    assert stalls(_uav(offset_ms=-400)) == []


def test_a_stall_just_under_and_at_the_threshold():
    assert stalls(_uav(stall_at_ms=5_000, stall_ms=STALL_MIN_MS - 20)) == []
    assert [s.length_ms for s in stalls(_uav(stall_at_ms=5_000, stall_ms=STALL_MIN_MS))] == [STALL_MIN_MS]


def test_a_stall_still_open_at_the_end_of_the_log_still_counts():
    uav = _uav(stall_at_ms=59_000, stall_ms=2_000, seconds=62)
    held = [(wall, boot) for wall, boot in uav.navpy if boot < 59_500]  # the log ends inside the stall
    found = stalls(UavArrivals(1, held, uav.ground))
    assert len(found) == 1 and found[0].boot_ms == 59_000 and found[0].open_at_end
    assert not stalls(_uav(stall_at_ms=10_000, stall_ms=700))[0].open_at_end


def test_the_aircraft_time_needs_a_fresh_ground_packet():
    uav = _uav()
    assert uav.boot_at(T0 + 1.0) == 10_000
    assert uav.boot_at(T0 - 0.01) is None
    assert uav.boot_at(T0 + 6.0 + 0.25) is None  # past the GCS's last packet by more than its staleness
    assert span_s(uav) == pytest.approx(59.98)


# --- files ---


def _write_arrivals(path: Path, rows: list[tuple[float, int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("wall_s,boot_ms\n" + "".join(f"{w},{b}\n" for w, b in rows), encoding="utf-8")


def test_named_files_pair_the_navpy_with_the_gcs(tmp_path):
    uav = _uav(stall_at_ms=1_000, stall_ms=400)
    times = tmp_path / "source-time"
    _write_arrivals(times / "pose_cadence_attitude_arrival_uav_2_navpy.csv", list(uav.navpy))
    _write_arrivals(times / "pose_cadence_attitude_arrival_uav_2_gcs.csv", list(uav.ground))
    pairs = uav_arrivals(tmp_path, 2)
    assert len(pairs) == 1 and [s.length_ms for s in stalls(pairs[0])] == [400]
    assert uav_arrivals(tmp_path, 1) == []


def test_per_process_files_pair_each_navpy_with_the_ground_process_it_overlaps_most(tmp_path):
    early, late = _uav(seconds=10), _uav(seconds=10, stall_at_ms=2_000, stall_ms=500)
    shift = lambda rows, s: [(w + s, b) for w, b in rows]  # noqa: E731
    times = tmp_path / "source-time"
    for uav in (1, 2):  # pids 10 and 20 wrote every UAV: ground processes, one per session
        _write_arrivals(times / f"pose_cadence_attitude_arrival_uav_{uav}_pid_10.csv", list(early.ground))
        _write_arrivals(times / f"pose_cadence_attitude_arrival_uav_{uav}_pid_20.csv", shift(late.ground, 100))
    _write_arrivals(times / "pose_cadence_attitude_arrival_uav_1_pid_31.csv", list(early.navpy))
    _write_arrivals(times / "pose_cadence_attitude_arrival_uav_1_pid_32.csv", shift(late.navpy, 100))
    pairs = uav_arrivals(tmp_path, 1)
    assert len(pairs) == 2
    assert [[s.length_ms for s in stalls(p)] for p in pairs] == [[], [500]]


# --- failures ---


def _log(path: Path, lines: list[tuple[float, str]]) -> None:
    text = "".join(f"{datetime.fromtimestamp(w).strftime('%Y-%m-%d %H:%M:%S,%f')[:-3]} INFO {line}\n"
                   for w, line in lines)
    path.write_text(text, encoding="utf-8")


def test_a_logged_nav_failure_has_its_kind_and_its_aircraft_time_after_the_nav_entry(tmp_path):
    uav = _uav()
    _log(tmp_path / "uav_1_navigation.log", [
        (T0 + 2.0, "INIT: NAV MODE"), (T0 + 2.9, "NAV_FAIL: source_command_stale task=1 obj=0"),
        (T0 + 3.0, "NAV_FAIL: missed_dock task=1 obj=0")])
    failures = wall_failures(tmp_path, 1, [uav])
    assert [(f.kind, f.text) for f in failures] == [("source_command_stale", "NAV_FAIL: source_command_stale")]
    assert failures[0].after_nav_s == pytest.approx(9.0, abs=0.03)
    assert wall_failures(tmp_path, 1)[0].after_nav_s is None  # no arrivals: no aircraft time


def test_report_gate_errors_are_failures_and_a_long_source_gap_is_not(tmp_path):
    episodes = [{"longest_gap_ms": GAP_GATE_MS + 240.0}, {"longest_gap_ms": GAP_GATE_MS}]
    report = {"vehicles": [
        {"sys_id": 3, "errors": ["max roll-command step 56.90deg > 50.00deg", "something else"],
         "cadence": {"episodes": episodes}},
        {"sys_id": 1, "errors": [], "cadence": {"episodes": [{"longest_gap_ms": None}]}}]}
    (tmp_path / "report.json").write_text(json.dumps(report), encoding="utf-8")
    assert [(f.kind, f.text) for f in wall_failures(tmp_path, 3)] == [
        ("roll_step", "max roll-command step 56.90deg > 50.00deg")]
    assert wall_failures(tmp_path, 1) == []


def test_failure_kinds_read_lockstep_errors_and_log_lines_alike():
    assert failure_kinds([
        "vehicle 2: max observation gap 1.180s > 0.500s", "median observation gap 0.300s > 0.200s",
        "DOCK_FAILED: no_detection", "max roll-command step 54.90deg > 50.00deg",
        "2026-01-01 00:02:37,484 INFO NAV_FAIL: source_command_stale task=1 obj=0"]) == {
        "observation_gap", "no_detection", "roll_step", "source_command_stale"}
    assert failure_kinds(["final-approach episode reached SNAP without DOCKED",
                          "NAV_FAIL: missed_dock", "forwarding: NAVLINK [1] router_filtered at tick 3"]) == set()


# --- fit ---


def _stalls(lengths: list[int]) -> list[Stall]:
    return [Stall(1, i * 1000, n) for i, n in enumerate(lengths)]


def test_the_freeze_fit_keeps_the_walls_share_at_or_over_the_tail_bound():
    lengths = [200] * 15 + [520]  # 1 in 16 at or over 500 ms
    freeze = fit.freeze_fit(_stalls(lengths), 80.0, 500)
    lo, hi, gate = freeze["min_ticks"], freeze["max_ticks"], 25
    assert lo == STALL_MIN_MS // fit.TICK_MS
    assert (hi - gate + 1) / (hi - lo + 1) >= 1 / 16 > (hi - 1 - gate + 1) / (hi - 1 - lo + 1)
    assert (freeze["min_gap_s"] + freeze["max_gap_s"]) / 2 == pytest.approx(80.0 / 16)
    assert freeze["min_gap_s"] == pytest.approx(2.5)


def test_the_fit_reaches_the_longest_stall_under_the_bound_and_never_past_the_longest():
    assert fit.freeze_fit(_stalls([200, 260, 300]), 30.0, 500)["max_ticks"] == 15
    assert fit.freeze_fit(_stalls([200, 300, 1900]), 30.0, 500)["max_ticks"] == 32  # 8 of 23 at 25 up
    assert fit.freeze_fit(_stalls([520, 1900]), 30.0, 500)["max_ticks"] == 95  # all over: up to the longest
    assert fit.freeze_fit(_stalls([240] * 3 + [600]), 30.0, 500)["max_ticks"] == 29  # 5 of 20 at 25 up
    with pytest.raises(ValueError, match="no stall"):
        fit.freeze_fit([], 30.0, 500)


def test_the_tail_bound_follows_the_walls_failures():
    assert fit.tail_bound_ms({"source_command_stale", "roll_step"}) == fit.SOURCE_AGE_MS == 1000
    assert fit.tail_bound_ms({"no_detection", "observation_gap"}) == GAP_GATE_MS == 500
    assert fit.tail_bound_ms(set()) == GAP_GATE_MS


def test_the_fitted_profile_is_a_valid_profile_and_the_shipped_wall_profiles_are_its_output():
    freeze = {"min_ticks": 10, "max_ticks": 25, "min_gap_s": 2.608, "max_gap_s": 7.823}
    parse_profile("fitted", fit.fitted_profile("process_freeze", freeze), CATALOGUE)
    shipped = json.loads(PROFILES_FILE.read_text(encoding="utf-8"))["profiles"]
    for name, model in (("wall_todo31", "process_freeze"), ("wall_todo65", "reader_lag")):
        params = shipped[name]["models"][model]["params"]
        assert shipped[name] == fit.fitted_profile(model, params), name
        parse_profile(name, shipped[name], CATALOGUE)


def test_the_lag_fit_takes_the_slow_lags_over_the_bound_and_scales_their_slopes_to_zero():
    found = [Stall(1, 0, 1_000, rise_ms=8_000, drain_ms=4_000),   # slow, scaled x 1000/800
             Stall(1, 0, 2_200, rise_ms=10_000, drain_ms=20_000),  # slow, x 2200/2000
             Stall(1, 0, 3_000, rise_ms=0, drain_ms=900),          # a burst: no lag episode
             Stall(1, 0, 400, rise_ms=4_000, drain_ms=4_000)]      # slow but under the bound
    lag = fit.lag_fit(found, 120.0, 500)
    assert lag == {"min_peak_ticks": 50, "max_peak_ticks": 110, "min_rise_s": 10.0, "max_rise_s": 11.0,
                   "min_drain_s": 5.0, "max_drain_s": 22.0, "min_gap_s": 30.0, "max_gap_s": 90.0}
    with pytest.raises(ValueError, match="no slow lag"):
        fit.lag_fit(found[2:], 120.0, 500)
    with pytest.raises(ValueError, match="no slow lag"):
        fit.lag_fit(found, 0.0, 500)


def test_a_slow_lag_the_log_ended_inside_is_left_out_of_the_lag_fit():
    uav = _slow(10_000, peak_ms=1_000, rise_ms=8_000, drain_ms=4_000)
    cut = UavArrivals(1, [(w, b) for w, b in uav.navpy if b < 18_000], uav.ground)  # ends at the peak
    (open_lag,) = stalls(cut)
    assert open_lag.slow and open_lag.open_at_end and open_lag.drain_ms < 200  # the drain was not seen
    closed = Stall(1, 0, 1_000, rise_ms=8_000, drain_ms=4_000)
    assert fit.lag_fit([open_lag, closed], 60.0, 500) == fit.lag_fit([closed], 60.0, 500)
    with pytest.raises(ValueError, match="no slow lag"):
        fit.lag_fit([open_lag], 60.0, 500)


def test_a_nav_window_runs_from_nav_entry_to_its_end_line_or_the_last_line(tmp_path):
    run = tmp_path / "wall"
    uav = _uav()
    _write_arrivals(run / "source-time" / "pose_cadence_attitude_arrival_uav_1_navpy.csv", list(uav.navpy))
    _write_arrivals(run / "source-time" / "pose_cadence_attitude_arrival_uav_1_gcs.csv", list(uav.ground))
    _log(run / "uav_1_navigation.log", [(T0 + 0.5, "INIT: NAV MODE"), (T0 + 1.5, "RESET: PASSED POI"),
                                        (T0 + 2.0, "INIT: NAV MODE"), (T0 + 2.5, "DOCKED: task=2"),
                                        (T0 + 3.0, "INIT: NAV MODE"), (T0 + 4.0, "other")])
    windows = fit.nav_windows(run, 1, uav_arrivals(run, 1))
    assert [(round(a, 1), round(b, 1)) for a, b in windows] == [(5.0, 15.0), (20.0, 25.0), (30.0, 40.0)]


def test_fit_pools_the_runs_and_names_the_walls_failures(tmp_path):
    run = tmp_path / "wall"
    uav = _uav(stall_at_ms=10_000, stall_ms=1_100)
    _write_arrivals(run / "source-time" / "pose_cadence_attitude_arrival_uav_2_navpy.csv", list(uav.navpy))
    _write_arrivals(run / "source-time" / "pose_cadence_attitude_arrival_uav_2_gcs.csv", list(uav.ground))
    _log(run / "uav_2_navigation.log", [(T0 + 0.9, "INIT: NAV MODE"),
                                        (T0 + 1.1, "NAV_FAIL: source_command_stale task=1 obj=0")])
    result = fit.fit([run])
    assert result["wall_kinds"] == ["source_command_stale"] and result["tail_bound_ms"] == 1000
    assert result["stalls"] == result["at_or_over_tail"] == 1
    assert result["process_freeze"]["max_ticks"] == 55 and result["nav_s"] == pytest.approx(2.0, abs=0.03)
    with pytest.raises(ValueError, match="no slow lag"):  # a burst is no lag episode
        fit.fit([run], "reader_lag")
    assert result["per_uav"][1]["failures"][0]["after_nav_s"] == pytest.approx(2.0, abs=0.03)


# --- search ---


def _attempt(run: str, cls: str, digest: str | None, errors: list[str]) -> dict:
    return {"run": run, "class": cls, "digest": digest, "errors": errors}


def _result(seed: int, status: str, errors: list[str]) -> dict:
    """A settled seed; a failing one confirmed by its rerun at one digest."""
    first = _attempt(f"wall_todo65-{seed}", "pass" if status == "pass" else "product", f"d{seed}", errors)
    rerun = [] if status == "pass" else [{**first, "run": f"wall_todo65-{seed}-r1"}]
    return {"profile": "wall_todo65", "seed": seed, "status": status, "attempts": [first, *rerun]}


def test_the_search_stops_at_the_first_confirmed_failure_of_a_wall_kind(monkeypatch, tmp_path):
    outcomes = {1: _result(1, "known", ["forwarding: NAVLINK [1] router_filtered at tick 3"]),
                2: _result(2, "pass", []),
                3: _result(3, "determinism_bug", ["max observation gap 0.9s > 0.500s"]),
                4: _result(4, "product", ["vehicle 2: max observation gap 0.880s > 0.500s"]),
                5: _result(5, "product", ["vehicle 1: max roll-command step 60.00deg > 50.00deg"])}
    settled: list[int] = []
    monkeypatch.setattr(fit, "settle", lambda job, base, runner: settled.append(job.seed) or outcomes[job.seed])
    record = fit.search("wall_todo65", {"observation_gap", "roll_step"}, [1, 2, 3, 4, 5, 6], tmp_path,
                        lambda job, d: 0, 2)
    assert record["reproduced"] and record["seed"] == 4
    assert settled == [1, 2, 3, 4]  # whole batches of 2: seed 5 is never run
    assert [r["kinds"] for r in record["tried"]] == [[], [], [], ["observation_gap"]]


def test_a_known_todos_failure_of_a_wall_kind_reproduces(monkeypatch, tmp_path):
    monkeypatch.setattr(fit, "settle", lambda job, base, runner: _result(
        job.seed, "known", ["vehicle 1: max observation gap 0.700s > 0.500s"]))
    record = fit.search("wall_todo65", {"observation_gap"}, [5], tmp_path, lambda job, d: 0, 1)
    assert record["reproduced"] and record["seed"] == 5


def test_a_search_without_a_match_in_its_budget_is_unreproduced(monkeypatch, tmp_path):
    monkeypatch.setattr(fit, "settle", lambda job, base, runner: _result(
        job.seed, "product", ["vehicle 1: max roll-command step 60.00deg > 50.00deg"]))
    record = fit.search("wall_todo65", {"no_detection"}, [7, 8, 9], tmp_path, lambda job, d: 0, 2)
    assert not record["reproduced"] and record["seed"] is None and len(record["tried"]) == 3


def test_a_seeds_kinds_include_its_navpys_logged_failures(tmp_path):
    run = tmp_path / "wall_todo31-3"
    run.mkdir()
    (run / "uav_2_navigation.log").write_text("x INFO NAV_FAIL: source_command_stale task=1 obj=0\n",
                                              encoding="utf-8")
    result = _result(3, "product", ["vehicle 2: final-approach episode reached SNAP without DOCKED"])
    result["attempts"][0]["run"] = run.name
    assert fit.run_kinds(result, tmp_path) == {"source_command_stale"}


def test_a_seeds_kinds_come_from_the_run_its_rerun_confirmed(tmp_path):
    result = {"seed": 6, "attempts": [
        _attempt("wall_todo65-6", "setup", None, ["vehicle 1: max roll-command step 60.00deg > 50.00deg"]),
        _attempt("wall_todo65-6-r1", "product", "d6", ["vehicle 2: max observation gap 0.880s > 0.500s"]),
        _attempt("wall_todo65-6-r2", "product", "d6", ["vehicle 2: max observation gap 0.880s > 0.500s"])]}
    assert fit.run_kinds(result, tmp_path) == {"observation_gap"}
    result["attempts"][-1]["digest"] = "other"
    with pytest.raises(ValueError, match="no confirmed pair"):
        fit.run_kinds(result, tmp_path)
