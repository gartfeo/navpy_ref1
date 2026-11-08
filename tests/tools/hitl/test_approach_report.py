"""Step 27's series report (tools/hitl/approach_report.py, approach_run_checks.py, approach_truth.py) and host-load
sampler (host_load.py): a run counts only with every check passed, missing or changed evidence never counts, a miss
breaks the series, and 10/10 needs ten seeds once each under one frozen configuration. Synthetic run folders."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from navpy.core.kernel.location import Location
from tools.hitl import approach_report, approach_run_checks as checks, approach_truth as truth, approach_video, host_load
from tools.hitl import run_manifest
from tools.tuning import approach_chain as chain, approach_contact as contact, approach_finish as finish
from tools.tuning import approach_quiesce as stop

DOCK = Location(40.0, 44.0, 1300.0, is_absolute=True)
OFFSET = float(chain.UNREAL_ALT_OFFSET_M)
SOURCE = "gstreamer+rtsp://encoded"
CONTACT_US = 100_000_000


@pytest.fixture(autouse=True)
def _truth_and_profile(monkeypatch):
    """The board centre Unreal built and the HITL profile's stream: fixed for the synthetic runs."""
    monkeypatch.setattr(checks, "read_dock", lambda scenario_dir: DOCK)
    monkeypatch.setattr(checks, "chain_source_id", lambda: SOURCE)


def _path_rows(miss_east_m: float, at_us: int = CONTACT_US) -> list[dict]:
    """A northbound pass at the dock's height, miss_east_m east of it, crossing its latitude at board time at_us."""
    rows = []
    east = math.degrees(miss_east_m / (truth.EARTH_RADIUS_M * math.cos(math.radians(DOCK.lat))))
    for k in range(-5, 6):
        north_m = 0.4 * k                                   # 20 m/s at 20 ms per row
        rows.append({"board_time_us": at_us + k * 20_000,
                     "lat_deg": DOCK.lat + math.degrees(north_m / truth.EARTH_RADIUS_M),
                     "lon_deg": DOCK.lng + east, "alt_msl_m": DOCK.alt - OFFSET})
    return rows


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["board_time_us", "lat_deg", "lon_deg", "alt_msl_m"])
        writer.writeheader()
        writer.writerows(rows)


def _navpy_argv(run_dir: Path) -> list[str]:
    return ["nice", "-n", "10", "python", "-s", "-m", "navpy.main", "--detector-type", "real", "--detector-backend",
            "charuco", "--vision-profile", chain.VISION_PROFILE, "--integration-timing", str(run_dir / "r.json"),
            "--hitl-contact", str(run_dir / contact.LINK_FILE)]


IDENTITY = {"task_id": "t-1", "dock_uid": 7, "episode": 3}


def make_run(root: Path, index: int, seed: int | None = None, *, record_extra: dict | None = None,
             miss_m: float = 0.2, edit=None, authority: str = "command") -> Path:
    """A run folder that passes every check; edit(run, run_dir) changes it before the manifest is built."""
    run_id = f"approach-20261108T{index:06d}Z"
    run_dir = root / run_id
    (run_dir / "publisher").mkdir(parents=True)
    seed = index if seed is None else seed
    scenario = {"series_id": "s1", "seed_index": seed, "scene_hash": f"{seed:064d}", "limits": {"observe_s": 120.0}}
    record = {"run": {"run_id": run_id, "mode": "pc-sitl"}, "scenario": scenario,
              "integration_timing": {"source_id": SOURCE}, "substitutions": {"autopilot": "SITL"},
              **(record_extra or {})}
    (run_dir / "approach_record.json").write_text(json.dumps(record), encoding="utf-8")
    ended = {"ended_after_stop": True}
    run = {"run_id": run_id, "record": str(run_dir / "approach_record.json"), "scenario": scenario,
           "authority": authority, "admission_problems": [], "orbit_check": {"feasible": True},
           "sitl_binary": {"sha256": "b" * 64}, "param_files": [{"path": "a.parm", "sha256": "c" * 64}],
           "producers": {"router": {**ended, "argv": ["python", "-m", "tools.tuning.mav_router", "--client",
                                                        f"runner={chain.RUNNER_PORT}", "--client",
                                                        f"navpy={chain.NAVPY_PORT}"]},
                         "navpy": {**ended, "argv": _navpy_argv(run_dir)}, "contact": ended},
           "orphans": [], "scene": {"reproduced": True, "scene_hash": scenario["scene_hash"]},
           "board": {"link_health": {"ok": True}},
           "observation": {"ok": True, "authority": authority},
           "outcome": {"outcome": "docked", "ended_by": stop.ENDED_BY_RESULT,
                       "navpy_record": {"task": "t-1", "dock": 7, "episode": 3}, "problems": []},
           "airspeed": {"problems": []}, "sources": {"ok": True, "problems": []},
           "owners": {phase: {"ok": True, "problems": []} for phase in ("preflight", "ready", "end")},
           "timing": {"passed": True, "failures": [], "attitude": {"median_s": 0.001, "p95_abs_s": 0.002,
                                                                   "max_abs_s": 0.003, "uncertainty_s": 0.002},
                      "gimbal": {"median_s": 0.0, "p95_abs_s": 0.001, "max_abs_s": 0.004, "uncertainty_s": 0.002}},
           "host_load": {"ok": True}}
    (run_dir / "publisher" / "publisher.json").write_text(json.dumps({
        "publisher": {"accounted": True, "failed": False, "max_queue_level": 2, "frames_in": 10, "pushed": 10},
        "profile": {"fps": 30, "bitrate_kbps": 1340},
        "mediamtx": {"write_queue_size": 64}, "drain": {"ok": True}, "receiver": {"frames_dropped": 0}}), "utf-8")
    write_encoded(run_dir, fps=30.0, kbps=1340.0)
    (run_dir / contact.EVIDENCE_FILE).write_text("\n".join(json.dumps(e) for e in [
        {"event": "arm", "arm_id": "a1", **IDENTITY},
        {"event": "contact", "arm_id": "a1", "board_time_us": CONTACT_US}]) + "\n",
        encoding="utf-8")
    _write_csv(run_dir / "board" / "link.csv", _path_rows(miss_m))
    for name in (*finish.OTHER_EVIDENCE, "navpy_integration.jsonl", "unreal.log", "emulator.jsonl",
                 "publisher/frames-001.csv", "board/logs/00000001.BIN"):
        if not (run_dir / name).exists():
            (run_dir / name).parent.mkdir(parents=True, exist_ok=True)
            (run_dir / name).write_text("x\n", encoding="utf-8")
    (run_dir / "publisher_profile.json").write_text(json.dumps({"bitrate_kbps": 1340, "native_host": f"10.0.0.{index}"}),
                                                    encoding="utf-8")
    if edit:
        edit(run, run_dir)
    timing_roles = [("run_record", run_dir / "approach_record.json"),
                    ("navpy_integration_log", run_dir / "navpy_integration.jsonl"),
                    ("publisher_status", run_dir / "publisher" / "publisher.json"),
                    ("publisher_frames", run_dir / "publisher" / "frames-001.csv"),
                    ("unreal_log", run_dir / "unreal.log"), ("emulator_log", run_dir / "emulator.jsonl")]
    manifest = {"files": [{"role": role, "name": path.name, **run_manifest.digest(path)} for role, path in timing_roles
                          if path.exists()],
                "other_files": [{"name": str(path.relative_to(run_dir)), **run_manifest.digest(path)}
                                for path in sorted(run_dir.rglob("*")) if path.is_file()
                                and path.name not in {"approach_record.json", "publisher.json", "frames-001.csv",
                                                      "navpy_integration.jsonl", "unreal.log", "emulator.jsonl"}]}
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (run_dir / "approach_run.json").write_text(json.dumps(run), encoding="utf-8")
    return run_dir


def write_encoded(run_dir: Path, fps: float, kbps: float, units: int = 301) -> None:
    """The publisher's encoded log: units at the given rate, each of the bytes that make the given mean bitrate."""
    size = round(kbps * 1000 / 8 / fps)
    with open(run_dir / "publisher" / "encoded-001.csv", "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["au_index", "pts_ns", "keyframe", "bytes"])
        writer.writerows([i + 1, round(i * 1e9 / fps), int(i % 60 == 0), size] for i in range(units))


def failed(run_dir: Path) -> dict:
    return {name: found for name, found in checks.score_run(run_dir)["checks"].items() if found}


def test_a_complete_docked_command_run_passes_every_check(tmp_path):
    result = checks.score_run(make_run(tmp_path, 1))
    assert result["passed"], result["checks"]
    assert result["truth"]["closest"]["distance_m"] == pytest.approx(0.2, abs=1e-6)
    assert result["timing"]["gimbal"]["max_abs_s"] == 0.004


def _set(path: str, value):
    def edit(run, run_dir):
        keys = path.split(".")
        target = run
        for key in keys[:-1]:
            target = target[key]
        target[keys[-1]] = value
    return edit


@pytest.mark.parametrize("edit, check", [
    (_set("authority", "observation"), "command"),
    (_set("observation.authority", "observation"), "command"),
    (_set("producers.router.argv", ["mav_router", "--reader", f"navpy={chain.NAVPY_PORT}"]), "command"),
    (_set("outcome.outcome", "missed"), "docked"),
    (_set("outcome.ended_by", "deadline"), "docked"),
    (_set("outcome.navpy_record.episode", None), "contact"),
    (_set("outcome.navpy_record.episode", 4), "contact"),
    (_set("outcome.navpy_record.task", "t-2"), "contact"),
    (_set("timing", {"passed": False, "failures": ["gimbal: max |skew| over 8 ms"]}), "timing"),
    (_set("timing", None), "timing"),
    (_set("host_load", {"ok": False, "throttle_reasons": ["hw_thermal_slowdown"]}), "host"),
    (_set("orbit_check", {"feasible": False}), "geometry"),
    (_set("admission_problems", ["commissioning fault delay:1"]), "source"),
    (_set("sources", {"ok": False, "problems": ["two pose streams"]}), "source"),
    (_set("observation.ok", False), "verdict"),
    (_set("orphans", [123]), "verdict"),
    (_set("producers.contact.ended_after_stop", False), "verdict"),
], ids=lambda value: value if isinstance(value, str) else "")
def test_each_failed_check_stops_the_run_counting(tmp_path, edit, check):
    assert check in failed(make_run(tmp_path, 1, edit=edit))


@pytest.mark.parametrize("argv_edit", [
    lambda argv: [w.replace("real", "sim") for w in argv],
    lambda argv: [w.replace(chain.VISION_PROFILE, "ideal_360") for w in argv],
    lambda argv: argv + ["--raw-frame-diagnostics"],
    lambda argv: [w for w in argv if w != "--hitl-contact"],
    lambda argv: [w for w in argv if w != "--integration-timing"],
])
def test_ideal_raw_frame_or_sim_acquisition_cannot_substitute(tmp_path, argv_edit):
    def edit(run, run_dir):
        run["producers"]["navpy"]["argv"] = argv_edit(run["producers"]["navpy"]["argv"])
    assert "source" in failed(make_run(tmp_path, 1, edit=edit))


def test_a_record_of_another_video_source_cannot_substitute(tmp_path):
    run_dir = make_run(tmp_path, 1, record_extra={"integration_timing": {"source_id": "camera:1"}})
    assert "source" in failed(run_dir)


@pytest.mark.parametrize("events, problem", [
    ([], "0 contacts sent, not 1"),
    ([{"event": "arm", "arm_id": "a1", **IDENTITY},
      {"event": "contact", "arm_id": "a1", "board_time_us": CONTACT_US}] * 2,
     "2 contacts sent, not 1"),
    ([{"event": "contact", "arm_id": "a9", "board_time_us": CONTACT_US}], "the contact names no arm the service saw"),
])
def test_exactly_one_contact_for_a_seen_arm(tmp_path, events, problem):
    def edit(run, run_dir):
        (run_dir / contact.EVIDENCE_FILE).write_text("".join(json.dumps(e) + "\n" for e in events), "utf-8")
    assert problem in failed(make_run(tmp_path, 1, edit=edit))["contact"]


def test_a_pass_over_half_a_metre_is_a_miss_whatever_navpy_said(tmp_path):
    problems = failed(make_run(tmp_path, 1, miss_m=0.51))["truth"]
    assert any("not below 0.5 m" in p for p in problems)


def test_a_truth_crossing_away_from_the_contact_time_does_not_count(tmp_path):
    def edit(run, run_dir):
        _write_csv(run_dir / "board" / "link.csv", _path_rows(0.1, at_us=CONTACT_US + 200_000))
    problems = failed(make_run(tmp_path, 1, edit=edit))["truth"]
    assert any("is not the contact" in p for p in problems)


@pytest.mark.parametrize("name", list(finish.OTHER_EVIDENCE) + ["unreal.log", "publisher/publisher.json"])
def test_missing_evidence_never_counts(tmp_path, name):
    def edit(run, run_dir):
        (run_dir / name).unlink()
    assert "evidence" in failed(make_run(tmp_path, 1, edit=edit))


def test_a_contact_for_another_episode_than_navpys_docked_never_counts(tmp_path):
    def edit(run, run_dir):
        events = [{"event": "arm", "arm_id": "a1", **IDENTITY}, {"event": "arm", "arm_id": "a2", **IDENTITY,
                                                                  "episode": 4},
                  {"event": "contact", "arm_id": "a2", "board_time_us": CONTACT_US}]
        (run_dir / contact.EVIDENCE_FILE).write_text("".join(json.dumps(e) + "\n" for e in events), "utf-8")
    problems = failed(make_run(tmp_path, 1, edit=edit))["contact"]
    assert any("is not navpy's result episode" in p for p in problems)


def test_the_runners_own_result_outcome_counts_as_docked():
    """The runner's outcome() for navpy's result record that ended the observation (Codex round 1, P1)."""
    record = {"trigger": "result", "outcome": "docked", "barrier": "acked", "gimbal_stop": "sent"}
    run = {"outcome": stop.outcome([record], stop.ENDED_BY_RESULT)}
    assert checks.docked_check(run, Path(".")) == []
    assert checks.docked_check({"outcome": stop.outcome([record], stop.ENDED_BY_DEADLINE)}, Path("."))


def test_a_run_without_a_board_log_never_counts(tmp_path):
    def edit(run, run_dir):
        (run_dir / "board" / "logs" / "00000001.BIN").unlink()
    assert "the manifest names no board DataFlash log" in failed(make_run(tmp_path, 1, edit=edit))["evidence"]


@pytest.mark.parametrize("name", ["board/link.csv", "unreal.log", contact.EVIDENCE_FILE, "navpy.out",
                                  finish.HOST_LOAD_CSV, "board/logs/00000001.BIN", "contact_link_extra.txt"])
def test_evidence_changed_after_the_manifest_never_counts(tmp_path, name):
    run_dir = make_run(tmp_path, 1, edit=lambda run, d: (d / "contact_link_extra.txt").write_text("k", "utf-8"))
    with open(run_dir / name, "a", encoding="utf-8") as handle:
        handle.write("\n")
    assert "evidence" in failed(run_dir)


def test_a_publisher_queue_over_its_bound_or_an_undrained_stream_fails_the_video(tmp_path):
    def edit(run, run_dir):
        path = run_dir / "publisher" / "publisher.json"
        status = json.loads(path.read_text(encoding="utf-8"))
        status["publisher"]["max_queue_level"] = 65
        status["drain"] = {"ok": False}
        path.write_text(json.dumps(status), encoding="utf-8")
    problems = failed(make_run(tmp_path, 1, edit=edit))["video"]
    assert len(problems) == 2


@pytest.mark.parametrize("fps, kbps, problem", [
    (25.9, 1340.0, "unique rate 25.90 fps"),               # step 16's run: 656 missed slots
    (29.0, 1340.0, "unique rate 29.00 fps"),
    (30.0, 1042.0, "mean bitrate"),
])
def test_a_stream_off_its_unique_rate_or_bitrate_fails_the_video(tmp_path, fps, kbps, problem):
    run_dir = make_run(tmp_path, 1, edit=lambda run, d: write_encoded(d, fps, kbps))
    assert any(problem in p for p in failed(run_dir)["video"])


def test_a_stream_within_its_tolerances_passes_and_its_numbers_are_reported(tmp_path):
    run_dir = make_run(tmp_path, 1, edit=lambda run, d: write_encoded(d, 30.5, 1450.0))
    assert "video" not in failed(run_dir)
    stats = checks.score_run(run_dir)["video"]["encoded"]
    assert stats["units"] == 301 and stats["unique_fps"] == pytest.approx(30.5, rel=1e-6)
    assert approach_video.UNIQUE_RATE_TOLERANCE == 0.02 and approach_video.BITRATE_TOLERANCE == 0.10


@pytest.mark.parametrize("change", ["no encoded log", "receiver drop"])
def test_no_encoded_log_or_a_receiver_drop_fails_the_video(tmp_path, change):
    def edit(run, run_dir):
        if change == "no encoded log":
            (run_dir / "publisher" / "encoded-001.csv").unlink()
        else:
            path = run_dir / "publisher" / "publisher.json"
            status = json.loads(path.read_text(encoding="utf-8"))
            status["receiver"]["frames_dropped"] = 1
            path.write_text(json.dumps(status), encoding="utf-8")
    assert "video" in failed(make_run(tmp_path, 1, edit=edit))


def test_an_unreadable_run_file_is_a_failed_attempt(tmp_path):
    run_dir = make_run(tmp_path, 1)
    (run_dir / "approach_run.json").write_text("{", encoding="utf-8")
    assert checks.score_run(run_dir)["passed"] is False


# ---- the series -----------------------------------------------------------------------------------------------
def observation_run(tmp_path, **kwargs) -> Path:
    """The held-out 10-minute observation run of the same configuration (its own folder, outside the series)."""
    extra = kwargs.pop("edit", None) or (lambda r, d: None)

    def covered(run, run_dir):
        run["timing"]["observation"] = True
        run["scenario"] = {**run["scenario"], "series_id": "obs"}
        extra(run, run_dir)
    return make_run(tmp_path / "observation", 99, authority="observation", edit=covered, **kwargs)


def series(tmp_path, **kwargs) -> dict:
    return approach_report.report(tmp_path, "s1", observation_run(tmp_path, **kwargs))


def test_ten_passing_runs_of_ten_seeds_under_one_configuration_score_ten_of_ten(tmp_path, capsys):
    for i in range(10):
        make_run(tmp_path, i, seed=9 - i)
    observation = observation_run(tmp_path)
    assert approach_report.main(["--series", "s1", "--run-root", str(tmp_path), "--observation-run",
                                 str(observation), "--out", str(tmp_path / "r.json")]) == 0
    result = json.loads((tmp_path / "r.json").read_text(encoding="utf-8"))
    assert result["score"] == "10/10" and result["passed"]
    assert result["substitutions"] == [{"autopilot": "SITL"}]
    assert result["timing_skew"]["gimbal"] == {"max_abs_s": 0.004, "runs": 10, "budget_s": 0.008}
    out = capsys.readouterr().out
    assert "10/10 PASS" in out and "substitution autopilot: SITL" in out and "timing skew gimbal" in out


def test_a_miss_breaks_the_series_and_later_passes_do_not_mend_it(tmp_path):
    for i in range(10):
        make_run(tmp_path, i, edit=_set("outcome.outcome", "missed") if i == 3 else None)
    result = series(tmp_path)
    assert result["score"] == "3/10" and not result["passed"]
    assert any("attempt 4" in p for p in result["problems"])


def test_fewer_than_ten_attempts_or_a_repeated_seed_fails(tmp_path):
    for i in range(9):
        make_run(tmp_path, i)
    assert series(tmp_path)["problems"] == ["9 of 10 attempts"]
    make_run(tmp_path, 9, seed=0)
    assert any("seeds" in p for p in approach_report.report(tmp_path, "s1", tmp_path / "observation" /
                                                            "approach-20261108T000099Z")["problems"])


def _profile(bitrate):
    def edit(run, run_dir):
        (run_dir / "publisher_profile.json").write_text(json.dumps({"bitrate_kbps": bitrate}), "utf-8")
    return edit


@pytest.mark.parametrize("changed", [
    {"record_extra": {"law": "other"}},
    {"edit": _set("sitl_binary", {"sha256": "d" * 64})},
    {"edit": _set("param_files", [{"path": "a.parm", "sha256": "e" * 64}])},
    {"edit": _profile(2000)},
], ids=["record", "sitl binary", "parameters", "publisher profile"])
def test_a_changed_configuration_starts_a_new_series(tmp_path, changed):
    for i in range(10):
        make_run(tmp_path, i, **(changed if i >= 5 else {}))
    assert any("configurations" in p for p in series(tmp_path)["problems"])


def test_the_per_run_publisher_address_is_not_a_configuration_change(tmp_path):
    for i in range(10):
        make_run(tmp_path, i)                 # native_host differs per run
    assert series(tmp_path)["passed"]


@pytest.mark.parametrize("observation, problem", [
    (None, "no 10-minute observation run given"),
    ({"edit": _set("timing.observation", False)}, "the observation run failed"),
    ({"edit": _set("host_load", {"ok": False})}, "the observation run failed"),
    ({"record_extra": {"law": "other"}}, "another configuration"),
])
def test_ten_passes_still_need_a_passing_observation_run_of_the_same_configuration(tmp_path, observation, problem):
    for i in range(10):
        make_run(tmp_path, i)
    run = None if observation is None else observation_run(tmp_path, **observation)
    result = approach_report.report(tmp_path, "s1", run)
    assert result["score"] == "10/10" and not result["passed"]
    assert any(problem in p for p in result["problems"])


def test_a_command_run_is_not_an_observation_run(tmp_path):
    run = make_run(tmp_path, 1, edit=_set("timing.observation", True))
    assert approach_report.score_observation(run)["checks"]["observation"]


def test_only_the_series_runs_are_attempts_and_attempts_after_ten_are_kept(tmp_path):
    for i in range(11):
        make_run(tmp_path, i, seed=i % 10)
    other = make_run(tmp_path / "x", 1)
    other.rename(tmp_path / "approach-20261108T999999Z")
    record = tmp_path / "approach-20261108T999999Z" / "approach_run.json"
    record.write_text(json.dumps({**json.loads(record.read_text("utf-8")), "scenario": {"series_id": "s2"}}), "utf-8")
    result = series(tmp_path)
    assert result["score"] == "10/10" and result["after_series"] == 1 and len(result["attempts"]) == 11


def test_a_folder_whose_run_file_is_missing_still_counts_as_a_failed_attempt(tmp_path):
    run_dir = make_run(tmp_path, 0)
    (run_dir / "approach_run.json").unlink()
    result = series(tmp_path)
    assert len(result["attempts"]) == 1 and result["score"] == "0/10"


def test_the_frozen_configuration_ignores_only_the_run_identity_and_seed():
    base = {"run": {"run_id": "a"}, "scenario": {"series_id": "s", "seed_index": 0, "scene_hash": "1"}, "law": "x"}
    same = {"run": {"run_id": "b"}, "scenario": {"series_id": "s", "seed_index": 4, "scene_hash": "2"}, "law": "x"}
    assert approach_report.frozen_fingerprint(base) == approach_report.frozen_fingerprint(same)
    assert approach_report.frozen_fingerprint(base) != approach_report.frozen_fingerprint({**same, "law": "y"})
    assert approach_report.frozen_fingerprint(base, {"sitl_binary": "a"}) != approach_report.frozen_fingerprint(
        same, {"sitl_binary": "b"})


# ---- the truth cross-check ------------------------------------------------------------------------------------
def test_the_closest_pass_is_between_logged_poses():
    path = [(t, lat, lon, h) for t, lat, lon, h in truth.path_rows(
        [{k: str(v) for k, v in row.items()} for row in _path_rows(0.3)], OFFSET)]
    sparse = path[0::2]                      # rows 40 ms apart, none at the crossing: it falls between two of them
    assert min(math.dist((0, 0), (0.3, 0.4 * k)) for k in (-5, -3, -1, 1, 3, 5)) == pytest.approx(0.5)
    closest = truth.closest_pass(sparse, DOCK)
    assert closest["distance_m"] == pytest.approx(0.3, abs=1e-6)
    assert closest["board_time_s"] == pytest.approx(CONTACT_US * 1e-6, abs=1e-6)


@pytest.mark.parametrize("cut", ["hole", "reboot", "repeat"])
def test_a_logging_hole_or_a_board_reset_is_never_bridged_into_a_crossing(tmp_path, cut):
    """Codex round 1: two poses either side of a hole would make a straight chord through the net."""
    rows = _path_rows(5.0)                        # flown 5 m east of the dock
    before, after = dict(rows[4]), dict(rows[6])
    after["lon_deg"] = DOCK.lng - (float(after["lon_deg"]) - DOCK.lng)    # the far side: the chord crosses the centre
    after["board_time_us"] = {"hole": before["board_time_us"] + 60_000, "reboot": 5_000,
                              "repeat": before["board_time_us"]}[cut]
    path = truth.path_rows([{k: str(v) for k, v in row.items()} for row in (before, after)], OFFSET)
    assert truth.closest_pass(path, DOCK) is None
    joined = dict(after, board_time_us=before["board_time_us"] + 20_000)
    path = truth.path_rows([{k: str(v) for k, v in row.items()} for row in (before, joined)], OFFSET)
    assert truth.closest_pass(path, DOCK)["distance_m"] < 0.5           # the same chord within the limit joins


def test_bad_rows_are_skipped_and_one_pose_is_no_path(tmp_path):
    rows = [{"board_time_us": "", "lat_deg": "40", "lon_deg": "44", "alt_msl_m": "1"},
            {"board_time_us": "1", "lat_deg": "nan", "lon_deg": "44", "alt_msl_m": "1"},
            {"board_time_us": "2", "lat_deg": "40", "lon_deg": "44", "alt_msl_m": "1"}]
    assert len(truth.path_rows(rows, OFFSET)) == 1
    _write_csv(tmp_path / "link.csv", [{k: v for k, v in rows[2].items()}])
    result = truth.cross_check(tmp_path / "link.csv", DOCK, OFFSET, 0.0)
    assert not result["ok"] and "no segment of two poses" in result["problems"][0] and result["skipped"] == 0
    assert not truth.cross_check(tmp_path / "missing.csv", DOCK, OFFSET, 0.0)["ok"]


# ---- host load --------------------------------------------------------------------------------------------------
def test_cpu_busy_share_from_proc_stat():
    before = host_load.parse_cpu("cpu  100 0 100 700 100 0 0 0 0 0\ncpu0 1 2 3 4\n")
    after = host_load.parse_cpu("cpu  200 0 200 800 100 0 0 0 0 0\n")
    assert host_load.busy_pct(before, after) == pytest.approx(200 / 300 * 100)
    assert host_load.busy_pct(after, after) is None


def test_gpu_samples_and_their_slowdowns():
    sample = host_load.parse_gpu("37, 61, 0x0000000000000040\n")
    assert (sample.util_pct, sample.temp_c, host_load.slowdowns(sample.throttle_mask)) == (
        37.0, 61.0, ["hw_thermal_slowdown"])
    assert host_load.slowdowns(0x4) == [] and host_load.parse_gpu("[N/A], 50, 0x0").error


def test_the_sampler_records_each_period_until_its_stop_and_fails_on_throttling_or_unread_gpus():
    ticks = iter([0.0, 1.0, 2.0, 3.0])
    cpu = iter([host_load.CpuTimes(0, 100), host_load.CpuTimes(50, 200), host_load.CpuTimes(60, 300)])
    gpus = iter([host_load.GpuSample(10.0, 60.0, 0), host_load.GpuSample(20.0, 70.0, 0x20)])
    sleeps = []

    def sleep(seconds):
        sleeps.append(seconds)
        if len(sleeps) == 3:
            raise KeyboardInterrupt

    out = io_text()
    summary = host_load.run(out, lambda: next(cpu), lambda: next(gpus), lambda: next(ticks), sleep)
    assert sleeps == [host_load.PERIOD_S] * 3 and summary["samples"] == 2
    assert summary["cpu_busy_max_pct"] == 50.0 and summary["gpu_temp_max_c"] == 70.0
    assert summary["throttle_reasons"] == ["sw_thermal_slowdown"] and summary["ok"] is False
    assert out.getvalue().count("\n") == 3                       # header and two rows
    assert host_load.summarize([])["ok"] is False
    unread = {"cpu_busy_pct": 1.0, "gpu_util_pct": None, "gpu_temp_c": None, "gpu_throttle_mask": None,
              "gpu_error": "exit 9"}
    assert host_load.summarize([unread])["ok"] is False


def io_text():
    import io
    return io.StringIO()
