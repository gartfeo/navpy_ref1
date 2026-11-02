"""The hand-run check: facts of a run folder, whatever is missing from it."""

from __future__ import annotations

import io
import json
import sys

import pytest

from navpy.core.approach.logs.log_events import STANDARD_EVENT_HEADER
from scripts import delivery_demo_check as check
from scripts import delivery_demo_check_uav as uav_check

CONFIG = "18:11:13.709,EVENT:CONFIG,min_log_interval_s=0.1;schema=primary\n"
HEADER = STANDARD_EVENT_HEADER + "\n"


def _armed(vehicle, episode=1, source_ts=4034.0, wall=1793542413.0, time="18:13:33.000"):
    identity = f"vehicle={vehicle};task=1;dock=0;episode={episode}"
    return f"{time},EVENT:DOCK_ARMED,{identity};source_ts={source_ts};wall_s={wall}\n"


def _command(time="18:13:33.100", obs_ts=4034.3):
    payload = (
        f"source=gimbal_0;generation=0;task=1;obj=0;obs_ts={obs_ts};dt_wall_ms=;body_bearing_deg=89.0;"
        "cmd_roll=16.0;cmd_pitch=-3.0;cmd_thr=0.55;issued=True;missed=False"
    )
    return f"{time},EVENT:FINAL_APPROACH_CMD,{payload}\n"


def _docked(vehicle, aircraft_ts=4034.4):
    return (
        f"18:13:33.500,EVENT:DOCKED,vehicle={vehicle};task=1;dock=0;episode=1;aircraft_ts={aircraft_ts};"
        "closest_3d_m=0.19;closest_h_m=0.18;closest_v_m=0.02\n"
    )


SNAP = "18:13:34.000,EVENT:SNAP_COMPONENTS,algorithm=VISION-NAV-PN;dist_3d_m=1.0\n"


def _failed(vehicle, reason="source_command_stale"):
    return f"18:13:34.100,EVENT:DOCK_FAILED,vehicle={vehicle};task=1;dock=0;episode=1;reason={reason}\n"


def _debug(run_dir, uav, *rows):
    (run_dir / f"uav_{uav}_navigation_debug.csv").write_text(HEADER + CONFIG + "".join(rows), encoding="utf-8")


def _nav_log(run_dir, uav, *messages):
    lines = [f"2026-11-01 18:13:{index:02d},000 {message}" for index, message in enumerate(messages)]
    (run_dir / f"uav_{uav}_navigation.log").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _source(run_dir, name, text):
    directory = run_dir / "source-time"
    directory.mkdir(exist_ok=True)
    (directory / name).write_text(text, encoding="utf-8")


ARRIVAL = "wall_s,boot_ms\n1793542413.0,4034000\n1793542413.1,4034020\n1793542413.2,4034040\n"


def _flown_run(tmp_path):
    """UAV 4 fails on a stale command, UAV 5 docks, UAV 6 only flies."""
    run_dir = tmp_path / "20261101-180608"
    run_dir.mkdir()
    _debug(run_dir, 4, _armed(4), _command(), SNAP, _failed(4))
    _debug(run_dir, 5, _armed(5), _command(), _docked(5), SNAP)
    _debug(run_dir, 6)
    _nav_log(
        run_dir, 4, "INFO Heartbeat from system 4 received.", "INFO noise", "WARNING Mission is empty",
        "INFO Asking confirmation for: 1", "INFO POI 1 confirmed by ground station.",
        "INFO NAV_FAIL: source_command_stale task=1 obj=0",
    )
    for uav in (4, 5, 6):
        for producer in ("navpy", "gcs"):
            _source(run_dir, f"pose_cadence_attitude_arrival_uav_{uav}_{producer}.csv", ARRIVAL)
    _source(
        run_dir, "pose_cadence_worker_uav_4_navpy.csv",
        "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome,work_cpu_ms,epoch\n"
        "100.0,0.4,4034.3,4034.5,measured,fresh,0.0,0\n"
        "100.1,0.4,4034.4,4034.5,measured,held,0.0,0\n",
    )
    backend = run_dir / "gcs" / "backend.log"
    backend.parent.mkdir()
    backend.write_text(
        "2026-11-01 18:06:47 [gcs.backend.cpu_share_holder] INFO: CPU share a granted: CPUs [8, 9, 18, 19] for 1200 s\n"
        "2026-11-01 18:06:48 [gcs.backend.other] INFO: noise\n"
        "2026-11-01 18:06:49 [gcs.backend.other] WARNING: careful\n"
        "2026-11-01 18:12:54 [gcs.backend.routes.control] INFO: [launch] /launch called: sys_ids=[4, 5, 6], force=False\n"
        "Traceback (most recent call last):\n",
        encoding="utf-8",
    )
    return run_dir


def test_a_flown_run_reads_as_facts(tmp_path):
    text = check.report(_flown_run(tmp_path))

    assert "UAVs (from file names): 4, 5, 6" in text
    assert "Approaches: 2: 1 DOCKED, 1 DOCK_FAILED (source_command_stale)" in text
    assert "No approach ended in the debug log: UAV 6" in text
    assert "approach: episode 1 task 1 dock 0: DOCK_FAILED (source_command_stale), 1 command rows" in text
    assert "approach: episode 1 task 1 dock 0: DOCKED, closest 0.19 m (h 0.18, v 0.02), 1 command rows" in text
    assert "cadence: approach not entered" in text
    assert "Commit and inputs: unknown, no input_manifest.json" in text
    assert "CPUs [8, 9, 18, 19]" in text
    assert "/launch called: sys_ids=[4, 5, 6]" in text
    assert "1 WARNING, 1 tracebacks" in text
    # A fact, with no verdict of its own.
    assert "PASS" not in text


def test_log_lines_are_the_flights_story_and_its_warnings(tmp_path):
    lines = uav_check._log_lines(_flown_run(tmp_path), 4)

    assert lines[0] == "log: 6 lines, 1 NavPy starts, 1 WARNING, 0 ERROR"
    assert lines[1:] == [
        "  18:13:02 Mission is empty",
        "  18:13:03 Asking confirmation for: 1",
        "  18:13:04 POI 1 confirmed by ground station.",
        "  18:13:05 NAV_FAIL: source_command_stale task=1 obj=0",
    ]


def test_source_time_lines_give_each_stream_and_name_what_is_missing(tmp_path):
    run_dir = _flown_run(tmp_path)

    lines = uav_check._source_time_lines(run_dir, 4)
    assert any(line.startswith("attitude_arrival navpy: 3 rows, wall gap p50 100.0") for line in lines)
    assert any(line.startswith("worker navpy: 2 rows {'fresh': 1, 'held': 1}, source age") for line in lines)
    # It entered an approach, so its observation file should be there.
    assert "observation navpy: file is missing" in lines
    # UAV 6 entered none: its absent approach streams are not missing.
    assert not [line for line in uav_check._source_time_lines(run_dir, 6) if "missing" in line]


def test_a_failure_with_no_command_row_is_still_an_outcome(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _debug(run_dir, 2, _armed(2), _failed(2, "no_detection"))

    assert uav_check._approach_lines(run_dir, 2) == [
        "approach: episode 1 task 1 dock 0: DOCK_FAILED (no_detection), 0 command rows"
    ]


def test_an_episode_with_no_end_row_says_so(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _debug(run_dir, 2, _armed(2), _command(), SNAP)

    assert "no DOCKED or DOCK_FAILED row" in "\n".join(uav_check._approach_lines(run_dir, 2))


def test_uav_ids_come_from_file_names_of_both_folders(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "uav_12_navigation.log").write_text("", encoding="utf-8")
    (run_dir / "uav_notanumber_navigation.log").write_text("", encoding="utf-8")
    (run_dir / "uav_7_confirmation_t1_r1_meta.json").write_text("{}", encoding="utf-8")
    _source(run_dir, "pose_cadence_worker_uav_30_navpy.csv", "")
    _source(run_dir, "unrelated.csv", "")

    assert uav_check.uav_ids(run_dir) == [7, 12, 30]


def test_an_empty_folder_says_what_is_missing(tmp_path):
    text = check.report(tmp_path)

    assert "UAVs: none, no uav_N_* file in this folder" in text
    assert "no input_manifest.json" in text
    assert "gcs/backend.log is missing" in text
    assert "Timeline: no gcs/sessions/*/timeline.jsonl" in text


def test_a_uav_with_only_some_files_says_which_are_missing(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _source(run_dir, "pose_cadence_attitude_arrival_uav_3_navpy.csv", ARRIVAL)

    text = check.report(run_dir)

    assert "UAV 3" in text
    assert "log: uav_3_navigation.log is missing" in text
    assert "approaches: uav_3_navigation_debug.csv is missing" in text
    assert "sensor config: uav_3_navigation.log is missing" in text
    assert "attitude_arrival gcs: file is missing" in text
    assert "Approaches not read: UAV 3" in text


@pytest.mark.parametrize(
    ("content", "expected"),
    [
        ("wall_s,boot_ms\n1793542413.0,4034000\n1793542413.1\n", "incomplete CSV row"),  # killed mid-row
        ("", "no header"),
        ("wall_s\n1.0\n", "missing source-time columns"),
    ],
)
def test_one_partial_source_file_loses_only_its_own_line(tmp_path, content, expected):
    run_dir = _flown_run(tmp_path)
    _source(run_dir, "pose_cadence_attitude_arrival_uav_5_navpy.csv", content)

    text = check.report(run_dir)

    assert "attitude_arrival navpy: could not read:" in text and expected in text
    # Everything else still prints.
    assert "attitude_arrival gcs: 3 rows" in text
    assert "UAV 6" in text


def test_an_open_episode_and_a_broken_debug_file_do_not_stop_the_report(tmp_path):
    run_dir = _flown_run(tmp_path)
    _debug(run_dir, 4, _armed(4), _command())  # stopped in the approach
    (run_dir / "uav_5_navigation_debug.csv").write_bytes(b"\xff\xfe\x00broken")

    text = check.report(run_dir)

    assert "approach: could not read: final-approach episode did not terminate" in text
    assert "Approaches not read: UAV 4, 5" in text
    assert "cadence: episode 1, ended by log_end" in text  # the open approach still shows
    assert "UAV 6" in text


@pytest.mark.parametrize("content", ["", "ts,ev", "18:11:13.709,EVENT:CONFIG,schema=primary\n"])
def test_a_debug_file_without_its_header_is_not_read_as_no_approach(tmp_path, content):
    run_dir = _flown_run(tmp_path)
    (run_dir / "uav_5_navigation_debug.csv").write_text(content, encoding="utf-8")

    text = check.report(run_dir)

    assert "Approaches not read: UAV 5" in text
    assert "approach: could not read: uav_5_navigation_debug.csv has no header row" in text
    assert "cadence: could not read: uav_5_navigation_debug.csv has no header row" in text
    assert not any(uav == "5" for uav in _no_approach_uavs(text))


def _no_approach_uavs(text):
    for line in text.splitlines():
        if line.startswith("No approach ended in the debug log: UAV "):
            return line.rsplit("UAV ", 1)[1].split(", ")
    return []


def test_a_header_only_debug_file_is_a_run_with_no_approach(tmp_path):
    run_dir = _flown_run(tmp_path)
    (run_dir / "uav_5_navigation_debug.csv").write_text(HEADER, encoding="utf-8")

    text = check.report(run_dir)

    assert "5" in _no_approach_uavs(text)
    assert "Approaches not read" not in text


def test_a_partial_manifest_says_so_and_a_broken_one_cannot_stop_the_report(tmp_path):
    run_dir = _flown_run(tmp_path)
    (run_dir / "input_manifest.json").write_text('{"source": {"git_head": "abc"', encoding="utf-8")
    assert "Commit and inputs: could not read:" in check.report(run_dir)

    (run_dir / "input_manifest.json").write_text(json.dumps({"source": {"git_head": "abc"}}), encoding="utf-8")
    lines = check.input_lines(run_dir)
    assert lines[0] == "Commit: abc, uncommitted changes: not recorded, untracked files: not recorded"
    assert "settings not recorded" in lines[1]


def test_clean_and_changed_source_are_told_apart(tmp_path):
    empty = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    (tmp_path / "input_manifest.json").write_text(
        json.dumps({"source": {"git_head": "abc", "uncommitted_diff_sha256": empty, "untracked_sha256": "f" * 64}}),
        encoding="utf-8",
    )

    assert check.input_lines(tmp_path)[0] == (
        "Commit: abc, uncommitted changes: none, untracked files: yes (sha256 start ffffffffffff)"
    )


def test_a_launch_only_run_says_launch_was_never_called(tmp_path):
    run_dir = tmp_path / "run"
    (run_dir / "gcs").mkdir(parents=True)
    (run_dir / "gcs" / "backend.log").write_text("2026-11-01 18:06:48 [x] INFO: noise\n", encoding="utf-8")

    assert check.backend_lines(run_dir) == ["Backend: 1 log lines, no warnings, 0 tracebacks, /launch never called"]


def test_rotated_backend_text_is_read_oldest_first(tmp_path):
    run_dir = tmp_path / "run"
    (run_dir / "gcs").mkdir(parents=True)
    for name, text in (
        ("backend.log.10", "t [x] INFO: CPU share granted [cpu_share_holder] first\n"),
        ("backend.log.2", "t [x] INFO: /launch called: second\n"),
        ("backend.log", "t [x] WARNING: later\n"),
        ("backend.log.bak", "t [x] WARNING: not a rotated file\n"),
    ):
        (run_dir / "gcs" / name).write_text(text, encoding="utf-8")

    assert check.backend_lines(run_dir) == [
        "Backend: 3 log lines (with 2 rotated files), 1 WARNING, 0 tracebacks",
        "  t [x] INFO: CPU share granted [cpu_share_holder] first",
        "  t [x] INFO: /launch called: second",
    ]


def test_with_rotated_backend_text_a_missing_launch_is_not_called_never(tmp_path):
    run_dir = tmp_path / "run"
    (run_dir / "gcs").mkdir(parents=True)
    for name in ("backend.log.1", "backend.log"):
        (run_dir / "gcs" / name).write_text("t [x] INFO: noise\n", encoding="utf-8")

    assert check.backend_lines(run_dir) == [
        "Backend: 2 log lines (with 1 rotated files), no warnings, 0 tracebacks, no /launch call in the text kept"
    ]


def test_the_timeline_counts_events_bad_rows_and_failures(tmp_path):
    path = tmp_path / "gcs" / "sessions" / "demo-eval" / "timeline.jsonl"
    path.parent.mkdir(parents=True)
    rows = [
        {"event": "session_started"},
        {"event": "client_timeout"},
        {"event": "http_request_finished", "route": "/ok", "outcome": "success"},
        {"event": "http_request_finished", "route": "/a", "outcome": "failure"},
    ]
    path.write_text("\n".join(map(json.dumps, rows)) + '\n{"event": "cut', encoding="utf-8")

    assert check.timeline_lines(tmp_path) == ["Timeline: 4 events, 1 unreadable lines: 1 client_timeout, 1 failed /a"]


def test_rotated_log_text_is_named(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _nav_log(run_dir, 1, "INFO noise")
    (run_dir / "uav_1_navigation.log.1").write_text("older", encoding="utf-8")

    assert "log: older text not read: uav_1_navigation.log.1" in uav_check._log_lines(run_dir, 1)


def test_sensor_config_differences_are_listed_and_a_match_is_stated(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _nav_log(
        run_dir, 1, "INFO Using vision profile 'siyi_zr10' from x",
        "INFO Mount 'siyi_zr10': 1280x720, pitch=-14.0, fixed=False",
    )
    _nav_log(run_dir, 2, "INFO Using vision profile 'other' from x")

    assert uav_check._sensor_lines(run_dir, 1) == ["sensor config: as the eval's"]
    assert any("'other'" in line for line in uav_check._sensor_lines(run_dir, 2))


def test_one_broken_section_leaves_the_others_and_a_new_section_is_one_entry(monkeypatch, tmp_path):
    def broken(_run_dir):
        raise RuntimeError("boom")

    monkeypatch.setattr(
        check, "RUN_SECTIONS",
        (("Broken", broken), ("Recorder", lambda _run_dir: ["Recorder: a later step's line"]), *check.RUN_SECTIONS),
    )

    text = check.report(tmp_path)

    assert "Broken: could not read: boom" in text
    assert "Recorder: a later step's line" in text
    assert "UAVs: none" in text


def test_the_newest_run_is_the_largest_name_among_folders(tmp_path):
    for name in ("20261031-153704", "20261101-180608", "20261025-210113"):
        (tmp_path / name).mkdir()
    (tmp_path / "20991231-000000").write_text("a file, not a run", encoding="utf-8")

    assert check.newest_run(tmp_path).name == "20261101-180608"
    assert check.newest_run(tmp_path / "absent") is None
    assert check.newest_run(tmp_path / "20261025-210113") is None


def test_main_defaults_to_the_newest_run_of_this_checkout(monkeypatch, tmp_path, capsys):
    run_dir = tmp_path / "20261101-180608"
    run_dir.mkdir()
    monkeypatch.setattr(check, "RUNS_DIR", tmp_path)

    assert check.main([]) == 0
    assert f"Run folder: {run_dir}" in capsys.readouterr().out


def test_main_says_when_there_is_no_run_or_no_folder(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(check, "RUNS_DIR", tmp_path)
    assert check.main([]) == 2
    assert "No hand demo run folder" in capsys.readouterr().err

    assert check.main([str(tmp_path / "absent")]) == 2
    assert "Not a folder" in capsys.readouterr().err


def test_log_text_outside_the_console_code_page_cannot_stop_the_report(monkeypatch, tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _nav_log(run_dir, 1, "WARNING док ✓")
    raw = io.BytesIO()
    monkeypatch.setattr(sys, "stdout", io.TextIOWrapper(raw, encoding="cp1252", errors="strict"))

    assert check.main([str(run_dir)]) == 0

    sys.stdout.flush()
    assert b"\\u0434" in raw.getvalue()
