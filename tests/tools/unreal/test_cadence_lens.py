"""tools/unreal/cadence_lens.py: the lens row each capture had to render with, rebuilt from the emulator's steps, on
synthetic runs whose camera follows FNavPyLensSelector's rule (NavPyLensTableTest.cpp tests that rule in Unreal),
and the mutations a wrong camera actor would make."""
from __future__ import annotations

import dataclasses
import json
import hashlib

import pytest

from navpy.core.perception.camera.reading_intrinsics import ABSOLUTE_ZOOM, ZoomSetting
from tests.tools.unreal.test_cadence_check import BOARD, OFFSET_S, board_view_run, synthetic_run
from tools.unreal import cadence_lens as cl, cadence_log as clog, cadence_oracle as co
from tools.unreal.lens_fixtures import PROFILE

TRAVEL_S = (2.0, 3.0)          # after OFFSET_S: the lens travels from the start-up 1.1 to the command 2.0's reading
SECOND_TRAVEL_S = (4.0, 5.0)   # with two_travels: then on to the command 3.0's reading


def bound_line(**override: str) -> dict:
    line = {"run": "run1", "path": str(PROFILE), "sha1": hashlib.sha1(PROFILE.read_bytes()).hexdigest().upper(),
            "profile": "siyi_zr10", "device": "siyi_zr10", "stream": "main", "size": "1920x1080"}
    return {**line, **override}


def row_after(command_tenths: int):
    """The table's row of the reading a command settles at."""
    setting = ZoomSetting(ABSOLUTE_ZOOM, command_tenths)
    return next(row for reading in TABLE.readings(STREAM)
                if (row := TABLE.optics(STREAM, float(reading), float(reading), setting)) is not None)


TABLE, STREAM = cl.load_bound_table(bound_line(), PROFILE)
START, TWO, THREE = row_after(10), row_after(20), row_after(30)


def _travel(t: float, span: tuple[float, float], start: float, end: float) -> float:
    return round(start + (end - start) * (t - span[0]) / (span[1] - span[0]), 1)


def lens_at(t_s: float, two_travels: bool = False) -> tuple[float, float, bool]:
    """The emulator's lens (reading, command, settled) at a step time."""
    t = t_s - OFFSET_S
    two, three = float(TWO.reading), float(THREE.reading)
    if t < TRAVEL_S[0]:
        return 1.1, 1.0, True
    if t < TRAVEL_S[1]:
        return _travel(t, TRAVEL_S, 1.1, two), 2.0, False
    if not two_travels or t < SECOND_TRAVEL_S[0]:
        return two, 2.0, True
    if t < SECOND_TRAVEL_S[1]:
        return _travel(t, SECOND_TRAVEL_S, two, three), 3.0, False
    return three, 3.0, True


def actor_labels(log, steps, streams, keep_old_history: bool = False) -> None:
    """Fill each capture's gimbal bracket and lens fields as the camera actor does (the history of the session it
    holds at the capture; a new session forgets the row); a capture with no row becomes a missed:no-lens slot.
    ``keep_old_history`` is the bug of an actor that never clears its history on a new session."""
    times = [s.t for s in steps]
    remembered, held_session = None, None
    for s in log.slots:
        capture_qpc = OFFSET_S + int(s["board_us"]) * 1e-6 + 0.04
        session = cl._session_at(steps, times, capture_qpc)
        if session != held_session and not keep_old_history:
            remembered, held_session = None, session
        own = steps if keep_old_history else [step for step in steps if step.session == session]
        offset = streams[int(s["gen"])].clock_offset(int(s["map_n"]), int(s["board_us"]))
        b = None if offset is None else co.gimbal_bracket(own, [x.t for x in own], int(s["board_us"]) * 1e-6 + offset)
        logged_session = b.newer.session if keep_old_history and b is not None else session
        s.update(capture_qpc=repr(capture_qpc), gimbal_session=logged_session,
                 gimbal_seq="0,0" if b is None else f"{b.older.seq},{b.newer.seq}",
                 gimbal_w="0.000000" if b is None else f"{b.weight:.6f}",
                 gimbal_valid="1" if b is not None and b.valid else "0")
        older, newer = (None, None) if b is None else (cl.step_lens(b.older), cl.step_lens(b.newer))
        if b is not None and b.valid and older is not None and older == newer:
            remembered, status = TABLE.optics(STREAM, older[0] / 10, older[0] / 10, older[1]), "settled"
        else:
            status = "held"
        if remembered is None:
            for key in ("frame", "board_us", "crc"):
                s.pop(key, None)
            s["outcome"] = cl.NO_LENS
            continue
        s.update(lens=status, lens_reading=remembered.reading, lens_set_by=cl.setting_text(remembered.setting),
                 lens_basis=remembered.basis)


def lens_run(two_travels: bool = False, session_from_s: float | None = None, keep_old_history: bool = False):
    """A synthetic run whose camera follows the actor's rule; ``session_from_s`` (after OFFSET_S) starts gimbal
    session s2 there."""
    log, frames, steps = synthetic_run(seconds=7.0 if two_travels else 6.0)
    steps = [dataclasses.replace(s, zoom=z, lens_command=c, lens_settled=settled,
                                 session="s2" if session_from_s is not None and s.t - OFFSET_S >= session_from_s
                                 else "s1")
             for s in steps for z, c, settled in [lens_at(s.t, two_travels)]]
    streams = co.stream_sources(log, log.mounts["run1"])
    for s in log.slots[:3]:
        s["map_n"] = "5"                     # the clock map is not ready: an invalid query, nothing to remember
    actor_labels(log, steps, streams, keep_old_history)
    corners = [[i, 0.0, 0.0] for i in range(co.MIN_REPROJECTION_CORNERS)]
    recorded = {int(f["frame"]): {**f, "corners": corners} for f in frames}
    return log, steps, streams, recorded


def check(log, steps, streams, recorded):
    return cl.check_lens(log, steps, streams, TABLE, STREAM, recorded)


def captures(log, status=None):
    return [s for s in log.slots if "frame" in s and (status is None or s["lens"] == status)]


def test_a_clean_run_settles_holds_through_travel_and_settles_again():
    log, steps, streams, recorded = lens_run()
    report, expected = check(log, steps, streams, recorded)
    assert report["pass"], report
    assert report["no_lens"] == 3 and report["bad_no_lens"] == 0         # until the clock map has 10 poses
    held = captures(log, "held")
    assert held and all(s["lens_reading"] == "1.1" for s in held)        # travel renders the remembered row
    assert {s["lens_reading"] for s in captures(log, "settled")} == {"1.1", TWO.reading}
    assert set(report["settled_frames_by_setting"]) == {"1.1/absolute_zoom:1.0", f"{TWO.reading}/absolute_zoom:2.0"}
    assert {status for _row, status, _interval in expected.values()} == {"settled", "held"}
    assert report["held_frames_by_interval"] == {1: report["held_frames"]}


def _relabel(records, row, status):
    for s in records:
        s.update(lens=status, lens_reading=row.reading, lens_set_by=cl.setting_text(row.setting), lens_basis=row.basis)


def test_an_actor_that_never_promotes_fails():
    log, steps, streams, recorded = lens_run()
    _relabel([s for s in captures(log) if s["lens_reading"] == TWO.reading], START, "held")
    assert check(log, steps, streams, recorded)[0]["label_mismatch"] > 0


def test_a_wrong_setting_at_the_same_reading_fails():
    log, steps, streams, recorded = lens_run()
    other = TABLE.optics(STREAM, 1.1, 1.1, ZoomSetting(ABSOLUTE_ZOOM, 12))
    assert other is not None and other.reading == START.reading
    _relabel(captures(log, "settled")[:5], other, "settled")
    report, _ = check(log, steps, streams, recorded)
    assert report["label_mismatch"] == 5 and not report["pass"]


def test_a_premature_switch_in_travel_fails():
    log, steps, streams, recorded = lens_run()
    _relabel(captures(log, "held")[:3], TWO, "settled")
    report, _ = check(log, steps, streams, recorded)
    assert report["label_mismatch"] == 3 and not report["pass"]


def test_wrong_bracket_sequences_fail_even_with_consistent_labels():
    # An actor that queried an earlier, settled bracket: the angles of this run would not show it, the sequences do.
    log, steps, streams, recorded = lens_run()
    for s in captures(log, "held")[:4]:
        older, newer = (int(v) for v in s["gimbal_seq"].split(","))
        s["gimbal_seq"] = f"{older - 50},{newer - 50}"
    report, _ = check(log, steps, streams, recorded)
    assert report["bracket_mismatch"] == 4 and report["label_mismatch"] == 0 and not report["pass"]


def test_a_no_lens_miss_while_a_row_is_remembered_fails():
    log, steps, streams, recorded = lens_run()
    s = captures(log, "settled")[-1]
    for key in ("frame", "board_us", "crc"):
        s.pop(key)
    s["outcome"] = cl.NO_LENS
    assert check(log, steps, streams, recorded)[0]["bad_no_lens"] == 1


def test_missing_coverage_fails():
    log, steps, streams, recorded = lens_run()
    for s in captures(log, "held"):
        recorded[int(s["frame"])]["corners"] = []
    report, _ = check(log, steps, streams, recorded)
    assert report["held_frames"] == 0 and not report["pass"]
    log, steps, streams, recorded = lens_run()
    for s in captures(log, "settled"):
        if s["lens_reading"] == TWO.reading:
            recorded[int(s["frame"])]["corners"] = None
    report, _ = check(log, steps, streams, recorded)
    assert report["settled_frames_by_setting"][f"{TWO.reading}/absolute_zoom:2.0"] == 0 and not report["pass"]


def test_a_travel_never_seen_fails_even_with_enough_held_frames_in_all():
    log, steps, streams, recorded = lens_run(two_travels=True)
    report, expected = check(log, steps, streams, recorded)
    assert report["pass"] and set(report["held_frames_by_interval"]) == {1, 2}
    for frame, (_row, status, interval) in expected.items():
        if status == "held" and interval == 2:
            recorded[frame]["corners"] = []              # say the second travel turned the board out of view
    report, _ = check(log, steps, streams, recorded)
    assert report["held_frames_by_interval"][2] == 0 and report["held_frames"] >= cl.MIN_LENS_FRAMES
    assert not report["pass"]


def test_a_travel_without_any_capture_fails():
    # Every slot of the second travel missed (late ticks): no held capture there, yet the travel must be seen.
    log, steps, streams, recorded = lens_run(two_travels=True)
    _report, expected = check(log, steps, streams, recorded)
    for s in captures(log):
        if expected.get(int(s["frame"]), (None, "", 0))[1:] == ("held", 2):
            for key in ("frame", "board_us", "crc"):
                s.pop(key)
            s["outcome"] = "missed:late-tick"
    report, _ = check(log, steps, streams, recorded)
    assert report["held_frames_by_interval"][2] == 0 and report["label_mismatch"] == 0 and not report["pass"]
    # Held frames after the travel do not stand in for it: the next captures lose their clock map, so they hold
    # the remembered row with an invalid query, correctly labelled.
    after = [s for s in captures(log) if float(s["capture_qpc"]) - OFFSET_S > SECOND_TRAVEL_S[1]][:3]
    for s in after:
        s.update(map_n="5", gimbal_seq="0,0", gimbal_w="0.000000", gimbal_valid="0", lens="held",
                 lens_reading=TWO.reading, lens_set_by=cl.setting_text(TWO.setting), lens_basis=TWO.basis)
    report, _ = check(log, steps, streams, recorded)
    assert report["label_mismatch"] == 0 and report["bracket_mismatch"] == 0, report
    assert report["held_frames_by_interval"][2] == 0 and not report["pass"]


def test_after_a_session_change_the_old_sessions_bracket_fails():
    # Session s2 starts in a settled stretch. The actor's history then holds s2 only: a capture whose query falls
    # before s2's first step is invalid and nothing is remembered. An actor that kept s1's cleared history renders
    # those captures from a valid s1 bracket with the same angles and lens; that must fail.
    start = TRAVEL_S[1] + 0.5
    report, _ = check(*lens_run(session_from_s=start))
    assert report["pass"], report
    report, _ = check(*lens_run(session_from_s=start, keep_old_history=True))
    assert report["bracket_mismatch"] > 0 and not report["pass"]


def test_a_short_wrong_travel_fails_its_own_interval():
    # Two travels hold the same row; the second, a few frames, was rendered with another K. Pooled with the first
    # travel it would pass; each interval is its own group.
    log, frames, _camera = board_view_run(lag_frames=0)
    wide, narrow = cl.row_camera(START, STREAM), cl.row_camera(TWO, STREAM)
    first, second = set(range(20, 80)), {90, 91}
    corners = co.board_corners_world(BOARD, 1.0, -1.0)
    for s in log.slots:
        frame = int(s["frame"])
        px = co.project(corners, clog.parse_vec(s["cam_loc"]), clog.parse_vec(s["cam_quat"]),
                        narrow if frame in second else wide)
        frames[frame]["corners"] = [[i, float(u), float(v)] for i, (u, v) in enumerate(px)
                                    if 0 <= u < STREAM.image_width and 0 <= v < STREAM.image_height]
    held = first | second
    log.slots = [s for s in log.slots if int(s["frame"]) in held]
    pooled = co.check_reprojection(log, log.slots, frames, lambda s: wide, lambda s: "held")
    split = co.check_reprojection(log, log.slots, frames, lambda s: wide,
                                  lambda s: cl.lens_group(START, "held", 2 if int(s["frame"]) in second else 1))
    assert pooled["groups"]["held"]["p95_px"] <= co.REPROJECTION_P95_PX
    assert not split["pass"] and split["groups"][cl.lens_group(START, "held", 2)]["p95_px"] > co.REPROJECTION_P95_PX


def test_a_settled_state_without_a_row_fails():
    log, steps, streams, recorded = lens_run()
    steps = [dataclasses.replace(s, lens_command=9.9) if s.t - OFFSET_S > TRAVEL_S[1] + 1.0 else s for s in steps]
    report, _ = check(log, steps, streams, recorded)
    assert report["settled_without_row"] and not report["pass"]


def test_the_bound_table_must_be_the_profile_the_run_loaded():
    with pytest.raises(ValueError, match="SHA-1"):
        cl.load_bound_table(bound_line(sha1="0" * 40), PROFILE)
    with pytest.raises(ValueError, match="stream"):
        cl.load_bound_table(bound_line(size="1280x720"), PROFILE)


def test_a_held_frame_rendered_with_another_projection_fails_the_reprojection():
    # The labels say held 1.1; the picture was rendered with the new row's K (the actor rebuilt the projection).
    log, frames, camera = board_view_run(lag_frames=0)
    wide, narrow = cl.row_camera(START, STREAM), cl.row_camera(TWO, STREAM)
    held = set(range(40, 80))
    corners = co.board_corners_world(BOARD, 1.0, -1.0)

    def detect(rendered_with):
        for s in log.slots:
            frame = int(s["frame"])
            px = co.project(corners, clog.parse_vec(s["cam_loc"]), clog.parse_vec(s["cam_quat"]), rendered_with(frame))
            frames[frame]["corners"] = [[i, float(u), float(v)] for i, (u, v) in enumerate(px)
                                        if 0 <= u < STREAM.image_width and 0 <= v < STREAM.image_height]

    status = {int(s["frame"]): "held" if int(s["frame"]) in held else "settled" for s in log.slots}
    expected = lambda s: wide                                   # every frame had to render with the remembered 1.1
    detect(lambda _f: wide)
    good = co.check_reprojection(log, log.slots, frames, expected, lambda s: status[int(s["frame"])])
    assert good["pass"] and good["groups"]["held"]["p95_px"] < 1e-6
    detect(lambda f: narrow if f in held else wide)
    bad = co.check_reprojection(log, log.slots, frames, expected, lambda s: status[int(s["frame"])])
    assert not bad["pass"] and bad["groups"]["held"]["p95_px"] > 10 * co.REPROJECTION_P95_PX
    assert bad["groups"]["settled"]["p95_px"] < 1e-6


def test_a_wrong_row_on_one_small_setting_fails_its_own_group():
    # A few frames had to render with the 2.0 row but were rendered with the 1.1 row's K. They hold under 3 % of the
    # corners, too few to move a pooled p95, so each (status, row) group is gated on its own.
    log, frames, _camera = board_view_run(lag_frames=0)
    wide, narrow = cl.row_camera(START, STREAM), cl.row_camera(TWO, STREAM)
    corners = co.board_corners_world(BOARD, 1.0, -1.0)
    for s in log.slots:
        px = co.project(corners, clog.parse_vec(s["cam_loc"]), clog.parse_vec(s["cam_quat"]), wide)
        frames[int(s["frame"])]["corners"] = [[i, float(u), float(v)] for i, (u, v) in enumerate(px)
                                              if 0 <= u < STREAM.image_width and 0 <= v < STREAM.image_height]
    usable = sorted((len(f["corners"]), n) for n, f in frames.items()
                    if len(f["corners"]) >= co.MIN_REPROJECTION_CORNERS)
    total, small, taken = sum(c for c, _n in usable), set(), 0
    for count, frame in usable:
        if taken + count > 0.03 * total:
            break
        small.add(frame)
        taken += count
    assert small
    row_of = {int(s["frame"]): TWO if int(s["frame"]) in small else START for s in log.slots}
    report = co.check_reprojection(log, log.slots, frames, lambda s: narrow if row_of[int(s["frame"])] is TWO else wide,
                                   lambda s: cl.lens_group(row_of[int(s["frame"])], "settled"))
    assert report["p95_px"] <= co.REPROJECTION_P95_PX                    # pooled, the wrong setting would hide
    assert not report["pass"]
    assert report["groups"][cl.lens_group(TWO, "settled")]["p95_px"] > co.REPROJECTION_P95_PX
    assert report["groups"][cl.lens_group(START, "settled")]["p95_px"] < 1e-6


def test_parse_the_bound_table_line_and_the_emulator_lens():
    log = clog.parse_log([
        "[2026.10.23-10.00.00:000][  1]LogTemp: Display: NavPyLensTable sys_id=1 run=abc path=C:/p.json sha1=AB12 "
        "profile=siyi_zr10 device=siyi_zr10 stream=main size=1920x1080 rows=92",
        "[2026.10.23-10.00.00:000][  1]LogTemp: Error: NavPyLensTable refused: no full-frame stream",
    ])
    assert log.lens_tables == {"abc": {"sys_id": "1", "run": "abc", "path": "C:/p.json", "sha1": "AB12",
                                       "profile": "siyi_zr10", "device": "siyi_zr10", "stream": "main",
                                       "size": "1920x1080", "rows": "92"}}
    true = {"yaw": 0, "pitch": 0, "roll": 0, "zoom": 1.8, "motion_mode": 2, "lens_command": 2.0, "lens_settled": False}
    steps = clog.parse_emulator_log([json.dumps({"event": "start", "session_id": "s9"}),
                                     json.dumps({"event": "step", "t": 1.0, "seq": 1, "true": true})])
    assert (steps[0].zoom, steps[0].lens_command, steps[0].lens_settled, steps[0].session) == (1.8, 2.0, False, "s9")
    assert cl.step_lens(steps[0]) is None                                 # travelling
    assert cl.step_lens(dataclasses.replace(steps[0], lens_settled=True)) == (18, ZoomSetting(ABSOLUTE_ZOOM, 20))
    assert cl.step_lens(dataclasses.replace(steps[0], lens_settled=True, lens_command=None)) is None   # manual
