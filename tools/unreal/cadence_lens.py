"""The capture-cadence check's lens part (HITL stage 1 step 5): which reading_intrinsics row each capture had to
render with, rebuilt from the emulator's steps without the camera actor's own query or labels.

For every capture with a record (sent or lost), in capture order (generation, slot): the gimbal session the actor
held at the capture (the newest emulator step's; a new session clears the plugin's history), the gimbal query
instant from the oracle's clock map (the poses the receiver admitted, as at the capture), the steps of that session
FNavPyGimbalHistory uses at that instant (sequences, weight, validity), which the logged bracket must equal; then
FNavPyLensSelector's rule on those steps' lens states gives the expected row and status (settled or held), which the
logged row and status must equal. A missed:no-lens slot is allowed only while no row is remembered in the session.
Each lens travel of the emulator (a run of steps without a settled, known lens) inside the captured span is its own
interval and must be seen with the board in view, by captures whose mapped query falls inside it, whatever the
camera captured then. The reprojection
then scores every frame, settled and held, with the expected row's K (``camera_for``), in groups of one settled
row or one travel interval, so a frame rendered with another projection fails even when its labels are right.
Scoring only: nothing here reaches navpy.
"""
from __future__ import annotations

import bisect
import hashlib
from pathlib import Path
from typing import Optional, Sequence

from navpy.core.perception.camera.reading_intrinsics import (ABSOLUTE_ZOOM, WIDE_STOP, ReadingIntrinsicsTable,
                                                             ReadingOptics, StreamMode, ZoomSetting, zoom_tenths)
from tools.unreal.cadence_log import GimbalStep, UnrealLog
from tools.unreal.cadence_oracle import MIN_REPROJECTION_CORNERS, GimbalBracket, StreamSources, gimbal_bracket
from tools.unreal.dock_pass_geometry import CameraModel
from tools.unreal.lens_fixtures import load_section

WEIGHT_TOLERANCE = 2.0e-6     # the plugin logs gimbal_w with 6 decimals
MIN_LENS_FRAMES = 10          # frames with enough detected corners per settled setting, and held frames in all
# Frames with enough detected corners in every travel interval: the interval is seen at all. The shortest travel of
# the live schedule (one level at 2 levels/s, 0.5 s) gives about 6 frames at the 13 fps a loaded laptop renders.
MIN_INTERVAL_FRAMES = 3
NO_LENS = "missed:no-lens"
LensState = tuple[int, ZoomSetting]
Expected = tuple[ReadingOptics, str, int]     # row, status, travel interval (0 when settled)


def setting_text(setting: ZoomSetting) -> str:
    """A setting as FNavPyLensSetting::ToString logs it: wide_stop or absolute_zoom:2.0."""
    if setting.kind == WIDE_STOP:
        return WIDE_STOP
    return f"{ABSOLUTE_ZOOM}:{setting.command_tenths // 10}.{setting.command_tenths % 10}"


def step_lens(step: Optional[GimbalStep]) -> Optional[LensState]:
    """An emulator step's lens as a row key when it is settled and set by a known command
    (FNavPyLensState::IsSettledKnown), else None."""
    if step is None or not step.lens_settled or step.lens_command is None:
        return None
    reading, command = zoom_tenths(step.zoom), zoom_tenths(step.lens_command)
    if reading is None or command is None:
        return None
    return reading, ZoomSetting(ABSOLUTE_ZOOM, command)


class LensSelector:
    """FNavPyLensSelector's rule: a row is settled only for a valid query whose steps carry one settled, known lens
    state that has a row, and is then remembered; anything else renders the remembered row as held; nothing
    remembered gives none. A new session forgets the row."""

    def __init__(self, table: ReadingIntrinsicsTable, stream: StreamMode) -> None:
        self.table, self.stream = table, stream
        self.remembered: Optional[ReadingOptics] = None
        self.session: Optional[str] = None

    def select(self, session: str, valid: bool, older: Optional[GimbalStep],
               newer: Optional[GimbalStep]) -> tuple[Optional[ReadingOptics], str]:
        if session != self.session:
            self.remembered, self.session = None, None
        a, b = step_lens(older), step_lens(newer)
        row = None
        if valid and a is not None and a == b:
            row = self.table.optics(self.stream, a[0] / 10, a[0] / 10, a[1])
        if row is not None:
            self.remembered, self.session = row, session
            return row, "settled"
        return (self.remembered, "held") if self.remembered is not None else (None, "none")


def load_bound_table(bound: dict, profile_path: Path) -> tuple[ReadingIntrinsicsTable, StreamMode]:
    """The table a NavPyLensTable line says the actor bound, from ``profile_path``, which must hash as it logged."""
    sha1 = hashlib.sha1(profile_path.read_bytes()).hexdigest()
    if sha1.lower() != bound["sha1"].lower():
        raise ValueError(f"{profile_path} has SHA-1 {sha1}, the run bound {bound['sha1']} ({bound['path']})")
    table = ReadingIntrinsicsTable.from_profile(load_section(profile_path, bound["profile"], bound["device"]))
    width, height = (int(v) for v in bound["size"].split("x"))
    stream = StreamMode(bound["stream"], width, height, (0, 0, width, height))
    if stream not in table.streams():
        raise ValueError(f"no full-frame stream {stream} in the table")
    return table, stream


def lens_group(row: ReadingOptics, status: str, interval: int = 0) -> str:
    """The reprojection group of a frame: its settled row, or its travel interval and held row, so one wrong
    setting or one wrong travel cannot hide in a pooled p95."""
    where = f"held#{interval}" if status == "held" else status
    return f"{where}:{row.reading}/{setting_text(row.setting)}"


def row_camera(row: ReadingOptics, stream: StreamMode) -> CameraModel:
    return CameraModel(f"{row.reading}/{setting_text(row.setting)}", stream.image_width, stream.image_height,
                       row.fx, row.fy, row.cx, row.cy, row.skew)


def _session_at(steps: Sequence[GimbalStep], times: Sequence[float], qpc_s: float) -> str:
    """The session of the newest step at a PC instant: the history the actor held when it captured."""
    return steps[max(0, bisect.bisect_right(times, qpc_s) - 1)].session if steps else ""


def travels(steps: Sequence[GimbalStep]) -> list[tuple[float, float]]:
    """The emulator's lens travels as [start, end): from the step leaving a settled, known state (or the first step,
    without one) to the next step with one (inf if none)."""
    out: list[tuple[float, float]] = []
    start = None
    for i, step in enumerate(steps):
        if step_lens(step) is None:
            if start is None:
                start = step.t
        elif start is not None:
            out.append((start, step.t))
            start = None
    if start is not None:
        out.append((start, float("inf")))
    return out


def _travel_of(spans: Sequence[tuple[float, float]], query: float) -> Optional[int]:
    """The 1-based travel whose span holds the query instant, else None."""
    k = bisect.bisect_right([start for start, _end in spans], query)
    return k if k and query < spans[k - 1][1] else None


def _by_session(steps: Sequence[GimbalStep]) -> dict[str, tuple[list[GimbalStep], list[float]]]:
    """Each session's steps and times: the history the plugin keeps after a session change."""
    out: dict[str, tuple[list[GimbalStep], list[float]]] = {}
    for step in steps:
        own, times = out.setdefault(step.session, ([], []))
        own.append(step)
        times.append(step.t)
    return out


def _bracket_ok(record: dict, session: str, bracket: Optional[GimbalBracket], mapped: bool) -> bool:
    if not mapped or bracket is None:
        return record["gimbal_valid"] == "0"
    older, newer = (int(v) for v in record["gimbal_seq"].split(","))
    return (record["gimbal_session"] == session and (older, newer) == (bracket.older.seq, bracket.newer.seq)
            and record["gimbal_valid"] == ("1" if bracket.valid else "0")
            and abs(float(record["gimbal_w"]) - bracket.weight) <= WEIGHT_TOLERANCE)


def _labels_ok(record: dict, row: Optional[ReadingOptics], status: str) -> bool:
    if row is None:
        return False                         # a capture record means the actor had a row
    return (record["lens"] == status and record["lens_reading"] == row.reading
            and record["lens_set_by"] == setting_text(row.setting) and record["lens_basis"] == row.basis)


def check_lens(log: UnrealLog, steps: Sequence[GimbalStep], streams: dict[int, StreamSources],
               table: ReadingIntrinsicsTable, stream: StreamMode,
               recorded: dict[int, dict]) -> tuple[dict, dict[int, Expected]]:
    """The lens report and, by frame, the row, status and travel interval each captured frame had to render with."""
    times = [s.t for s in steps]
    sessions = _by_session(steps)
    selector = LensSelector(table, stream)
    expected: dict[int, Expected] = {}
    bracket_mismatch = label_mismatch = bad_no_lens = pending_no_lens = 0
    statuses = {"settled": 0, "held": 0}
    spans = travels(steps)
    starts = [start for start, _end in spans]
    span: list[float] = []
    seen_in_travel: dict[int, int] = {}      # frame: the travel its mapped query fell in
    for s in sorted(log.slots, key=lambda r: (int(r["gen"]), int(r["slot"]))):
        if s["outcome"] == NO_LENS:
            pending_no_lens += 1
            continue
        if "frame" not in s:
            continue
        board_us, gen = int(s["board_us"]), int(s["gen"])
        source = streams.get(gen)
        offset = None if source is None else source.clock_offset(int(s["map_n"]), board_us)
        session = _session_at(steps, times, float(s["capture_qpc"]))
        own, own_times = sessions.get(session, ([], []))
        query = float(s["capture_qpc"]) if offset is None else board_us * 1e-6 + offset
        span.append(query)
        bracket = None if offset is None else gimbal_bracket(own, own_times, query)
        # No-lens slots before this capture were right only if the actor had no row in this session then.
        if selector.remembered is not None and selector.session == session:
            bad_no_lens += pending_no_lens
        pending_no_lens = 0
        bracket_mismatch += not _bracket_ok(s, session, bracket, offset is not None)
        valid = bracket is not None and bracket.valid
        row, status = selector.select(session, valid, bracket and bracket.older, bracket and bracket.newer)
        label_mismatch += not _labels_ok(s, row, status)
        if row is not None:
            statuses[status] += 1
            if s["outcome"] == "sent":
                travel = bisect.bisect_right(starts, query) if status == "held" else 0
                expected[int(s["frame"])] = (row, status, travel)
                inside = None if offset is None else _travel_of(spans, query)
                if status == "held" and inside is not None:
                    seen_in_travel[int(s["frame"])] = inside
    if selector.remembered is not None:
        bad_no_lens += pending_no_lens

    def usable(frame: int) -> bool:
        return len(recorded.get(frame, {}).get("corners") or []) >= MIN_REPROJECTION_CORNERS

    settled_states = {key for step in steps if (key := step_lens(step)) is not None}
    without_row = sorted(f"{r / 10:.1f}/{setting_text(z)}" for r, z in settled_states
                         if table.optics(stream, r / 10, r / 10, z) is None)
    per_setting = {f"{r / 10:.1f}/{setting_text(z)}": 0 for r, z in settled_states}
    held_usable = 0
    # Every travel that began inside the captured span, captured or not; only frames queried inside it count.
    per_interval = {n: 0 for n, t in enumerate(starts, 1) if span and min(span) <= t <= max(span)}
    for frame, (row, status, _travel) in expected.items():
        if usable(frame):
            if status == "settled":
                per_setting[f"{row.reading}/{setting_text(row.setting)}"] += 1
            else:
                held_usable += 1
                if seen_in_travel.get(frame) in per_interval:
                    per_interval[seen_in_travel[frame]] += 1
    report = {"statuses": statuses, "no_lens": sum(1 for s in log.slots if s["outcome"] == NO_LENS),
              "bad_no_lens": bad_no_lens, "bracket_mismatch": bracket_mismatch, "label_mismatch": label_mismatch,
              "settled_without_row": without_row, "settled_frames_by_setting": per_setting,
              "held_frames": held_usable, "held_frames_by_interval": per_interval}
    report["pass"] = (bool(expected) and bad_no_lens == 0 and bracket_mismatch == 0 and label_mismatch == 0
                      and not without_row and held_usable >= MIN_LENS_FRAMES
                      and all(n >= MIN_LENS_FRAMES for n in per_setting.values())
                      and all(n >= MIN_INTERVAL_FRAMES for n in per_interval.values()))
    return report, expected
