r"""Offline check of the Unreal native camera's board-time capture cadence (HITL stage 1 step 4).

Two commands:

  record   listen like navpy (the Unreal camera connects), keep every NPYC frame's header, the CRC32 of its
           pixels and the ChArUco corners detected in it (JSONL, one line per frame; no pixels stored);
  check    join that recording with Unreal's log and the ZR10 emulator's --log and score the run.

  .venv\Scripts\python.exe -m tools.unreal.cadence_check record --out C:\tune_hitl\cadence\frames.jsonl --seconds 660
  .venv\Scripts\python.exe -m tools.unreal.cadence_check check --frames C:\tune_hitl\cadence\frames.jsonl ^
      --unreal-log ...\AasSimProject.log --emulator-log C:\tune_hitl\cadence\zr10.jsonl --out C:\tune_hitl\cadence\report.json

Unreal runs with -NavPyNativeCameraBoardTimeCadence=1 -NavPyInterpolatedVehiclePose=1
-NavPyNativeCameraHeaderVersion=2 -NavPyDockCharucoBoard=1 and either -NavPyLensProfile=<vision_profiles.json>
(the lens table, step 5: a zooming lens, scored per frame with its row) or -NavPyNativeCameraIntrinsics=1 (fixed
zoom-1 K at 1280 x 720: the lens must not move). One camera run per recording (one Unreal session, one camera
connection).

What is checked (step 4's done list, and the design's rendered-state identity):
- identity: one camera run (reconnects of it allowed); no frame index twice in the log (sent and lost) or the
  recording;
- joins: every recorded frame matches one NavPyCaptureSlot "sent" record by frame index, with the same board time and
  pixel CRC; a sent frame may be missing only as the one frame in flight when a recorder connection closed (at most
  one per connection change, numbered between the last frame of that connection and the first lost:disconnect
  frame before the next);
- grid: every captured slot's board time is origin + round(slot * 1e6 / F), one origin per stream;
- coverage: per stream generation, sent + lost + missed slots are exactly 0 .. next_slot - 1 of the receiver's
  stream_end line, each once; every generation the receiver started needs its stream_end line;
- unique rate: sent frames per second of represented board time, per stream, within 2 % of the cadence;
- repeated pictures: no equal pixel CRC on frames of different board times whose camera moved;
- oracle, independent of the plugin's own clock map and gimbal query: the board clock map is rebuilt from the
  receiver's NavPyPoseAdmit lines (the poses it admitted, with their receipt times) as at the capture, the gimbal
  query instant is the slot's board time through that map, and the camera transform is rebuilt from those poses
  interpolated at the slot's board time, the emulator's true gimbal angles at that instant and the NavPyCameraMount
  offsets; it must equal the logged transform within 0.01 deg / 1 mm, with valid pose and gimbal brackets, after a
  2 s warm-up per stream;
- lens (step 5, cadence_lens.py): the gimbal bracket of every capture rebuilt from the oracle's query instant and
  the emulator's steps must equal the logged one, and the row and status (settled / held) FNavPyLensSelector's rule
  gives on those steps' lens states must equal the logged ones; no-lens misses only while nothing is remembered;
  every settled lens state has a row; >= 10 frames with the board at every settled setting and held in travel;
- reprojection: the board corners projected with each frame's logged camera and the K it had to render with (its
  expected row, settled or held; the fixed K) fit the detected corners (p95 of the corner residuals <= 2 px, also
  per lens status), better than the neighbouring frames' cameras do where the camera moved between them.

Scoring only: nothing here reaches navpy's navigation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable, Optional, Sequence

from tools.unreal.cadence_lens import check_lens, lens_group, load_bound_table, row_camera
from tools.unreal.cadence_log import GimbalStep, UnrealLog, parse_emulator_log, parse_log
from tools.unreal.cadence_oracle import (ANGLE_TOLERANCE_DEG, LOCATION_TOLERANCE_CM, record_transform, check_oracle,
                                         check_reprojection, stream_sources)
from tools.unreal.cadence_record import DEFAULT_LISTEN, parse_endpoint, record
from tools.unreal.dock_pass_geometry import zr10_camera
from tools.unreal.lens_fixtures import PROFILE
from tools.unreal.pose_stamp_log import _angle_deg as quat_angle_deg
from tools.unreal.unreal_math import slot_time_us

UNIQUE_RATE_TOLERANCE = 0.02          # step 4: unique rate within 2 % of F


# --- check ---------------------------------------------------------------------------------------------------------
def _slot_range(record: dict) -> range:
    first = int(record["slot"])
    return range(first, first + int(record.get("count", "1")))


def in_flight_at_close(sent_frames: Iterable[int], frames: Sequence[dict],
                       disconnect_lost: Iterable[int]) -> tuple[list[int], list[int]]:
    """Sent frames the recording lacks, split into the accepted ones and the rest. Accepted: the single frame in
    flight at a connection change, numbered between one connection's last frame and the camera's first
    lost:disconnect frame of that change (the ledger's proof of the close; a frame after it was sent on the new
    connection)."""
    recorded = {int(f["frame"]) for f in frames}
    lost = sorted(disconnect_lost)
    spans: dict[int, list[int]] = {}
    for f in frames:
        span = spans.setdefault(int(f.get("conn", 1)), [int(f["frame"]), int(f["frame"])])
        span[0], span[1] = min(span[0], int(f["frame"])), max(span[1], int(f["frame"]))
    ordered = [spans[k] for k in sorted(spans)]
    gaps = []
    for a, b in zip(ordered, ordered[1:]):
        close = next((f for f in lost if a[1] < f < b[0]), None)
        if close is not None:
            gaps.append((a[1], close))
    ok, bad, used = [], [], set()
    for frame in sorted(set(sent_frames) - recorded):
        gap = next((g for g in gaps if g[0] < frame < g[1]), None)
        if gap is None or gap in used:
            bad.append(frame)
        else:
            used.add(gap)
            ok.append(frame)
    return ok, bad


def check_run(log: UnrealLog, frames: Sequence[dict], steps: Sequence[GimbalStep], profile: Path = PROFILE) -> dict:
    """A run whose camera bound a lens table (NavPyLensTable line) is scored per frame with the row it had to render,
    from ``profile`` (the file it loaded); one without is a fixed-K run, scored with the zoom-1 K at a still lens."""
    report: dict = {}
    sent = [s for s in log.slots if s.get("outcome") == "sent"]
    runs = sorted({s["run"] for s in log.slots})
    # The camera's frame index never restarts within a run (reconnects included); missed slots have none.
    captured_frames = [int(s["frame"]) for s in log.slots if "frame" in s]
    recorded_frames = [int(f["frame"]) for f in frames]
    report["identity"] = {"runs": runs, "connections": len({f.get("conn", 1) for f in frames}),
                          "duplicate_captured": len(captured_frames) - len(set(captured_frames)),
                          "duplicate_recorded": len(recorded_frames) - len(set(recorded_frames)),
                          "pass": len(runs) == 1 and runs[0] in log.mounts
                          and len(set(captured_frames)) == len(captured_frames)
                          and len(set(recorded_frames)) == len(recorded_frames)}
    if not report["identity"]["pass"]:
        report["pass"] = False
        return report
    mount = log.mounts[runs[0]]
    by_frame = {int(s["frame"]): s for s in sent}
    recorded = {int(f["frame"]): f for f in frames}
    bound = log.lens_tables.get(runs[0])
    if bound is None:
        seen_zoom = sorted({s.zoom for s in steps})
        report["zoom"] = {"expected": 1.0, "seen": seen_zoom, "pass": seen_zoom == [1.0]}

    # Joins.
    joined = [i for i in by_frame if i in recorded and int(recorded[i]["board_us"]) == int(by_frame[i]["board_us"])
              and int(recorded[i]["crc"]) == int(by_frame[i]["crc"], 16)]
    in_flight_ok, in_flight_bad = in_flight_at_close(
        by_frame, frames, [int(s["frame"]) for s in log.slots if s.get("outcome") == "lost:disconnect"])
    report["joins"] = {"sent": len(by_frame), "recorded": len(recorded), "joined": len(joined),
                       "in_flight_at_close": in_flight_ok, "unrecorded": in_flight_bad,
                       "pass": len(joined) == len(recorded) > 0 and not in_flight_bad
                       and len(joined) + len(in_flight_ok) == len(by_frame)}

    # Grid: each stream has one origin, and every slot that carries a board time sits on it.
    origins: dict[int, set[int]] = {}
    off_grid = 0
    for s in log.slots:
        origins.setdefault(int(s["gen"]), set()).add(int(s["origin_us"]))
        if "board_us" in s:
            off_grid += int(s["board_us"]) != slot_time_us(int(s["origin_us"]), int(s["slot"]), mount.fps)
    report["grid"] = {"off_grid": off_grid, "origins": {g: len(o) for g, o in origins.items()},
                      "pass": off_grid == 0 and all(len(o) == 1 for o in origins.values())}

    # Coverage: every consumed slot of each stream (below the receiver's next_slot) ended once.
    slots: dict[int, list[int]] = {}
    for s in log.slots:
        slots.setdefault(int(s["gen"]), []).extend(_slot_range(s))
    ends = {gen: v for (port, gen), v in log.stream_ends.items() if port == mount.receiver_port}
    started = {gen for port, gen in log.generations if port == mount.receiver_port}
    streams = {}
    for gen in sorted(set(slots) | started | {g for g, v in ends.items() if max(v) > 0}):
        logged, end = slots.get(gen, []), ends.get(gen, [])
        streams[gen] = {"logged": len(logged), "next_slot": end[0] if len(end) == 1 else end,
                        "pass": len(end) == 1 and sorted(logged) == list(range(end[0]))}
    missed = sum(len(_slot_range(s)) for s in log.slots if s["outcome"].startswith("missed:"))
    lost = sum(1 for s in log.slots if s["outcome"].startswith("lost:"))
    report["coverage"] = {"streams": streams, "missed": missed, "lost": lost,
                          "pass": bool(streams) and all(v["pass"] for v in streams.values())}

    # Unique rate over each stream's represented board time.
    intervals, span_s = 0, 0.0
    for gen in streams:
        times = sorted(int(s["board_us"]) for s in sent if int(s["gen"]) == gen)
        if len(times) > 1:
            intervals += len(times) - 1
            span_s += (times[-1] - times[0]) / 1e6
    rate = intervals / span_s if span_s > 0 else 0.0
    report["unique_rate"] = {"fps": mount.fps, "rate": round(rate, 4), "span_s": round(span_s, 3),
                             "pass": mount.fps > 0 and abs(rate / mount.fps - 1) <= UNIQUE_RATE_TOLERANCE}

    # Repeated pictures: one CRC on frames of different board times whose camera moved.
    by_crc: dict[str, list[dict]] = {}
    for s in sent:
        by_crc.setdefault(s["crc"], []).append(s)
    repeated = 0
    for group in by_crc.values():
        for a, b in zip(group, group[1:]):
            (la, qa), (lb, qb) = record_transform(a), record_transform(b)
            moved = math.dist(la, lb) > LOCATION_TOLERANCE_CM or quat_angle_deg(qa, qb) > ANGLE_TOLERANCE_DEG
            repeated += a["board_us"] != b["board_us"] and moved
    report["repeated_pictures"] = {"count": repeated, "pass": repeated == 0}

    report["oracle"] = check_oracle(log, sent, steps, mount)
    if bound is None:
        fixed = zr10_camera()
        report["reprojection"] = check_reprojection(log, sent, recorded, lambda _s: fixed)
    else:
        try:
            table, stream = load_bound_table(bound, profile)
        except (OSError, ValueError, KeyError) as error:
            report["lens"] = {"pass": False, "reason": str(error)}
        else:
            report["lens"], expected = check_lens(log, steps, stream_sources(log, mount), table, stream, recorded)
            cameras = {frame: row_camera(row, stream) for frame, (row, *_rest) in expected.items()}
            report["reprojection"] = check_reprojection(
                log, sent, recorded, lambda s: cameras.get(int(s["frame"])),
                lambda s: lens_group(*expected[int(s["frame"])]))
    report["pass"] = all(part["pass"] for part in report.values() if isinstance(part, dict))
    return report


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="record NPYC frame identities and detected corners")
    rec.add_argument("--listen", default=DEFAULT_LISTEN)
    rec.add_argument("--out", type=Path, required=True)
    rec.add_argument("--seconds", type=float, required=True)
    chk = sub.add_parser("check", help="score a recorded run")
    chk.add_argument("--frames", type=Path, required=True)
    chk.add_argument("--unreal-log", type=Path, required=True)
    chk.add_argument("--emulator-log", type=Path, required=True)
    chk.add_argument("--profile", type=Path, default=PROFILE, help="the vision_profiles.json the run's camera loaded")
    chk.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.command == "record":
        print(json.dumps(record(parse_endpoint(args.listen), args.out, args.seconds)))
        return 0
    frames = [json.loads(line) for line in args.frames.read_text(encoding="utf-8").splitlines() if line.strip()]
    log = parse_log(args.unreal_log.read_text(encoding="utf-8", errors="replace").splitlines())
    steps = parse_emulator_log(args.emulator_log.read_text(encoding="utf-8").splitlines())
    report = check_run(log, frames, steps, args.profile)
    text = json.dumps(report, indent=1, default=str)
    if args.out is not None:
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
