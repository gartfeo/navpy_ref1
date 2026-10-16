r"""Replay a link.csv's poses into Unreal at real time (no board, no bridge) and measure what Unreal renders.

The bridge's rows (tools/tuning/hitl_bridge, --log-csv) are sent the way the bridge sends them: one
vehicle_state.v1 datagram per POSE_PERIOD_US slot of board time, at the pace of the board time. navpy listens
for the native camera (npyc://, NPYC v2) and records every frame's capture time and pose stamp.

Reported (JSON, --out):
- frames: count, capture gaps over GAP_S (Unreal freezes), the longest gap;
- smoothness: per pair of consecutive stamped frames, board time advanced / capture time advanced (1 = the
  vehicle moves at its real pace in the pictures): median and p95 of |ratio - 1|, and the share of frames
  that repeat the previous pose;
- with --unreal-log (Unreal launched with -NavPyPoseStampLog=1, the log read after the replay):
  per game tick the same ratio for the applied pose; the stamp check (each applied pose against the received
  samples interpolated at its stamp: position cm, attitude deg); the vehicle move cost (NavPyMoveCost);
  and the path error of interpolating the 50 Hz poses (the applied stamps' interpolated poses against the
  link.csv rows around them, metres).

  .venv\Scripts\python.exe -m tools.unreal.pose_replay --link-csv C:\tune_hitl\dock4\link.csv
    --alt-offset-m 23.5 --out C:\tune_hitl\replay\interp.json
    --unreal-log C:\repos\unreal-projects\AasSimProject\Saved\Logs\AasSimProject.log
This is simulator truth replayed for rendering checks only; nothing of it reaches navpy's navigation.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

from tools.tuning.hitl_bridge.physics import PlantState
from tools.tuning.hitl_bridge.unreal_pose import POSE_PERIOD_US, UnrealPoseSender
from tools.unreal.pose_stamp_log import parse_unreal_log, quantiles, stamp_check

GAP_S = 1.0                     # a capture gap longer than this is a freeze, not a skipped tick
POSE_STAMP_VALID, POSE_EXACT, POSE_INTERPOLATED = 1, 2, 4      # NPYC v2 pose_flags (npyc_codec.py)
EARTH_RADIUS_M = 6378137.0
SETTLE_S = 2.0                  # after the last pose: frames still in flight
_PACE_SLEEP_S = 0.002           # pacing granularity of the sender loop (well under the 20 ms pose period)


@dataclass(frozen=True)
class Pose:
    board_time_us: int
    state: PlantState


@dataclass(frozen=True)
class Frame:
    capture_s: float            # Unreal's capture time (FPlatformTime)
    board_time_us: int
    pose_flags: int


def load_poses(link_csv: Path, start_s: float = 0.0, end_s: float = math.inf) -> list[Pose]:
    poses = []
    with link_csv.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if not row.get("lat_deg"):
                continue
            board_us = int(row["board_time_us"])
            if start_s * 1e6 <= board_us <= end_s * 1e6:
                poses.append(Pose(board_us, PlantState(
                    float(row["lat_deg"]), float(row["lon_deg"]), float(row["alt_msl_m"]),
                    math.radians(float(row["roll_deg"])), math.radians(float(row["pitch_deg"])),
                    math.radians(float(row["yaw_deg"])), *([0.0] * 11))))
    return poses


def replay(poses: list[Pose], sender: UnrealPoseSender, pace: bool = True) -> None:
    """Send each pose when its board time comes (relative to the first); the sender keeps one per slot."""
    if not poses:
        return
    start, first_us = time.monotonic(), poses[0].board_time_us
    for pose in poses:
        due_s = (pose.board_time_us - first_us) / 1e6
        while pace and time.monotonic() - start < due_s:
            time.sleep(_PACE_SLEEP_S)
        sender(pose.board_time_us, pose.state)


def pace_stats(times_s: list[float], board_us: list[int]) -> Optional[dict]:
    """Per consecutive pair: board time advanced / clock advanced. |ratio - 1| and the share of repeats."""
    deviations, repeats = [], 0
    for k in range(1, len(times_s)):
        dt = times_s[k] - times_s[k - 1]
        db = (board_us[k] - board_us[k - 1]) / 1e6
        if dt <= 0 or db < 0 or dt > GAP_S:
            continue
        repeats += db == 0
        deviations.append(abs(db / dt - 1.0))
    stats = quantiles(deviations)
    if stats is not None:
        stats["repeated_share"] = round(repeats / len(deviations), 4)
    return stats


def frame_stats(frames: list[Frame]) -> dict:
    gaps = [round(b.capture_s - a.capture_s, 2) for a, b in zip(frames, frames[1:]) if b.capture_s - a.capture_s > GAP_S]
    longest = max((b.capture_s - a.capture_s for a, b in zip(frames, frames[1:])), default=0.0)
    stamped = [f for f in frames if f.pose_flags & POSE_STAMP_VALID and f.pose_flags & (POSE_EXACT | POSE_INTERPOLATED)]
    return {"frames": len(frames), "stamped": len(stamped),
            "interpolated": sum(bool(f.pose_flags & POSE_INTERPOLATED) for f in stamped),
            "gaps_over_s": gaps, "longest_gap_s": round(longest, 3),
            "pace": pace_stats([f.capture_s for f in stamped], [f.board_time_us for f in stamped])}


def _lla_at(poses: list[Pose], board_time_us: int) -> Optional[tuple[float, float, float]]:
    times = [p.board_time_us for p in poses]
    i = bisect.bisect_right(times, board_time_us)
    if i == 0 or (i == len(poses) and times[-1] != board_time_us):
        return None
    a = poses[i - 1].state
    if times[i - 1] == board_time_us:
        return a.lat_deg, a.lon_deg, a.alt_msl_m
    b, alpha = poses[i].state, (board_time_us - times[i - 1]) / (times[i] - times[i - 1])
    return (a.lat_deg + alpha * (b.lat_deg - a.lat_deg), a.lon_deg + alpha * (b.lon_deg - a.lon_deg),
            a.alt_msl_m + alpha * (b.alt_msl_m - a.alt_msl_m))


def path_error(sent: list[Pose], rows: list[Pose], stamps: Iterable[int]) -> Optional[dict]:
    """Interpolating the sent poses at a stamp against the link.csv rows around it (the real path), metres."""
    errors = []
    for stamp in stamps:
        a, b = _lla_at(sent, stamp), _lla_at(rows, stamp)
        if a is None or b is None:
            continue
        north = math.radians(a[0] - b[0]) * EARTH_RADIUS_M
        east = math.radians(a[1] - b[1]) * EARTH_RADIUS_M * math.cos(math.radians(b[0]))
        errors.append(math.sqrt(north * north + east * east + (a[2] - b[2]) ** 2))
    return quantiles(errors)


def sent_poses(poses: list[Pose], period_us: int = POSE_PERIOD_US) -> list[Pose]:
    """The poses the sender's slot rule keeps (the first of each board-time slot)."""
    kept, last_slot = [], None
    for pose in poses:
        slot = pose.board_time_us // period_us
        if slot != last_slot:
            kept.append(pose)
            last_slot = slot
    return kept


def log_report(log_path: Path, poses: list[Pose]) -> dict:
    with log_path.open(encoding="utf-8", errors="replace") as stream:
        samples, applied, moves = parse_unreal_log(stream)
    ticks = pace_stats([p.log_s for p in applied if p.valid], [p.board_time_us for p in applied if p.valid])
    return {"samples": len(samples), "applied": len(applied), "applied_valid": sum(p.valid for p in applied),
            "tick_pace": ticks, "stamp_check": stamp_check(samples, applied),
            "path_error_m": path_error(sent_poses(poses), poses,
                                       [p.board_time_us for p in applied if p.valid and p.interpolated]),
            "move_cost": {"windows": len(moves),
                          "mean_ms": round(sum(m["mean_ms"] * m["moves"] for m in moves)
                                           / max(1, sum(m["moves"] for m in moves)), 4),
                          "max_ms": max((m["max_ms"] for m in moves), default=None)}}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--link-csv", type=Path, required=True)
    parser.add_argument("--alt-offset-m", type=float, required=True,
                        help="ellipsoid minus MSL at the origin, as the bridge's --unreal-pose (23.5 at Yeghvard)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--start-s", type=float, default=0.0)
    parser.add_argument("--end-s", type=float, default=math.inf)
    parser.add_argument("--unreal", default="127.0.0.1:17110")
    parser.add_argument("--camera", default="npyc://127.0.0.1:18110")
    parser.add_argument("--unreal-log", type=Path, help="read after the replay (-NavPyPoseStampLog=1)")
    parser.add_argument("--analyze-only", action="store_true", help="only read --unreal-log; no replay")
    args = parser.parse_args(argv)

    poses = load_poses(args.link_csv, args.start_s, args.end_s)
    report: dict = {"link_csv": str(args.link_csv), "poses": len(poses)}
    if not args.analyze_only:
        from navpy.plugins.camera.unreal.frame_capture_npyc import NpycFrameCapture
        from tools.unreal.npyc_live_check import _Logger

        frames: list[Frame] = []
        backend = NpycFrameCapture.open(args.camera, _Logger(), None,
                                        lambda h, _read, _stamp: frames.append(
                                            Frame(h.capture_time_s, h.board_time_us, h.pose_flags)))
        stop = threading.Event()
        reader = threading.Thread(target=backend.run, args=(lambda _f, _s: None, lambda: not stop.is_set()),
                                  daemon=True)
        reader.start()
        host, port = args.unreal.rsplit(":", 1)
        sender = UnrealPoseSender((host, int(port)), args.alt_offset_m)
        replay(poses, sender)
        time.sleep(SETTLE_S)
        stop.set()
        backend.close()
        sender.close()
        report["sent"] = str(sender.stats)
        report["camera"] = frame_stats(frames)
    if args.unreal_log is not None:
        report["unreal_log"] = log_report(args.unreal_log, poses)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
