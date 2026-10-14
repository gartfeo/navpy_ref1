r"""Offline detection report of a dock-pass run (tools/tuning/dock_pass.py): per pass and per slant range, the frames
that had the board in view, the detector's hits and the detected board pose against the truth.

The truth (the bridge's plant state, link.csv) is used here only, after the flight, for scoring; nothing of it reaches
navpy. A frame shows the pose of its board time (NPYC v2 stamp, pose_flags exact), so its truth is the link.csv row of
that board time. The camera is the fixed body camera of hitl_unreal_camera.md (dock_pass_geometry.py): its measured
offset from the vehicle's position, pitched MOUNT_PITCH_DEG in the body, FOV_H_DEG wide on a 1280 x 720 picture, no distortion.
"In view" means all four corners of the board project inside the picture. The detected pose is OpenCV's ChArUco
corners and solvePnP on the frames the live check saved (--save-detected), with the board's Unreal size.

  .venv\Scripts\python.exe -m tools.unreal.dock_pass_report --run-json C:\tune_hitl\dock1.json
    --link-csv C:\tune_hitl\dock1\link.csv --frames C:\tune_hitl\dock1_frames --out C:\tune_hitl\dock1_report
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import shutil
import statistics
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from tools.unreal.dock_pass_geometry import Truth, board_view, detected_pose, pose_error
from tools.unreal.npyc_live_check import DETECTED_DIR

POSE_EXACT_FLAG = 2                   # NPYC v2 pose_flags bit 1: the rendered pose is that board-time sample
POSE_STAMP_VALID_FLAG = 1
# A frame's board time must match a link.csv row this closely (both are the same board frame's time_usec)
TRUTH_MATCH_US = 1000
SLANT_BINS_M = (0.0, 15.0, 20.0, 25.0, 30.0, 40.0, 60.0, math.inf)
SAMPLES_PER_PASS = 2                  # nearest and farthest detection of each pass, with corners drawn
# Link gate of the run (hitl_gate.py): reported from the run's JSON
STALE_MAX, AGE_MAX_MS = 0.05, 50.0


def load_truth(link_csv: Path) -> tuple[list[int], dict[int, Truth]]:
    states: dict[int, Truth] = {}
    with link_csv.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("lat_deg"):
                states[int(row["board_time_us"])] = Truth(float(row["lat_deg"]), float(row["lon_deg"]),
                                                          float(row["alt_msl_m"]), float(row["roll_deg"]),
                                                          float(row["pitch_deg"]), float(row["yaw_deg"]))
    return sorted(states), states


def truth_at(times: list[int], states: dict[int, Truth], board_time_us: int) -> Optional[Truth]:
    i = bisect.bisect_left(times, board_time_us)
    best = min((t for t in times[max(0, i - 1):i + 1]), key=lambda t: abs(t - board_time_us), default=None)
    return states[best] if best is not None and abs(best - board_time_us) <= TRUTH_MATCH_US else None


def pass_of(board_ms: float, windows: list[tuple[int, int]]) -> Optional[int]:
    return next((k for k, (lo, hi) in enumerate(windows) if lo <= board_ms <= hi), None)


def _stats(values: list[float]) -> Optional[dict]:
    if not values:
        return None
    ordered = sorted(values)
    return {"n": len(ordered), "median": round(statistics.median(ordered), 3),
            "p90": round(ordered[min(len(ordered) - 1, int(0.9 * len(ordered)))], 3), "max": round(ordered[-1], 3)}


def score_frames(rows: list[dict], times: list[int], states: dict[int, Truth], board: tuple[float, float, float],
                 windows: list[tuple[int, int]], detected_dir: Optional[Path]) -> list[dict]:
    """One record per frame inside a pass window with an exact pose: geometry, detection and pose error."""
    scored = []
    for row in rows:
        flags = int(row["pose_flags"] or 0)
        if flags & (POSE_STAMP_VALID_FLAG | POSE_EXACT_FLAG) != POSE_STAMP_VALID_FLAG | POSE_EXACT_FLAG:
            continue
        board_us = int(row["board_time_us"])
        k = pass_of(board_us / 1000.0, windows)
        truth = truth_at(times, states, board_us) if k is not None else None
        if truth is None:
            continue
        view = board_view(truth, board)
        rec = {"frame_index": int(row["frame_index"]), "pass": k, "board_time_us": board_us, "view": view,
               "ran": row["detected"] != "", "detected": row["detected"] == "1",
               "latency_ms": float(row["latency_ms"]) if row.get("latency_ms") else None, "pose": None, "error": None}
        image_path = detected_dir / f"{rec['frame_index']:06d}.png" if detected_dir is not None else None
        if rec["detected"] and image_path is not None and image_path.is_file():
            pose = detected_pose(cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED))
            if pose is not None:
                rec["pose"], rec["error"] = pose, pose_error(view, pose)
            rec["image"] = image_path
        scored.append(rec)
    return scored


def summarize_group(recs: list[dict]) -> dict:
    seen = [r for r in recs if r["view"].in_view]
    ran = [r for r in seen if r["ran"]]
    hits = [r for r in ran if r["detected"]]
    errors = [r["error"] for r in hits if r["error"] is not None]
    return {"frames": len(recs), "in_view": len(seen), "in_view_ran": len(ran), "detected": len(hits),
            "rate": round(len(hits) / len(ran), 3) if ran else None,
            "detected_out_of_view": sum(r["detected"] for r in recs if not r["view"].in_view),
            "agl_in_view_m": _stats([r["view"].agl_m for r in seen]),
            "slant_in_view_m": _stats([r["view"].slant_m for r in seen]),
            "slant_detected_max_m": round(max(r["view"].slant_m for r in hits), 2) if hits else None,
            "pose_scored": len(errors),
            "position_error_m": _stats([e["position_m"] for e in errors]),
            "range_error_m": _stats([abs(e["range_m"]) for e in errors]),
            "tilt_error_deg": _stats([e["tilt_deg"] for e in errors])}


def save_samples(recs: list[dict], out: Path) -> list[str]:
    """For each pass, its nearest and farthest detections with the truth outline (green) and the detected
    ChArUco corners (red)."""
    out.mkdir(parents=True, exist_ok=True)
    saved = []
    for k in sorted({r["pass"] for r in recs}):
        hits = sorted((r for r in recs if r["pass"] == k and r.get("image") is not None),
                      key=lambda r: r["view"].slant_m)
        for r in ({id(h): h for h in (hits[:1] + hits[-1:])}.values() if hits else []):
            image = cv2.imread(str(r["image"]), cv2.IMREAD_COLOR)
            outline = r["view"].corners_px
            if np.all(np.isfinite(outline)):
                cv2.polylines(image, [outline.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 1)
            if r["pose"] is not None:
                for x, y in r["pose"]["corners"]:
                    cv2.circle(image, (int(round(x)), int(round(y))), 2, (0, 0, 255), -1)
            name = f"pass{k + 1}_slant{r['view'].slant_m:.0f}m_frame{r['frame_index']:06d}.png"
            cv2.imwrite(str(out / name), image)
            saved.append(str(out / name))
    return saved


def link_gate(run: dict) -> dict:
    health = (run.get("vehicle") or {}).get("link_health")
    if not health:
        return {"available": False}
    # a log without the gate's numbers (a SITL build whose HIL record lacks fields, or no HIL frames: hitl_gate.py
    # gives None) fails the numeric gate
    all_logs = list(health["logs"].values())
    logs = [log for log in all_logs if log.get("stale_fraction") is not None and log.get("age_max_us") is not None]
    stale = max((log["stale_fraction"] for log in logs), default=None)
    age = max((log["age_max_us"] for log in logs), default=None)
    within = (bool(logs) and len(logs) == len(all_logs) and stale <= STALE_MAX and age / 1000.0 <= AGE_MAX_MS)
    return {"available": True, "ok": health["ok"], "stale_fraction": stale,
            "age_max_ms": age / 1000.0 if age is not None else None,
            "within_stale_5pct_age_50ms": within, "problems": health.get("problems", [])}


def read_run(path: Path) -> dict:
    """The run's JSON record: the runner writes it first in the file, other output may follow."""
    text = path.read_text(encoding="utf-8", errors="replace")
    return json.JSONDecoder().raw_decode(text.lstrip())[0]


def report(run_json: Path, link_csv: Path, frames_dir: Path, out: Path) -> dict:
    run = read_run(run_json)
    card = run["card"]
    board = (card["board"]["lat"], card["board"]["lon"], card["board"]["msl_m"])
    passes = run.get("passes", [])
    windows = [tuple(p["window_board_ms"]) for p in passes]
    times, states = load_truth(link_csv)
    with (frames_dir / "frames.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    detected_dir = frames_dir / DETECTED_DIR
    recs = score_frames(rows, times, states, board, windows, detected_dir if detected_dir.is_dir() else None)
    per_pass = [{"pass": k + 1, "height_m": p["height_m"], "heading_deg": p["heading_deg"],
                 **summarize_group([r for r in recs if r["pass"] == k])} for k, p in enumerate(passes)]
    per_slant = []
    for lo, hi in zip(SLANT_BINS_M, SLANT_BINS_M[1:]):
        group = [r for r in recs if lo <= r["view"].slant_m < hi]
        per_slant.append({"slant_m": [lo, hi if math.isfinite(hi) else None], **summarize_group(group)})
    latencies = [r["latency_ms"] for r in recs if r["latency_ms"] is not None]
    if out.exists():
        shutil.rmtree(out / "samples", ignore_errors=True)
    result = {"board": card["board"], "passes": per_pass, "by_slant_range": per_slant,
              "latency_ms_in_windows": _stats(latencies), "link_gate": link_gate(run),
              "frames_in_windows": len(recs), "samples": save_samples(recs, out / "samples")}
    (out / "report.json").write_text(json.dumps(result, indent=1))
    return result


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--run-json", type=Path, required=True, help="the dock-pass run's JSON record")
    parser.add_argument("--link-csv", type=Path, required=True, help="the bridge's link.csv of the run (the truth)")
    parser.add_argument("--frames", type=Path, required=True, help="the live check's --out folder")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    print(json.dumps(report(args.run_json, args.link_csv, args.frames, args.out), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
