r"""Offline FOV check of the Unreal lens table run (HITL stage 1 step 5, hitl_final_approach.md section 9): per
used zoom, the pinhole the rendered pictures show, against the real camera's reference FOV.

From a cadence_check recording of a run with -NavPyLensProfile: for every settled frame (cadence_lens's expected
row, the labels checked by cadence_check), the detected ChArUco corners against the board's truth corners seen from
the frame's logged camera (harness only, after the run) give a linear fit of fx, fy, cx, cy and skew per row. Its
pinhole FOV (both sides of the fitted centre) is compared with the reference knot of that reading in the private
camera reference (zoom_knots: absolute zoom N settles at knot N; reading 1.0 is the wide stop) and with the row
itself. Pass: every used row with a knot within 2 % in both axes. The pictures here are raw NPYC frames: step 5's
decoded-chart item stays open until the encoded stream exists (step 8). Scoring only: nothing reaches navpy.

  .venv\Scripts\python.exe -m tools.unreal.lens_fov_check --frames C:\tune_hitl\lens\frames.jsonl ^
      --unreal-log ...\AasSimProject.log --emulator-log C:\tune_hitl\lens\zr10.jsonl --out C:\tune_hitl\lens\fov.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from navpy.core.perception.camera.reading_intrinsics import ABSOLUTE_ZOOM, WIDE_STOP, ReadingOptics, StreamMode
from tools.unreal.cadence_lens import check_lens, load_bound_table, row_camera, setting_text
from tools.unreal.cadence_log import GimbalStep, UnrealLog, parse_emulator_log, parse_log
from tools.unreal.cadence_oracle import (MIN_REPROJECTION_CORNERS, board_corners_world, check_reprojection,
                                         record_transform, stream_sources)
from tools.unreal.lens_fixtures import PROFILE
from tools.unreal.unreal_math import quat_conj, quat_rotate

FOV_TOLERANCE = 0.02          # step 5: within 2 % of the real reference at every used zoom
MIN_FIT_CORNERS = 50          # corners per row for a fit of five unknowns from several views
REFERENCE = Path(os.environ.get("LOCALAPPDATA", "")) / "navpy-hitl-tools/measurements/zr10_camera_reference_20261019.json"


def fov_deg(fx: float, fy: float, cx: float, cy: float, width: int, height: int) -> tuple[float, float]:
    """Pinhole FOV across the frame, both sides of the principal point (the reference's convention)."""
    return (math.degrees(math.atan(cx / fx) + math.atan((width - cx) / fx)),
            math.degrees(math.atan(cy / fy) + math.atan((height - cy) / fy)))


def fit_pinhole(normalized: np.ndarray, pixels: np.ndarray) -> tuple[float, float, float, float, float]:
    """Least squares of u = fx x + skew y + cx and v = fy y + cy over normalized OpenCV coordinates (x, y)."""
    x, y = normalized[:, 0], normalized[:, 1]
    ones = np.ones_like(x)
    (fx, skew, cx), *_ = np.linalg.lstsq(np.column_stack([x, y, ones]), pixels[:, 0], rcond=None)
    (fy, cy), *_ = np.linalg.lstsq(np.column_stack([y, ones]), pixels[:, 1], rcond=None)
    return float(fx), float(fy), float(cx), float(cy), float(skew)


def normalized_corners(record: dict, frame: dict, corners_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The detected corners' pixels and their truth points in the frame's camera as OpenCV (x/z, y/z)."""
    loc, quat = record_transform(record)
    inverse = quat_conj(quat)
    points, pixels = [], []
    for corner_id, u, v in frame["corners"]:
        p = corners_world[int(corner_id)]
        x, y, z = quat_rotate(inverse, (p[0] - loc[0], p[1] - loc[1], p[2] - loc[2]))
        if x > 0:
            points.append([y / x, -z / x])
            pixels.append([u, v])
    return np.array(points).reshape(-1, 2), np.array(pixels).reshape(-1, 2)


def reference_knots(path: Path) -> dict[str, tuple[float, float]]:
    """The reference's pinhole FOV (h, v) by reading."""
    rows = json.loads(path.read_text(encoding="utf-8"))["optics"]["zoom_knots"]["rows"]
    return {f"{row['reading']:.1f}": (float(row["hfov_deg_pinhole"]), float(row["vfov_deg_pinhole"])) for row in rows}


def knot_for(row: ReadingOptics, knots: dict[str, tuple[float, float]]) -> Optional[tuple[float, float]]:
    """A row's knot: the wide stop at 1.0, or an integer absolute command at the reading it settles at."""
    if row.setting.kind == WIDE_STOP or (row.setting.kind == ABSOLUTE_ZOOM and row.setting.command_tenths % 10 == 0):
        return knots.get(row.reading)
    return None


def _difference_pct(got: Sequence[float], want: Sequence[float]) -> list[float]:
    return [round(100.0 * (g / w - 1.0), 3) for g, w in zip(got, want)]


def check_fov(log: UnrealLog, frames: Sequence[dict], steps: Sequence[GimbalStep], profile: Path,
              knots: dict[str, tuple[float, float]]) -> dict:
    runs = sorted({s["run"] for s in log.slots})
    if len(runs) != 1 or runs[0] not in log.lens_tables or runs[0] not in log.mounts or log.board is None:
        return {"pass": False, "reason": "needs one camera run with a NavPyLensTable, NavPyCameraMount and board line"}
    table, stream = load_bound_table(log.lens_tables[runs[0]], profile)
    recorded = {int(f["frame"]): f for f in frames}
    lens, expected = check_lens(log, steps, stream_sources(log, log.mounts[runs[0]]), table, stream, recorded)
    sent = [s for s in log.slots if s.get("outcome") == "sent"]
    cameras = {frame: row_camera(row, stream) for frame, (row, *_rest) in expected.items()}
    flips = check_reprojection(log, sent, recorded, lambda s: cameras.get(int(s["frame"]))).get("flips")
    if flips is None:
        return {"pass": False, "reason": "no frame with enough detected corners", "lens": lens}
    corners_world = board_corners_world(log.board, *flips)
    groups: dict[tuple[str, str], tuple[ReadingOptics, list, list]] = {}
    for s in sent:
        frame = int(s["frame"])
        if frame not in expected or expected[frame][1] != "settled" or frame not in recorded:
            continue
        if len(recorded[frame].get("corners") or []) < MIN_REPROJECTION_CORNERS:
            continue
        row = expected[frame][0]
        points, pixels = normalized_corners(s, recorded[frame], corners_world)
        group = groups.setdefault((row.reading, setting_text(row.setting)), (row, [], []))
        group[1].append(points)
        group[2].append(pixels)
    rows = _row_results(groups, stream, knots)
    return {"lens_labels_pass": lens["pass"], "rows": rows, **_verdict(rows, lens["pass"])}


def _row_results(groups: dict, stream: StreamMode, knots: dict[str, tuple[float, float]]) -> list[dict]:
    results = []
    for (reading, setting), (row, points, pixels) in sorted(groups.items()):
        result = {"reading": reading, "set_by": setting, "frames": len(pixels), "corners": sum(map(len, pixels))}
        points, pixels = np.vstack(points), np.vstack(pixels)
        if len(points) >= MIN_FIT_CORNERS:
            fx, fy, cx, cy, skew = fit_pinhole(points, pixels)
            fitted = fov_deg(fx, fy, cx, cy, stream.image_width, stream.image_height)
            own = fov_deg(row.fx, row.fy, row.cx, row.cy, stream.image_width, stream.image_height)
            knot = knot_for(row, knots)
            result.update(fit={"fx": round(fx, 2), "fy": round(fy, 2), "cx": round(cx, 2), "cy": round(cy, 2),
                               "skew": round(skew, 3)},
                          fitted_fov_deg=[round(v, 3) for v in fitted], row_fov_deg=[round(v, 3) for v in own],
                          vs_row_pct=_difference_pct(fitted, own),
                          reference_fov_deg=None if knot is None else list(knot),
                          vs_reference_pct=None if knot is None else _difference_pct(fitted, knot))
        results.append(result)
    return results


def _verdict(rows: list[dict], lens_pass: bool) -> dict:
    """Pass only when the lens labels passed and every settled setting was fitted and has a reference knot."""
    scored = [r for r in rows if r.get("vs_reference_pct") is not None]
    worst = max((abs(p) for r in scored for p in r["vs_reference_pct"]), default=None)
    unfitted = [f"{r['reading']}/{r['set_by']}" for r in rows if "fit" not in r]
    unreferenced = [f"{r['reading']}/{r['set_by']}" for r in rows if "fit" in r and r["vs_reference_pct"] is None]
    return {"scored_rows": len(scored), "unfitted_rows": unfitted, "unreferenced_rows": unreferenced,
            "worst_vs_reference_pct": worst,
            "pass": lens_pass and bool(scored) and not unfitted and not unreferenced and worst is not None
            and worst <= 100.0 * FOV_TOLERANCE}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--unreal-log", type=Path, required=True)
    parser.add_argument("--emulator-log", type=Path, required=True)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--reference", type=Path, default=REFERENCE)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(argv)
    frames = [json.loads(line) for line in args.frames.read_text(encoding="utf-8").splitlines() if line.strip()]
    log = parse_log(args.unreal_log.read_text(encoding="utf-8", errors="replace").splitlines())
    steps = parse_emulator_log(args.emulator_log.read_text(encoding="utf-8").splitlines())
    report = check_fov(log, frames, steps, args.profile, reference_knots(args.reference))
    text = json.dumps(report, indent=1)
    if args.out is not None:
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
