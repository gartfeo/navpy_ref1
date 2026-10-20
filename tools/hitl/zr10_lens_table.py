"""Build the ZR10 lens table (vision_profiles.json, siyi_zr10 camera.reading_intrinsics) from the saved measurements.

The one source of the table: every row is computed here from the measurements in the private folder
(tools/tuning/private_data.py, measurements/), never typed by hand. Rerun it when a measurement is added. The measured
constants, the input folders and which sessions to use are private too: measurements/zr10_lens_inputs.json.

- Focal length, every (reading, absolute zoom command): a still-board sweep of every command, up and down. The board
  and gimbal did not move, so the similarity scale between two views is their focal-length ratio. One constant ties
  the sweep to the tilted board fits; a laser range test's correction of board-fit focal lengths scales the result.
- Wide stop: the absolute zoom 1.0 row divided by the measured wide-stop ratio.
- Image centre, fy/fx and distortion: measured where a set with tilted views exists (the wall model at the wide stop,
  the ChArUco calibration and the tilted board sets), interpolated between them in log focal length, distortion in
  pixel units (k1/fx^2, k2/fx^4). Rows say which (basis). The missing tilted sets are a gimbal-controller to-do
  (docs/TODO.md).
- Check: every saved board session is scored with its row (each view's pose solved with the lens fixed); the report
  goes to the private folder.

    python -m tools.hitl.zr10_lens_table [--write-profile]
"""
from __future__ import annotations

import argparse
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from navpy.core.perception.camera.reading_intrinsics import ABSOLUTE_ZOOM, INTERPOLATED, MEASURED, WIDE_STOP
from tools.hitl.zr10_board_sets import candidate, fit, load_session, score
from tools.hitl.zr10_lens_profile import PROFILE, section, write_profile
from tools.tuning.private_data import private_dir

SIZE = (1920, 1080)
LENS_KEYS = ("fx", "fy", "cx", "cy", "k1", "k2")


@dataclass(frozen=True)
class Inputs:
    sweep: Path         # holds <sweep run>/sweep.json
    sessions: Path      # holds <session>/session.json and candidate-*.json
    anchors: Path       # wall_solution_v3.json, charuco_zoom2_base_model.json
    manifest: Path      # Manifest fields, plus the folders above relative to the manifest's folder


@dataclass(frozen=True)
class Manifest:
    """The private inputs that are not files of their own (Inputs.manifest)."""

    laser_factor: float             # board-fit focal length -> laser-ranged focal length
    wide_in: float                  # absolute zoom 1.0 focal length / wide-stop focal length
    sweep_runs: tuple[str, ...]     # chained in this order
    tilted: dict[float, str]        # absolute zoom command -> tilted board session
    refit_from_sweep: frozenset[float]  # tilted sets whose saved fit is refit from the sweep-implied focal length
    charuco_command: float          # the absolute zoom command of the ChArUco anchor


def load_manifest(path: Path) -> Manifest:
    raw = json.loads(path.read_text(encoding="utf-8"))
    tilted = {float(k): str(v) for k, v in raw["tilted"].items()}
    refit = frozenset(float(v) for v in raw["refit_from_sweep"])
    if not refit <= set(tilted):
        raise ValueError(f"{path.name}: refit_from_sweep names a command without a tilted set")
    if len(tilted) <= len(refit):
        raise ValueError(f"{path.name}: the sweep constant needs a tilted set that is not refit")
    return Manifest(float(raw["laser_factor"]), float(raw["wide_in"]), tuple(raw["sweep_runs"]), tilted, refit,
                    float(raw["charuco_command"]))


def default_inputs(root: Optional[Path] = None) -> Inputs:
    manifest = (private_dir() if root is None else root) / "measurements" / "zr10_lens_inputs.json"
    raw = json.loads(manifest.read_text(encoding="utf-8"))
    return Inputs(*(manifest.parent / raw[key] for key in ("sweep_dir", "sessions_dir", "anchors_dir")), manifest)


def _tenths(value: float) -> int:
    return round(value * 10)


def _label(tenths: int) -> str:
    return f"{tenths // 10}.{tenths % 10}"


def _similarity_scale(a: np.ndarray, b: np.ndarray) -> float:
    m, _ = cv2.estimateAffinePartial2D(a.astype(np.float32), b.astype(np.float32), method=cv2.LMEDS)
    return float(math.hypot(m[0, 0], m[1, 0]))


def sweep_scales(runs: Sequence[Sequence[Mapping[str, Any]]]) -> dict[tuple[int, int], list[float]]:
    """Relative scale of every settled sweep view, keyed by (reading, command) in tenths. Runs are chained through
    a (reading, command) both saw; a view whose reading changed while it was taken is dropped."""
    views = []
    for run_index, records in enumerate(runs):
        for r in records:
            frames = [np.asarray(f["corners"], float).reshape(-1, 2) for f in r["frames"] if f.get("corners")]
            if frames and r.get("reading") is not None and r.get("reading") == r.get("reading_after"):
                views.append((run_index, (_tenths(r["reading"]), _tenths(r["command"])), np.mean(frames, axis=0)))
    if not views:
        raise ValueError("the sweep has no settled view with corners")
    ref = views[0][2]
    base = {views[0][0]: 1.0}
    for run_index in sorted({v[0] for v in views}):
        if run_index in base:
            continue
        link = next(((p, v) for v in views if v[0] == run_index for p in views
                     if p[0] in base and p[1] == v[1]), None)
        if link is None:
            raise ValueError(f"sweep run {run_index} shares no (reading, command) with the runs before it")
        p, v = link
        base[run_index] = base[p[0]] * _similarity_scale(ref, p[2]) / _similarity_scale(ref, v[2])
    scales: dict[tuple[int, int], list[float]] = {}
    for run_index, key, corners in views:
        scales.setdefault(key, []).append(base[run_index] * _similarity_scale(ref, corners))
    return scales


def anchors(inputs: Inputs, m: Manifest, scales: Mapping[tuple[int, int], float]) -> tuple[dict, float, dict]:
    """The measured lens shapes keyed by (reading, setting), the sweep constant and each tilted set's deviation."""
    found, const = {}, {}
    for command, sid in m.tilted.items():
        s, objs, imgs = load_session(inputs.sessions / sid)
        request = s.get("absolute_zoom_request") or {}
        if request.get("requested") != command:
            raise ValueError(f"{sid}: absolute zoom request {request.get('requested')} is not {command}")
        key = (_tenths(s["zoom_reading"]), _tenths(command))
        found[key] = {"lens": {k: float(candidate(inputs.sessions / sid)[k]) for k in LENS_KEYS},
                      "source": f"gimbal-controller out/zoom_calibrations/{sid}", "session": sid}
    for key, a in found.items():
        if key[1] / 10 not in m.refit_from_sweep:
            const[key] = a["lens"]["fx"] / scales[key]
    implied = float(np.mean(list(const.values())))
    for key, a in found.items():
        if key[1] / 10 in m.refit_from_sweep:
            _, objs, imgs = load_session(inputs.sessions / a["session"])
            a["lens"] = fit(objs, imgs, SIZE, implied * scales[key])
            a["source"] += " (refit from the sweep-implied focal length)"
            const[key] = a["lens"]["fx"] / scales[key]
    c = float(np.mean(list(const.values())))
    spread = {_label(k[0]): round(float(v / c - 1) * 100, 3) for k, v in const.items()}
    ch = json.loads((inputs.anchors / "charuco_zoom2_base_model.json").read_text(encoding="utf-8"))
    request = ch["provenance"]["absolute_zoom_request"]
    if request["requested"] != m.charuco_command or (ch["width"], ch["height"]) != SIZE:
        raise ValueError(f"the ChArUco anchor is not absolute zoom {m.charuco_command} at 1920x1080")
    found[(_tenths(request["reported"]), _tenths(m.charuco_command))] = {
        "lens": {k: float(ch[k]) for k in LENS_KEYS},
        "source": "charuco_zoom2_base_model.json"}
    wall = json.loads((inputs.anchors / "wall_solution_v3.json").read_text(encoding="utf-8"))["matrix"]
    found[(10, WIDE_STOP)] = {
        "lens": {"fx": wall["f"], "fy": wall["f"] * wall["aspect"], "cx": wall["cx"], "cy": wall["cy"],
                 "k1": wall["k1"], "k2": wall["k2"]},
        "source": "wall_solution_v3.json (wall marks)"}
    return found, c, spread


def _shape(lens: Mapping[str, float], at_fx: float) -> dict[str, float]:
    """A measured lens as an interpolation point: placed at the table's focal length for its setting (the fit's own
    focal length carries the board-scale bias), distortion in pixel units of its own fit."""
    f = lens["fx"]
    return {"logf": math.log(at_fx), "aspect": lens["fy"] / f, "cx": lens["cx"], "cy": lens["cy"],
            "a1": lens["k1"] / f ** 2, "a2": lens["k2"] / f ** 4}


def lens_at(fx: float, shapes: Sequence[Mapping[str, float]]) -> dict[str, float]:
    """Centre, fy/fx and pixel-unit distortion interpolated in log focal length between measured shapes (held
    constant beyond the end ones), at the given focal length."""
    pts = sorted(shapes, key=lambda p: p["logf"])
    xs = [p["logf"] for p in pts]
    v = {k: float(np.interp(math.log(fx), xs, [p[k] for p in pts])) for k in ("aspect", "cx", "cy", "a1", "a2")}
    return {"fx": fx, "fy": fx * v["aspect"], "cx": v["cx"], "cy": v["cy"], "k1": v["a1"] * fx ** 2,
            "k2": v["a2"] * fx ** 4}


def table_rows(scales: Mapping[tuple[int, int], float], found: Mapping, c: float, m: Manifest) -> list[dict]:
    """One row per (reading, command) the sweep saw, plus the wide stop, in the profile's row form."""
    focal = {key: m.laser_factor * c * s for key, s in scales.items()}
    focal[(10, WIDE_STOP)] = m.laser_factor * c * scales[(11, 10)] / m.wide_in
    shapes = [_shape(a["lens"], focal[key]) for key, a in found.items()]
    rows = []
    for key in sorted(focal, key=lambda k: (k[0], -1 if k[1] == WIDE_STOP else k[1])):
        reading, setting = key
        lens = lens_at(focal[key], shapes)
        rows.append({"reading": float(_label(reading)),
                     "set_by": WIDE_STOP if setting == WIDE_STOP else {ABSOLUTE_ZOOM: setting / 10},
                     "basis": MEASURED if key in found else INTERPOLATED,
                     "fx": round(lens["fx"], 3), "fy": round(lens["fy"], 3), "cx": round(lens["cx"], 3),
                     "cy": round(lens["cy"], 3), "skew": 0.0,
                     "dist": [float(f"{lens['k1']:.6g}"), float(f"{lens['k2']:.6g}"), 0.0, 0.0, 0.0]})
    return rows


def check_sessions(rows: Sequence[Mapping[str, Any]], sessions: Path) -> list[dict]:
    by_key = {(_tenths(r["reading"]), _tenths(r["set_by"][ABSOLUTE_ZOOM])): r
              for r in rows if isinstance(r["set_by"], Mapping)}
    out = []
    for folder in sorted(p for p in sessions.iterdir() if (p / "session.json").is_file()):
        s, objs, imgs = load_session(folder)
        request = (s.get("absolute_zoom_request") or {}).get("requested")
        key = (_tenths(s["zoom_reading"]), None if request is None else _tenths(request))
        entry = {"session": folder.name, "reading": s["zoom_reading"], "absolute_zoom": request, "views": len(imgs)}
        row = by_key.get(key)
        out.append({**entry, "result": "no row: no absolute zoom request recorded" if row is None or not imgs
                    else {"basis": row["basis"], **score(row, objs, imgs)}})
    return out


def leave_one_out(scales: Mapping[tuple[int, int], float], found: Mapping, c: float, m: Manifest,
                  sessions: Path) -> dict:
    """Each board-set anchor's views scored with its row rebuilt from the other anchors: how well interpolated
    centre and distortion hold between measured settings."""
    out = {}
    for key, a in found.items():
        if "session" not in a:
            continue
        rest = {k: v for k, v in found.items() if k != key}
        row = next(r for r in table_rows(scales, rest, c, m) if isinstance(r["set_by"], Mapping)
                   and (_tenths(r["reading"]), _tenths(r["set_by"][ABSOLUTE_ZOOM])) == key)
        _, objs, imgs = load_session(sessions / a["session"])
        out[_label(key[0])] = {"session": a["session"], **score(row, objs, imgs)}
    return out


def build(inputs: Inputs) -> tuple[list[dict], dict]:
    m = load_manifest(inputs.manifest)
    runs = [json.loads((inputs.sweep / run / "sweep.json").read_text(encoding="utf-8"))["records"]
            for run in m.sweep_runs]
    raw = sweep_scales(runs)
    scales = {k: float(np.mean(v)) for k, v in raw.items()}
    found, c, spread = anchors(inputs, m, scales)
    rows = table_rows(scales, found, c, m)
    report = {"what": "ZR10 lens table build (tools/hitl/zr10_lens_table.py)", "laser_factor": m.laser_factor,
              "wide_in": m.wide_in, "sweep_constant": c, "tilted_set_vs_constant_pct": spread,
              "sweep_direction_spread_pct_max": round(max((max(v) / min(v) - 1) * 100 for v in raw.values()), 3),
              "anchors": {f"{_label(k[0])}|{k[1]}": {"source": a["source"], **a["lens"]} for k, a in found.items()},
              "rows": len(rows), "measured_rows": sum(r["basis"] == MEASURED for r in rows),
              "leave_one_anchor_out": leave_one_out(scales, found, c, m, inputs.sessions),
              "sessions": check_sessions(rows, inputs.sessions)}
    return rows, report


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write-profile", action="store_true", help="rewrite the profile's reading_intrinsics")
    parser.add_argument("--report", type=Path, help="report path (default: private measurements folder)")
    args = parser.parse_args(argv)
    inputs = default_inputs()
    rows, report = build(inputs)
    out = args.report or inputs.sweep.parent / "zr10_lens_table_report.json"
    out.write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    scored = [s["result"] for s in report["sessions"] if isinstance(s["result"], Mapping)]
    print(f"{len(rows)} rows ({report['measured_rows']} measured); tilted sets vs sweep constant "
          f"{report['tilted_set_vs_constant_pct']} %; leave-one-anchor-out p95 "
          f"{ {k: v['p95_px'] for k, v in report['leave_one_anchor_out'].items()} } px; worst session p95 {max(r['p95_px'] for r in scored)} px; "
          f"report {out}")
    if args.write_profile:
        write_profile(PROFILE, section(rows, SIZE))
        print("wrote", PROFILE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
