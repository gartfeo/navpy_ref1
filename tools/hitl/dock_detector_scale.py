"""Detection against box size for the whole-dock detector, squash against tiles (HITL step 23b; hitl_dock_detector.md
section 6 items 1 and 5).

A proxy until the Unreal scene has its car: real ZR10 frames of cars (the owner's gimbal-controller clips, private,
never committed) are shrunk by a factor k to emulate range, mirrored out to the full 1920 x 1080 frame round the
centre copy, and run through navpy's rfdetr detector in each input mode. Each reference car of the clip's fixed
reference (``dets_ref.npz``, COCO car only) shrunk with the frame counts as found when a detection's centre lies in
its box and their IoU is at least ``MATCH_IOU``. Recall is reported by the reference box's smaller side in the
detector's own input pixels, with Wilson 95 % intervals, for every reference car and for isolated ones (no other
reference box within ``ISOLATION_GAP`` box sizes: nearer the series' one car on open ground), and the time per frame
of each mode on this PC's GPU.

Not the design's held-out Unreal set: real video, shrunk (no atmosphere, no encoder at the far size), the
reference's own misses not counted, no false-box rate. It answers the tiling question only.

    python -s -m tools.hitl.dock_detector_scale --clips DIR --weights rf-detr-large.pth --out RESULT.json
"""
from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Iterator, Sequence
from pathlib import Path

import cv2
import numpy as np

from navpy.core.ports.detection import Detection
from navpy.plugins.detection.rfdetr.rfdetr_detector import RfDetrDetector
from navpy.plugins.detection.rfdetr.rfdetr_runtime import TorchRfDetrRuntime
from navpy.plugins.detection.rfdetr.rfdetr_settings import INPUT_MODES, RfDetrSettings

WIDTH, HEIGHT = 1920, 1080
COCO_CAR = 2
CONFIDENCE = 0.45                                   # gimbal-controller's tracking threshold for RF-DETR-L
MATCH_IOU = 0.3                                     # my value: small boxes are a few pixels off
SCALES = (1.0, 0.5, 0.35, 0.25, 0.18, 0.13, 0.1)    # my values: 1x down to about a tenth
ISOLATION_GAP = 1.0                                 # my value: a gap of one box size to any other reference car
BINS_PX = (0, 6, 8, 10, 12, 14, 16, 20, 24, 32, 48, 10_000)


def wilson(found: int, total: int, z: float = 1.96) -> tuple[float, float]:
    if total == 0:
        return math.nan, math.nan
    p = found / total
    centre = (p + z * z / (2 * total)) / (1 + z * z / total)
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / (1 + z * z / total)
    return centre - half, centre + half


def shrunk(frame: np.ndarray, k: float) -> tuple[np.ndarray, float, float]:
    """The frame shrunk by k at the centre of a full frame mirrored round it; the centre copy's offset."""
    small = cv2.resize(frame, (max(1, round(WIDTH * k)), max(1, round(HEIGHT * k))), interpolation=cv2.INTER_AREA)
    h, w = small.shape[:2]
    left, top = (WIDTH - w) // 2, (HEIGHT - h) // 2
    canvas = np.pad(small, ((top, HEIGHT - h - top), (left, WIDTH - w - left), (0, 0)), mode="symmetric")
    return canvas, float(left), float(top)


def _iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    inter = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(0.0, min(ay1, by1) - max(ay0, by0))
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def found(ref: Sequence[float], detections: Sequence[Detection]) -> bool:
    return any(abs(d.cx - ref[0]) <= ref[2] / 2 and abs(d.cy - ref[1]) <= ref[3] / 2
               and _iou(ref, (d.cx, d.cy, d.w, d.h)) >= MATCH_IOU for d in detections)


def isolated(rows: np.ndarray) -> list[bool]:
    """Whether each reference box keeps a gap of ISOLATION_GAP of its own size to every other one."""
    out = []
    for i, (cx, cy, w, h, _, _) in enumerate(rows):
        near = [abs(cx - o[0]) < (w + o[2]) / 2 + ISOLATION_GAP * w and abs(cy - o[1]) < (h + o[3]) / 2
                + ISOLATION_GAP * h for j, o in enumerate(rows) if j != i]
        out.append(not any(near))
    return out


def _frames(clip: Path, per_clip: int) -> Iterator[tuple[Path, np.ndarray]]:
    ref = np.load(clip / "dets_ref.npz")
    offsets = np.concatenate([[0], np.cumsum(ref["n"])])
    picks = np.linspace(0, len(ref["index"]) - 1, per_clip).round().astype(int)
    for k in picks:
        rows = ref["dets"][offsets[k]:offsets[k + 1]]
        yield clip / "frames" / f"{int(ref['index'][k])}.jpg", rows[rows[:, 5] == COCO_CAR]


def measure(clips: Path, weights: str, per_clip: int) -> dict:
    runtime = TorchRfDetrRuntime(weights, "large", "auto")
    settings = {mode: RfDetrSettings("large", 704, ((COCO_CAR, 0),), mode) for mode in INPUT_MODES}
    detectors = {mode: RfDetrDetector(runtime, settings[mode], CONFIDENCE) for mode in INPUT_MODES}
    samples = {mode: [] for mode in INPUT_MODES}
    times = {mode: [] for mode in INPUT_MODES}
    for clip in sorted(p for p in clips.iterdir() if (p / "dets_ref.npz").is_file()):
        for path, refs in _frames(clip, per_clip):
            frame = cv2.imread(str(path))
            if frame is None or not len(refs):
                continue
            for k in SCALES:
                canvas, left, top = shrunk(frame, k)
                for mode, detector in detectors.items():
                    start = time.perf_counter()
                    detections = detector.detect(canvas)
                    times[mode].append(time.perf_counter() - start)
                    sx, sy = settings[mode].input_scale(WIDTH, HEIGHT)
                    for (cx, cy, w, h, _, _), alone in zip(refs, isolated(refs)):
                        box = (left + cx * k, top + cy * k, w * k, h * k)
                        samples[mode].append((min(box[2] * sx, box[3] * sy), found(box, detections), alone))
    runtime.close()
    return {"confidence": CONFIDENCE, "match_iou": MATCH_IOU, "scales": SCALES, "per_clip": per_clip,
            "isolation_gap": ISOLATION_GAP,
            "modes": {mode: _table(samples[mode], times[mode]) for mode in INPUT_MODES}}


def _bins(samples: list) -> list[dict]:
    rows = []
    for low, high in zip(BINS_PX, BINS_PX[1:]):
        hits = [ok for side, ok in samples if low <= side < high]
        lo, hi = wilson(sum(hits), len(hits))
        rows.append({"input_px": [low, high], "n": len(hits), "recall": round(sum(hits) / len(hits), 4)
                     if hits else None, "ci95": [round(lo, 4), round(hi, 4)] if hits else None})
    return rows


def _table(samples: list, times: list) -> dict:
    ms = sorted(1000 * t for t in times[3:])
    return {"bins": _bins([(side, ok) for side, ok, _ in samples]),
            "isolated_bins": _bins([(side, ok) for side, ok, alone in samples if alone]),
            "ms_per_frame_p50": round(ms[len(ms) // 2], 1) if ms else None,
            "ms_per_frame_p95": round(ms[int(0.95 * (len(ms) - 1))], 1) if ms else None}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.dock_detector_scale", description=__doc__.split("\n")[0])
    parser.add_argument("--clips", required=True, type=Path)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--per-clip", type=int, default=8)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(argv)
    result = measure(args.clips, args.weights, args.per_clip)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
