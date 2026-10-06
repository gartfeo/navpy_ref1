"""Geometric association, validation metrics, and text reporting."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from scripts.python.validate_tracking_config import CAR


EmitLine = Callable[[str], None]


def append_metric_line(
    message: str = "",
    *,
    metrics_path: str,
) -> None:
    """Print and append one report line without a persistent file handle."""
    print(message, flush=True)
    with open(metrics_path, "a", encoding="utf-8") as report:
        report.write(message + "\n")


def iou(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    ax1 = first[0] - first[2] / 2
    ay1 = first[1] - first[3] / 2
    ax2 = first[0] + first[2] / 2
    ay2 = first[1] + first[3] / 2
    bx1 = second[0] - second[2] / 2
    by1 = second[1] - second[3] / 2
    bx2 = second[0] + second[2] / 2
    by2 = second[1] + second[3] / 2
    intersection_width = max(0, min(ax2, bx2) - max(ax1, bx1))
    intersection_height = max(0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_width * intersection_height
    union = (
        first[2] * first[3]
        + second[2] * second[3]
        - intersection
    )
    return intersection / union if union > 0 else 0.0


def associate(
    frames: list[list[tuple[int, tuple[float, ...], int]]],
    coast: int = 10,
    thr: float = 0.3,
) -> list[dict]:
    active: list[dict] = []
    tracklets: list[dict] = []
    for frame_index, snapshots in enumerate(frames):
        used = [False] * len(snapshots)
        for tracklet in active:
            best, best_index = thr, -1
            for index, snapshot in enumerate(snapshots):
                if used[index] or snapshot[2] != tracklet["cls"]:
                    continue
                overlap = iou(tracklet["bb"], snapshot[1])
                if overlap > best:
                    best, best_index = overlap, index
            if best_index >= 0:
                snapshot = snapshots[best_index]
                used[best_index] = True
                tracklet["bb"], tracklet["last"] = (
                    snapshot[1],
                    frame_index,
                )
                tracklet["pts"].append(
                    (frame_index, snapshot[0], snapshot[1])
                )
        active = [
            tracklet
            for tracklet in active
            if frame_index - tracklet["last"] <= coast
        ]
        for index, snapshot in enumerate(snapshots):
            if not used[index]:
                active.append(
                    {
                        "bb": snapshot[1],
                        "last": frame_index,
                        "cls": snapshot[2],
                        "pts": [(frame_index, snapshot[0], snapshot[1])],
                    }
                )
                tracklets.append(active[-1])
    return tracklets


def tracklet_stats(tracklet: dict) -> tuple[int, int, float, float, float, float]:
    xs = [point[2][0] for point in tracklet["pts"]]
    ys = [point[2][1] for point in tracklet["pts"]]
    return (
        len(set(point[1] for point in tracklet["pts"])),
        len(tracklet["pts"]),
        float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0])),
        float(np.hypot(np.std(xs), np.std(ys))),
        float(np.mean(xs)),
        float(np.mean(ys)),
    )


def report_recognition(
    car_counts: list[int],
    frame_count: int,
    emit: EmitLine,
) -> None:
    visible_frames = sum(count > 0 for count in car_counts)
    emit("\n[1] RECOGNITION")
    emit(
        f"  frames with >=1 car: {visible_frames}/{frame_count} "
        f"({100 * visible_frames / max(1, frame_count):.0f}%) | "
        f"avg cars/frame {np.mean(car_counts):.1f} "
        f"| max {max(car_counts)}"
    )


def report_identity_stability(
    tracklets: list[dict],
    fps: float,
    frame_width: int,
    frame_height: int,
    emit: EmitLine,
) -> None:
    cars = [
        tracklet
        for tracklet in tracklets
        if tracklet["cls"] == CAR
        and len(tracklet["pts"]) >= int(1.5 * fps)
    ]
    parked = [
        (tracklet, tracklet_stats(tracklet))
        for tracklet in cars
        if tracklet_stats(tracklet)[3] < 18
    ]
    moving = [
        (tracklet, tracklet_stats(tracklet))
        for tracklet in cars
        if tracklet_stats(tracklet)[2] > 120
    ]
    emit(
        "\n[2] ID STABILITY "
        "(independent geometric GT; distinct stable ids per car)"
    )
    emit(f"  PARKED cars (different positions): n={len(parked)}")
    for _, stats in sorted(parked, key=lambda item: -item[1][1])[:6]:
        emit(
            f"    pos=({stats[4]:.0f},{stats[5]:.0f}) "
            f"present={stats[1]}f distinct_ids={stats[0]} "
            f"-> {'STABLE' if stats[0] == 1 else 'churn'}"
        )
    emit(f"  MOVING cars (different positions): n={len(moving)}")
    for _, stats in sorted(moving, key=lambda item: -item[1][2])[:6]:
        emit(
            f"    travel={stats[2]:.0f}px present={stats[1]}f "
            f"distinct_ids={stats[0]} "
            f"-> {'STABLE' if stats[0] == 1 else 'churn'}"
        )
    parked_ids = [stats[0] for _, stats in parked]
    moving_ids = [stats[0] for _, stats in moving]
    emit(
        f"  parked single-id: "
        f"{sum(value == 1 for value in parked_ids)}/{len(parked_ids)} | "
        f"moving single-id: "
        f"{sum(value == 1 for value in moving_ids)}/{len(moving_ids)}"
    )
    parked_interior = [
        stats[0]
        for _, stats in parked
        if _is_interior(stats, frame_width, frame_height)
    ]
    moving_interior = [
        stats[0]
        for _, stats in moving
        if _is_interior(stats, frame_width, frame_height)
    ]
    emit(
        f"  INTERIOR-only parked single-id: "
        f"{sum(value == 1 for value in parked_interior)}"
        f"/{len(parked_interior)} | "
        f"moving single-id: "
        f"{sum(value == 1 for value in moving_interior)}"
        f"/{len(moving_interior)}"
    )


def _is_interior(
    stats: tuple[int, int, float, float, float, float],
    frame_width: int,
    frame_height: int,
) -> bool:
    return (
        120 < stats[4] < frame_width - 120
        and 80 < stats[5] < frame_height - 80
    )


def find_occ(
    tracklets: list[dict],
    bus_boxes: list[list[tuple[float, ...]]],
    fps: float,
) -> dict | None:
    best = None
    for tracklet in tracklets:
        if (
            tracklet["cls"] != CAR
            or len(tracklet["pts"]) < int(0.6 * fps)
        ):
            continue
        for index in range(1, len(tracklet["pts"])):
            before_frame, before_id, before_box = tracklet["pts"][index - 1]
            after_frame, after_id, after_box = tracklet["pts"][index]
            gap = after_frame - before_frame
            if gap < 4:
                continue
            covered = any(
                iou(bus_box, before_box) > 0.2
                or iou(bus_box, after_box) > 0.2
                for gap_frame in range(before_frame + 1, after_frame)
                for bus_box in bus_boxes[gap_frame]
            )
            if covered and (best is None or gap > best["gap"]):
                best = {
                    "fb": before_frame,
                    "fa": after_frame,
                    "gap": gap,
                    "ib": before_id,
                    "ia": after_id,
                    "bb": before_box,
                    "ba": after_box,
                }
    return best


__all__ = [
    "append_metric_line",
    "associate",
    "find_occ",
    "iou",
    "report_identity_stability",
    "report_recognition",
    "tracklet_stats",
]
