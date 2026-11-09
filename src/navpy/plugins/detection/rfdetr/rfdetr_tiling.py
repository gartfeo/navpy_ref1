"""How a frame is fed to RF-DETR: squashed whole, or as fixed native-resolution tiles (hitl_dock_detector.md section 3).

``squash``: the whole frame is one input, resized to the model's square (rfdetr's own predict()); a 1920 x 1080 frame
is scaled by 704/1920 across and 704/1080 down.

``tiles``: a fixed grid of input-size squares at the frame's own resolution (scale 1), the fewest per axis that cover
it, spaced evenly so neighbours overlap. A box is mapped back to the full frame by its tile's offset. A box cut by an
inner tile edge (one that is not a frame edge) is dropped when an uncut box of the same class from another tile covers
most of it: that one holds the whole object (two cut boxes never drop each other; the merge below keeps the best). The rest are merged: of boxes of one class overlapping by more than
``MERGE_IOU``, the best scored is kept.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Boxes of one class overlapping more than this are one object seen in two tiles (my value).
MERGE_IOU = 0.5
# A box edge within this many pixels of an inner tile edge is cut by it (the decode clips to the tile).
EDGE_PX = 1.0
# Of a cut box, at least this share inside a box from another tile means that tile holds the object (my value).
COVER_SHARE = 0.5


@dataclass(frozen=True)
class Tile:
    x0: int
    y0: int
    width: int
    height: int


def _starts(length: int, side: int) -> list[int]:
    if length <= side:
        return [0]
    count = math.ceil(length / side)
    return [round(i * (length - side) / (count - 1)) for i in range(count)]


def tiles(width: int, height: int, side: int) -> list[Tile]:
    """The fixed grid of side x side tiles (or the whole axis when it is shorter) covering the frame."""
    return [Tile(x, y, min(side, width), min(side, height))
            for y in _starts(height, side) for x in _starts(width, side)]


def _xyxy(rows: np.ndarray) -> np.ndarray:
    return np.stack([rows[:, 0] - rows[:, 2] / 2, rows[:, 1] - rows[:, 3] / 2,
                     rows[:, 0] + rows[:, 2] / 2, rows[:, 1] + rows[:, 3] / 2], axis=1)


def _cut(box: np.ndarray, tile: Tile, width: int, height: int) -> bool:
    x0, y0, x1, y1 = box
    return ((tile.x0 > 0 and x0 <= tile.x0 + EDGE_PX) or (tile.y0 > 0 and y0 <= tile.y0 + EDGE_PX)
            or (tile.x0 + tile.width < width and x1 >= tile.x0 + tile.width - EDGE_PX)
            or (tile.y0 + tile.height < height and y1 >= tile.y0 + tile.height - EDGE_PX))


def _inter(a: np.ndarray, b: np.ndarray) -> float:
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    return max(0.0, float(w)) * max(0.0, float(h))


def _area(a: np.ndarray) -> float:
    return max(0.0, float(a[2] - a[0])) * max(0.0, float(a[3] - a[1]))


def merge_tiles(per_tile: list[np.ndarray], grid: list[Tile], width: int, height: int) -> np.ndarray:
    """Each tile's rows (tile pixels) -> one frame's rows (frame pixels), cut duplicates dropped, overlaps merged."""
    rows, owners = [], []
    for index, (tile_rows, tile) in enumerate(zip(per_tile, grid)):
        for row in np.asarray(tile_rows, np.float32).reshape(-1, 6):
            moved = row.copy()
            moved[0] += tile.x0
            moved[1] += tile.y0
            rows.append(moved)
            owners.append(index)
    if not rows:
        return np.zeros((0, 6), np.float32)
    table = np.stack(rows)
    boxes = _xyxy(table)
    cut = [_cut(boxes[i], grid[owners[i]], width, height) for i in range(len(table))]
    keep = []
    for i in range(len(table)):
        if cut[i] and any(
                not cut[j] and owners[j] != owners[i] and table[j, 5] == table[i, 5]
                and _inter(boxes[i], boxes[j]) >= COVER_SHARE * _area(boxes[i]) for j in range(len(table))):
            continue
        keep.append(i)
    keep.sort(key=lambda i: -float(table[i, 4]))
    chosen: list[int] = []
    for i in keep:
        if all(table[j, 5] != table[i, 5] or _iou(boxes[i], boxes[j]) <= MERGE_IOU for j in chosen):
            chosen.append(i)
    return table[chosen].astype(np.float32) if chosen else np.zeros((0, 6), np.float32)


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = _inter(a, b)
    union = _area(a) + _area(b) - inter
    return inter / union if union > 0.0 else 0.0


__all__ = ["COVER_SHARE", "EDGE_PX", "MERGE_IOU", "Tile", "merge_tiles", "tiles"]
