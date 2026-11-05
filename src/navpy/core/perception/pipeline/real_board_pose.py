"""Attach a frame's board pose to the track it belongs to (HITL step 22).

A board detection carries its corners only on the frame that found them, so
the pose is solved for a track that the same frame updated and whose box
matches that detection. The pose uses the frame's own optics (the captured
mount state's intrinsics and distortion).
"""

from __future__ import annotations

from collections.abc import Sequence

from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.ports.board_pose import BoardPoseResult, BoardPoseSolver
from navpy.core.ports.detection import Detection
from navpy.core.ports.tracks import TrackedObject

# A track and a board detection are the same board when their boxes overlap
# this much; the tracker's box follows the detection closely on its own frame.
BOARD_TRACK_MIN_IOU = 0.5


class TrackBoardPose:
    """Solve the board pose for a track updated by this frame's board."""

    def __init__(self, solver: BoardPoseSolver | None) -> None:
        self._solver = solver

    def for_track(
        self,
        track: TrackedObject,
        boards: Sequence[Detection],
        mount_state: CameraMountFrameState,
    ) -> BoardPoseResult | None:
        if self._solver is None or track.missed != 0:
            return None
        match = _best_match(track, boards)
        if match is None or match.board is None:
            return None
        return self._solver.solve(match.board, mount_state.k, mount_state.dist)


def _best_match(
    track: TrackedObject,
    boards: Sequence[Detection],
) -> Detection | None:
    best = None
    best_iou = BOARD_TRACK_MIN_IOU
    for board in boards:
        if board.board is None or board.class_id != track.class_id:
            continue
        overlap = _iou(
            (track.cx, track.cy, track.w, track.h),
            (board.cx, board.cy, board.w, board.h),
        )
        if overlap >= best_iou:
            best, best_iou = board, overlap
    return best


def _iou(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    ax1, ay1 = first[0] - first[2] / 2.0, first[1] - first[3] / 2.0
    ax2, ay2 = ax1 + first[2], ay1 + first[3]
    bx1, by1 = second[0] - second[2] / 2.0, second[1] - second[3] / 2.0
    bx2, by2 = bx1 + second[2], by1 + second[3]
    width = min(ax2, bx2) - max(ax1, bx1)
    height = min(ay2, by2) - max(ay1, by1)
    if width <= 0.0 or height <= 0.0:
        return 0.0
    inter = width * height
    union = first[2] * first[3] + second[2] * second[3] - inter
    return inter / union if union > 0.0 else 0.0


__all__ = ["BOARD_TRACK_MIN_IOU", "TrackBoardPose"]
