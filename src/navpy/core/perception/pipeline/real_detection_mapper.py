"""Tracked-object to public POI conversion for the real detector."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from navpy.core.ports.board_pose import BoardPoseSolver
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.detection import Detection
from navpy.core.perception.pipeline.real_confirmation_frame_selector import (
    ConfirmationFrameSelector,
)
from navpy.core.ports.tracks import TrackedObject
from navpy.core.perception.pipeline.real_detected_object_builder import (
    DetectedObjectBuilder,
    visual_attitude_components,
)
from navpy.core.perception.pipeline.real_board_pose import TrackBoardPose
from navpy.core.perception.pipeline.real_detector_state import ConfirmationFrameStore
from navpy.core.perception.pipeline.real_frame_association import RealFrameAssociation
from navpy.core.perception.pipeline.real_track_observation import map_track_observation


@dataclass(frozen=True)
class DetectionMappingConfig:
    reference_height_m: float = 2.0
    output_mode: str = "all"

    def __post_init__(self) -> None:
        normalized = self.output_mode.lower().strip()
        if normalized not in ("all", "locked"):
            raise ValueError("output_mode must be 'all' or 'locked'")
        object.__setattr__(self, "output_mode", normalized)


class DetectedObjectMapper:
    """Map tracks using only their immutable inference-frame association."""

    def __init__(
            self,
            config: DetectionMappingConfig,
            confirmation_frames: ConfirmationFrameStore,
            board_pose: BoardPoseSolver | None = None,
    ) -> None:
        self._config = config
        self._confirmation_frames = ConfirmationFrameSelector(
            confirmation_frames,
        )
        self._poi_builder = DetectedObjectBuilder(config.reference_height_m)
        self._board_pose = TrackBoardPose(board_pose)

    def convert(
            self,
            tracks: Sequence[TrackedObject],
            locked: Optional[TrackedObject],
            association: RealFrameAssociation,
            boards: Sequence[Detection] = (),
    ) -> list[DetectedObject]:
        mount_state = association.mount_state
        if mount_state is None:
            return []
        tracks_to_emit = _select_tracks(
            self._config.output_mode,
            tracks,
            locked,
        )
        if tracks_to_emit is None:
            return []

        pois: list[DetectedObject] = []
        visual_attitude = visual_attitude_components(
            association,
        )
        for track in tracks_to_emit:
            observation = map_track_observation(
                track,
                mount_state.k,
                mount_state.dist,
                association.frame_width,
                association.frame_height,
            )
            confirmation = self._confirmation_frames.select(
                track.id,
                association.frame,
                observation.bbox_cxcywh,
                observation.center_score,
            )
            pois.append(self._poi_builder.build(
                track,
                observation,
                confirmation,
                association,
                mount_state,
                visual_attitude,
                self._board_pose.for_track(track, boards, mount_state),
            ))

        self._confirmation_frames.evict_except({track.id for track in tracks})
        return pois


def _select_tracks(
        output_mode: str,
        tracks: Sequence[TrackedObject],
        locked: Optional[TrackedObject],
) -> list[TrackedObject] | None:
    if output_mode == "locked":
        return None if locked is None else [locked]
    return [track for track in tracks if track.is_confirmed]


__all__ = ["DetectedObjectMapper", "DetectionMappingConfig"]
