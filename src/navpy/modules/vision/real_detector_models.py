"""Model construction and transactional resource rollback."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.perception.tracking.appearance_async import AsyncAppearanceEmbedder
from navpy.core.perception.detection.deep_search import (
    DeepSearchConfig,
    DeepSearchDetector,
)
from navpy.core.perception.tracking.lost_poi_bridge import LostPoiBridge
from navpy.core.ports.detection import FrameDetector
from navpy.modules.vision.real_detector_ports import ModelResourceCloser
from navpy.core.perception.tracking.poi_lock import PoiLock
from navpy.core.perception.tracking.track_identity import TrackIdentityResolver
from navpy.core.ports.tracker_backends import TrackerBackend


@dataclass(frozen=True)
class RealDetectorModels:
    frame_detector: FrameDetector
    tracker: TrackerBackend
    identity: TrackIdentityResolver
    appearance: AsyncAppearanceEmbedder | None
    poi_lock: PoiLock
    bridge: LostPoiBridge
    deep_config: DeepSearchConfig | None
    deep_search: DeepSearchDetector | None


ModelResource = Union[
    FrameDetector,
    TrackerBackend,
    ModelResourceCloser,
    AsyncAppearanceEmbedder,
    DeepSearchDetector,
]


def _close_resource(
    logger: ILogger,
    name: str,
    resource: ModelResource | None,
    seen: set[int],
) -> Exception | None:
    if resource is None or id(resource) in seen:
        return None
    seen.add(id(resource))
    try:
        resource.close()
    except Exception as error:
        try:
            logger.warning(
                f"Detector: {name} construction rollback failed: {error}"
            )
        except Exception:
            pass
        return error
    return None


def close_model_resources(
    logger: ILogger,
    *,
    tracker: TrackerBackend | None,
    appearance: AsyncAppearanceEmbedder | ModelResourceCloser | None,
    frame_detector: FrameDetector | None,
    deep_search: DeepSearchDetector | None,
) -> tuple[Exception, ...]:
    """Attempt every model rollback and return all failures."""
    seen: set[int] = set()
    errors = tuple(
        error
        for error in (
            _close_resource(logger, "tracker", tracker, seen),
            _close_resource(logger, "appearance", appearance, seen),
            _close_resource(logger, "frame detector", frame_detector, seen),
            _close_resource(logger, "deep-search", deep_search, seen),
        )
        if error is not None
    )
    return errors


__all__ = [
    "RealDetectorModels",
    "close_model_resources",
]
