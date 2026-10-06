"""Model construction and transactional resource rollback."""

from __future__ import annotations

from navpy.exception_groups import ExceptionGroup

from dataclasses import dataclass
from typing import Union, cast

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.appearance import (
    AppearanceEmbedder,
    AsyncAppearanceEmbedder,
    create_appearance_embedder,
)
from navpy.modules.vision.deep_search import (
    DeepSearchConfig,
    DeepSearchDetector,
    deep_search_config_from_settings,
)
from navpy.modules.vision.lost_poi_bridge import LostPoiBridge
from navpy.modules.vision.real_detector_config import RealDetectorConfig
from navpy.modules.vision.real_detector_ports import ModelResourceCloser
from navpy.modules.vision.real_detector_state import RuntimeMetrics
from navpy.modules.vision.poi_lock import PoiLock
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import (
    TrackerBackend,
    create_tracker_backend,
)
from navpy.modules.vision.yolo_detector import YoloDetector


@dataclass(frozen=True)
class RealDetectorModels:
    yolo: YoloDetector
    tracker: TrackerBackend
    identity: TrackIdentityResolver
    appearance: AsyncAppearanceEmbedder | None
    poi_lock: PoiLock
    bridge: LostPoiBridge
    deep_config: DeepSearchConfig | None
    deep_search: DeepSearchDetector | None


ModelResource = Union[
    YoloDetector,
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
    yolo: YoloDetector | None,
    deep_search: DeepSearchDetector | None,
) -> tuple[Exception, ...]:
    """Attempt every model rollback and return all failures."""
    seen: set[int] = set()
    errors = tuple(
        error
        for error in (
            _close_resource(logger, "tracker", tracker, seen),
            _close_resource(logger, "appearance", appearance, seen),
            _close_resource(logger, "yolo", yolo, seen),
            _close_resource(logger, "deep-search", deep_search, seen),
        )
        if error is not None
    )
    return errors


def build_models(
    logger: ILogger,
    config: RealDetectorConfig,
    metrics: RuntimeMetrics,
) -> RealDetectorModels:
    model = config.model
    yolo: YoloDetector | None = None
    tracker: TrackerBackend | None = None
    embedder: AppearanceEmbedder | None = None
    appearance: AsyncAppearanceEmbedder | None = None
    deep_search: DeepSearchDetector | None = None
    try:
        yolo = YoloDetector(
            model.model_path,
            imgsz=model.imgsz,
            conf=model.conf,
            device=model.device,
            classes=model.classes,
            logger=logger,
        )
        tracker = create_tracker_backend(
            model.tracker,
            detector_device=yolo.device,
            detector_conf=model.conf,
            logger=logger,
        )
        embedder = create_appearance_embedder(
            model.appearance,
            detector_device=yolo.device,
            logger=logger,
        )
        if embedder is not None:
            owned_embedder = embedder
            embedder = None
            appearance = AsyncAppearanceEmbedder(
                owned_embedder,
                logger=logger,
                on_batch=lambda count: metrics.bump("embeddings", count),
            )
        deep_config = deep_search_config_from_settings(model.deep_search)
        deep_search = (
            DeepSearchDetector(
                deep_config,
                default_model_path=model.model_path,
                default_device=model.device,
                default_classes=model.classes,
                logger=logger,
            )
            if deep_config is not None
            else None
        )
        return RealDetectorModels(
            yolo=yolo,
            tracker=tracker,
            identity=TrackIdentityResolver(),
            appearance=appearance,
            poi_lock=PoiLock(
                max_lost_frames=120,
                auto_lock=config.pipeline.auto_poi_lock,
            ),
            bridge=LostPoiBridge(),
            deep_config=deep_config,
            deep_search=deep_search,
        )
    except Exception as creation_error:
        cleanup_errors = close_model_resources(
            logger,
            tracker=tracker,
            appearance=(
                appearance
                if appearance is not None
                else cast(ModelResourceCloser | None, embedder)
            ),
            yolo=yolo,
            deep_search=deep_search,
        )
        if cleanup_errors:
            raise ExceptionGroup(
                "detector model construction and rollback failed",
                [creation_error, *cleanup_errors],
            ) from None
        raise


__all__ = [
    "RealDetectorModels",
    "build_models",
    "close_model_resources",
]
