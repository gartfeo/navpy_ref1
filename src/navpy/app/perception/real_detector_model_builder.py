"""Real-detector model construction: backend choice, rollback on failure."""

from __future__ import annotations

from collections.abc import Callable
from typing import cast

from navpy.core.kernel.exception_groups import ExceptionGroup
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.appearance_embedding import AppearanceEmbedder
from navpy.modules.vision.appearance_async import AsyncAppearanceEmbedder
from navpy.modules.vision.appearance import create_appearance_embedder
from navpy.modules.vision.deep_search import (
    DeepSearchConfig,
    DeepSearchDetector,
    deep_search_config_from_settings,
)
from navpy.modules.vision.lost_poi_bridge import LostPoiBridge
from navpy.modules.vision.charuco_detector import CharucoBoardDetector
from navpy.core.kernel.config.detector_backend import (
    CHARUCO_BACKEND,
    YOLO_BACKEND,
)
from navpy.modules.vision.real_detector_config import (
    DetectorModelConfig,
    RealDetectorConfig,
)
from navpy.core.ports.detection import FrameDetector
from navpy.modules.vision.real_detector_ports import ModelResourceCloser
from navpy.modules.vision.real_detector_runtime_state import RuntimeMetrics
from navpy.modules.vision.poi_lock import PoiLock
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import TrackerBackend
from navpy.app.perception.tracker_backend_factory import create_tracker_backend
from navpy.modules.vision.yolo_detector import YoloDetector
from navpy.modules.vision.real_detector_models import RealDetectorModels, close_model_resources


def _build_yolo(model: DetectorModelConfig, logger: ILogger) -> FrameDetector:
    return YoloDetector(
        model.model_path,
        imgsz=model.imgsz,
        conf=model.conf,
        device=model.device,
        classes=model.classes,
        logger=logger,
    )


def _build_charuco(
    model: DetectorModelConfig,
    logger: ILogger,
) -> FrameDetector:
    return CharucoBoardDetector(
        model.charuco_board,
        min_confidence=model.conf,
        logger=logger,
    )


_FRAME_DETECTOR_BUILDERS: dict[
    str,
    Callable[[DetectorModelConfig, ILogger], FrameDetector],
] = {
    CHARUCO_BACKEND: _build_charuco,
    YOLO_BACKEND: _build_yolo,
}

# Deep search reruns a (higher-resolution) YOLO model; a fiducial board
# detector has no such model, so the profile's deep_search block is inert.
_DEEP_SEARCH_BACKENDS = frozenset({YOLO_BACKEND})


def _deep_search_config(
    logger: ILogger,
    model: DetectorModelConfig,
) -> DeepSearchConfig | None:
    deep_config = deep_search_config_from_settings(model.deep_search)
    if deep_config is not None and model.backend not in _DEEP_SEARCH_BACKENDS:
        logger.warning(
            f"Detector: deep_search is YOLO-only; disabled for the "
            f"'{model.backend}' backend"
        )
        return None
    return deep_config


def build_models(
    logger: ILogger,
    config: RealDetectorConfig,
    metrics: RuntimeMetrics,
) -> RealDetectorModels:
    model = config.model
    frame_detector: FrameDetector | None = None
    tracker: TrackerBackend | None = None
    embedder: AppearanceEmbedder | None = None
    appearance: AsyncAppearanceEmbedder | None = None
    deep_search: DeepSearchDetector | None = None
    try:
        frame_detector = _FRAME_DETECTOR_BUILDERS[model.backend](model, logger)
        tracker = create_tracker_backend(
            model.tracker,
            detector_device=frame_detector.device,
            detector_conf=model.conf,
            logger=logger,
        )
        embedder = create_appearance_embedder(
            model.appearance,
            detector_device=frame_detector.device,
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
        deep_config = _deep_search_config(logger, model)
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
            frame_detector=frame_detector,
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
            frame_detector=frame_detector,
            deep_search=deep_search,
        )
        if cleanup_errors:
            raise ExceptionGroup(
                "detector model construction and rollback failed",
                [creation_error, *cleanup_errors],
            ) from None
        raise
