"""Appearance embedding algorithms and backend construction."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

import numpy as np

from navpy.logger.logger_api import ILogger
from navpy.modules.vision.appearance_async import AsyncAppearanceEmbedder
from navpy.modules.vision.device import (
    DeviceT,
    free_cuda_cache,
    resolve_reid_device,
    uses_cuda_device,
)


class AppearanceEmbedder(Protocol):
    def embed(
        self,
        frame: np.ndarray,
        boxes_xyxy: Sequence[Sequence[float]],
    ) -> np.ndarray:
        """Return one L2-normalized embedding per box."""
        ...


class _FeatureBackend(Protocol):
    def get_features(self, boxes: np.ndarray, frame: np.ndarray) -> np.ndarray: ...


def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Return cosine distance for two already-normalized vectors."""
    if a is None or b is None:
        return 1.0
    left = np.asarray(a, dtype=np.float32).ravel()
    right = np.asarray(b, dtype=np.float32).ravel()
    if left.size == 0 or right.size == 0 or left.size != right.size:
        return 1.0
    return float(1.0 - float(np.dot(left, right)))


def l2_normalize(vectors: np.ndarray) -> np.ndarray:
    normalized = np.asarray(vectors, dtype=np.float32)
    if normalized.ndim == 1:
        normalized = normalized[None, :]
    norms = np.linalg.norm(normalized, axis=1, keepdims=True)
    return normalized / np.maximum(norms, 1e-12)


class BoxmotReidEmbedder:
    """On-demand appearance embedder backed by a BoxMOT ReID model."""

    def __init__(
        self,
        *,
        reid_device: DeviceT = "auto",
        detector_device: DeviceT = "auto",
        weights: str | None = None,
        half: bool = True,
        logger: ILogger | None = None,
    ) -> None:
        self._device = resolve_reid_device(reid_device, detector_device)
        self._half = bool(half) and uses_cuda_device(self._device)
        self._weights = weights
        self._logger = logger
        self._backend: _FeatureBackend | None = self._build_backend()

    def _build_backend(self) -> _FeatureBackend:
        from boxmot.reid.core import ReID

        reid = ReID(
            weights=self._weights,
            device=self._device,
            half=self._half,
        )
        return reid.get_backend()

    def embed(
        self,
        frame: np.ndarray,
        boxes_xyxy: Sequence[Sequence[float]],
    ) -> np.ndarray:
        boxes = np.asarray(boxes_xyxy, dtype=np.float32).reshape(-1, 4)
        if boxes.shape[0] == 0:
            return np.empty((0, 0), dtype=np.float32)
        if self._backend is None:
            raise RuntimeError("appearance backend is closed")
        features = self._backend.get_features(boxes, frame)
        return l2_normalize(np.asarray(features, dtype=np.float32))

    def close(self) -> None:
        self._backend = None
        free_cuda_cache(self._device)


def create_appearance_embedder(
    settings: dict | None,
    *,
    detector_device: DeviceT = "auto",
    logger: ILogger | None = None,
) -> AppearanceEmbedder | None:
    """Build the configured appearance backend, or return None when disabled."""
    if not isinstance(settings, dict) or not settings.get("enabled", False):
        return None
    return BoxmotReidEmbedder(
        reid_device=settings.get("reid_device", settings.get("device", "auto")),
        detector_device=detector_device,
        weights=settings.get("weights"),
        half=bool(settings.get("half", True)),
        logger=logger,
    )


__all__ = [
    "AppearanceEmbedder",
    "AsyncAppearanceEmbedder",
    "BoxmotReidEmbedder",
    "cosine_distance",
    "create_appearance_embedder",
    "l2_normalize",
]
