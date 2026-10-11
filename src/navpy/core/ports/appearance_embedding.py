"""Appearance-embedding contract and the vector metrics trackers share."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

import numpy as np


class AppearanceEmbedder(Protocol):
    def embed(
        self,
        frame: np.ndarray,
        boxes_xyxy: Sequence[Sequence[float]],
    ) -> np.ndarray:
        """Return one L2-normalized embedding per box."""
        ...


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
