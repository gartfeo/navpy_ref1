"""Names of the per-frame inference backends a real detector can run.

Dependency-free so the CLI layer can offer the choices without importing
OpenCV, ultralytics, or torch.

``charuco`` is the product default: a deterministic fiducial-board detector
for the cooperative dock reference. ``yolo`` runs an explicitly supplied
ultralytics model (``--detector-model-path``) and is opt-in only. ``rfdetr``
runs RF-DETR weights (``--detector-model-path``) with the profile's
``detector.rfdetr`` block (HITL step 23b, the whole-dock detector); it is
opt-in only. Developer bench models such as face detectors are never selected
implicitly.
"""

from __future__ import annotations

from typing import Literal

DetectorBackend = Literal["charuco", "yolo", "rfdetr"]

CHARUCO_BACKEND: DetectorBackend = "charuco"
YOLO_BACKEND: DetectorBackend = "yolo"
RFDETR_BACKEND: DetectorBackend = "rfdetr"
DETECTOR_BACKENDS: tuple[DetectorBackend, ...] = (CHARUCO_BACKEND, YOLO_BACKEND, RFDETR_BACKEND)
_MODEL_PATH_BACKENDS = frozenset({YOLO_BACKEND, RFDETR_BACKEND})
DEFAULT_DETECTOR_BACKEND: DetectorBackend = CHARUCO_BACKEND


def backend_requires_model_path(backend: str) -> bool:
    """True when the backend loads weights from ``--detector-model-path``."""
    return backend in _MODEL_PATH_BACKENDS


def validate_detector_backend(backend: str) -> DetectorBackend:
    if backend not in DETECTOR_BACKENDS:
        raise ValueError(
            f"Unknown detector backend {backend!r}; "
            f"expected one of {DETECTOR_BACKENDS}"
        )
    return backend  # type: ignore[return-value]


__all__ = [
    "CHARUCO_BACKEND",
    "DEFAULT_DETECTOR_BACKEND",
    "DETECTOR_BACKENDS",
    "DetectorBackend",
    "RFDETR_BACKEND",
    "YOLO_BACKEND",
    "backend_requires_model_path",
    "validate_detector_backend",
]
