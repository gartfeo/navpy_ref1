"""Mutable camera intrinsic state with pluggable calibration policies."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.peripheral.camera_abc import CameraAbc
from navpy.modules.vision.peripheral.camera_intrinsics_calibration import (
    normalize_zoom_map,
)
from navpy.modules.vision.peripheral.camera_intrinsics_zoom import (
    CameraIntrinsicsZoomPolicyMixin,
    estimate_intrinsics,
)


class CameraIntrinsics(CameraIntrinsicsZoomPolicyMixin, CameraAbc):
    """Own the currently selected intrinsic matrix and distortion vector."""

    def __init__(
        self,
        fx: float | None = None,
        fy: float | None = None,
        cx: float | None = None,
        cy: float | None = None,
        *,
        skew: float = 0.0,
        dist: list[float] | None = None,
        image_width: int | None = None,
        image_height: int | None = None,
        zoom_map: dict[str, dict] | None = None,
    ) -> None:
        # These two fields are retained as the legacy compatibility boundary
        # used by mount calibration readers and existing camera integrations.
        self._zoom_map = normalize_zoom_map(
            zoom_map,
            image_width,
            image_height,
        )
        self._zoom = None
        self._k = None
        self._dist = None

        if self._zoom_map:
            self.set_zoom(next(iter(self._zoom_map)))
            return
        if fx is None or fy is None or cx is None or cy is None:
            raise ValueError(
                "fx, fy, cx, cy are required when zoom_map is not provided."
            )
        self._apply_intrinsics(
            fx,
            fy,
            cx,
            cy,
            skew,
            dist,
            image_width,
            image_height,
        )

    def _apply_intrinsics(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        skew: float,
        dist: list[float] | None,
        image_width: int | None,
        image_height: int | None,
    ) -> None:
        self._fx = float(fx)
        self._fy = float(fy)
        self._cx = float(cx)
        self._cy = float(cy)
        self._skew = float(skew)
        self.image_width = image_width
        self.image_height = image_height
        self._k = np.array(
            [
                [self._fx, self._skew * self._fx, self._cx],
                [0.0, self._fy, self._cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        if dist is not None and len(dist) >= 4:
            self._dist = np.array(dist[:5], dtype=np.float32)
        else:
            self._dist = np.zeros(5, dtype=np.float32)

    def get_k(self) -> np.ndarray:
        return self._k.copy()

    def get_dist(self) -> np.ndarray:
        """Return distortion coefficients [k1, k2, p1, p2, k3]."""
        if self._dist is not None:
            return self._dist.copy()
        return np.zeros(5, dtype=np.float32)

    def get_zoom_key(self) -> str | None:
        return None if self._zoom is None else str(self._zoom)

    def get_zoom_map(self) -> dict[str, dict]:
        return self._zoom_map

    def set_zoom(self, zoom: object) -> bool:
        if not self._zoom_map:
            self._zoom = zoom
            return True

        key = str(zoom)
        if key in self._zoom_map:
            entry = self._zoom_map[key]
        else:
            entry = self._estimate_intrinsics(zoom)
        if entry is None:
            return False
        self._apply_entry(entry)
        self._zoom = key
        return True

    def _apply_entry(self, entry: dict[str, object]) -> None:
        self._apply_intrinsics(
            entry["fx"],
            entry["fy"],
            entry["cx"],
            entry["cy"],
            entry["skew"],
            entry["dist"],
            entry["image_width"],
            entry["image_height"],
        )

    def _estimate_intrinsics(
        self,
        zoom: object,
    ) -> dict[str, object] | None:
        return estimate_intrinsics(
            self._zoom_map,
            zoom,
            scale=self._scale_entry,
            interpolate=self._lerp_entry,
        )

    def is_valid(self, u: float | None, v: float | None) -> bool:
        if u is None or v is None:
            return False
        if self.image_width is None or self.image_height is None:
            return True
        return 0 <= u <= self.image_width and 0 <= v <= self.image_height

    def raise_if_failed(self) -> None:
        """Static intrinsics have no background worker to report."""


__all__ = ["CameraIntrinsics"]
