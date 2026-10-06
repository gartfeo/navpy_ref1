"""Stateless public diagnostics facet for Detector."""

from __future__ import annotations

import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_public_ports import RealDiagnosticsParts


class DetectorDiagnosticsFacet:
    _parts: RealDiagnosticsParts

    def get_overlay_tracks(self) -> list[TrackedObject]:
        return self._parts.diagnostics.get_overlay_tracks()

    @property
    def tracking_fps(self) -> float:
        return self._parts.diagnostics.tracking_fps

    def get_raw_frame(self) -> np.ndarray | None:
        return self._parts.diagnostics.get_raw_frame()

    def get_debug_frame(self) -> np.ndarray | None:
        return self._parts.diagnostics.get_debug_frame()

    def ui_step(self) -> bool:
        return self._parts.diagnostics.ui_step()

    def stats(self) -> dict[str, float]:
        return self._parts.diagnostics.stats()


__all__ = ["DetectorDiagnosticsFacet"]
