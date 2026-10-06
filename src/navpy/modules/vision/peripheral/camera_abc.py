from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping

from navpy.modules.vision.worker_failure import StaticFailureHealth


class CameraAbc(StaticFailureHealth, ABC):
    @abstractmethod
    def get_k(self):
        pass

    @abstractmethod
    def set_zoom(self, zoom):
        pass

    def get_zoom_key(self) -> str | None:
        """Return the selected intrinsic zoom key, when the camera has one."""
        return None

    def get_zoom_map(self) -> Mapping[object, object] | None:
        """Return intrinsic calibration entries, when available."""
        return None
