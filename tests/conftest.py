"""Shared test fixtures and helpers."""
from types import SimpleNamespace

import numpy as np

from navpy.modules.vision.peripheral.camera_abc import CameraAbc


class FovCamera(CameraAbc):
    """Test camera with configurable intrinsics from FOV. Supports dynamic zoom."""

    def __init__(
            self,
            image_width: int = 1920,
            image_height: int = 1080,
            fov: float = 60.0,
            sensor_width: float = 4.8,
            sensor_height: float = 3.6,
    ):
        self.image_width = image_width
        self.image_height = image_height
        self._sensor_width = sensor_width
        self._sensor_height = sensor_height
        self.cx = image_width / 2
        self.cy = image_height / 2
        self.zoom = round(sensor_height / (2 * np.tan(np.radians(fov / 2))))

    def refresh(self):
        pass

    def get_k(self):
        focal_distance = self.zoom
        fx = focal_distance * (self.image_width / self._sensor_width)
        fy = focal_distance * (self.image_height / self._sensor_height)

        return np.array([[fx, 0, self.cx],
                         [0, fy, self.cy],
                         [0, 0, 1]])

    def set_zoom(self, zoom):
        self.zoom = zoom

    def is_valid(self, u, v):
        return u is not None and v is not None and 0 <= u <= self.image_width and 0 <= v <= self.image_height


def create_test_camera(
        image_width: int = 1920,
        image_height: int = 1080,
        fov: float = 60.0,
        sensor_width: float = 4.8,
        sensor_height: float = 3.6,
) -> FovCamera:
    """Create FovCamera from FOV and sensor dimensions for testing."""
    return FovCamera(image_width, image_height, fov, sensor_width, sensor_height)


def create_mock_args():
    """Create mock args with required attributes for NavigationPoiArgs."""
    args = SimpleNamespace()
    # NavigationPoiArgs.PARAMS defaults
    args.AAS_TARG_WPS = '4'
    args.AAS_TARG_ALT = 150
    args.use_terrain = False
    return args
