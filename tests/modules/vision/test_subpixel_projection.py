"""Projection keeps sub-pixel precision; in-frame is the continuous sensor rectangle.

Rounding calc_uv to whole pixels put a +-0.5 px error on every sim frame, and
the vision law's two-frame bearing difference turned it into LOS-rate spikes at
20 ms frame gaps. A real detector reports sub-pixel corners, so the projection
must too. The frame-edge rule is continuous: [0, width] x [0, height].
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from pymap3d import ned2geodetic

from navpy.core.command.gimbal.gimbal_geo_zoom_selector import GeoZoomSelector
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.config.uas_args import UasArgs
from navpy.core.kernel.location import Location
from navpy.core.perception.camera.camera_intrinsics import CameraIntrinsics
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.perception.profile.vision_camera_calibration import (
    CameraZoomCalibration,
)
from navpy.core.ports.detect_data import DetectStatus
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.skills.acquire.peer_geo import PeerGeoAcquisition
from navpy.sim.camera.finite_poi_projector import FinitePoiProjector
from navpy.sim.camera.sim_camera_ports import FrameSize, ProjectionCameraPort
from navpy.sim.world.simulation_object import SimulationObject

WIDTH, HEIGHT = 1280, 720
FX = FY = 1000.0
CX, CY = 640.0, 360.0
RANGE_M = 100.0
UAV = Location(40.0, -74.0, 300.0)
ATTITUDE = Attitude(pitch=2.0, yaw=30.0, roll=5.0)
GIMBAL = GimbalData(att=Attitude(pitch=-10.0, yaw=0.0, roll=0.0), name="cam")

# Just inside / just outside each frame edge. The outside points used to round
# onto the edge (-0.25 -> 0, 1280.25 -> 1280) and pass as in frame.
OUTSIDE = [(-0.25, CY), (WIDTH + 0.25, CY), (CX, -0.25), (CX, HEIGHT + 0.25)]
INSIDE = [(0.25, CY), (WIDTH - 0.25, CY), (CX, 0.25), (CX, HEIGHT - 0.25)]


def _k64() -> np.ndarray:
    # float64 so the round trip tests rounding, not float32 matrix inversion.
    return np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(FX, FY, CX, CY, image_width=WIDTH, image_height=HEIGHT)


def _ned_for_pixel(u: float, v: float) -> np.ndarray:
    """NED offset from the UAV whose projection is exactly (u, v)."""
    ray = np.asarray(
        GeoRefCalc(UasArgs()).calc_ned(u, v, _k64(), GIMBAL, ATTITUDE),
        dtype=float,
    )
    return ray / np.linalg.norm(ray) * RANGE_M


def _poi_location(u: float, v: float) -> Location:
    north, east, down = _ned_for_pixel(u, v)
    lat, lng, alt = ned2geodetic(north, east, down, UAV.lat, UAV.lng, UAV.alt)
    return Location(lat, lng, alt)


def test_calc_uv_returns_the_fractional_pixel() -> None:
    geo_ref = GeoRefCalc(UasArgs())
    u, v = 700.37, 371.81

    projected = geo_ref.calc_uv(_ned_for_pixel(u, v), _k64(), GIMBAL, ATTITUDE)

    assert projected == pytest.approx((u, v), abs=1e-6)


@pytest.mark.parametrize("pixel", OUTSIDE)
def test_detect_rejects_a_point_just_outside_the_continuous_frame(pixel) -> None:
    assert _detect(*pixel).status is DetectStatus.OutOfView


@pytest.mark.parametrize("pixel", INSIDE)
def test_detect_accepts_a_point_just_inside_the_continuous_frame(pixel) -> None:
    result = _detect(*pixel)

    assert result.status is DetectStatus.DETECTED
    observed = (result.poi.pixel.u_px, result.poi.pixel.v_px)
    assert observed == pytest.approx(pixel, abs=1e-3)


def _detect(u: float, v: float):
    intrinsics = _intrinsics()
    projector = FinitePoiProjector(
        ProjectionCameraPort(
            intrinsics.get_k,
            lambda: GIMBAL,
            FrameSize(WIDTH, HEIGHT),
            intrinsics.is_valid,
        ),
        GeoRefCalc(UasArgs()).calc_uv,
        lambda: (),
        lambda: 12.5,
    )
    poi = SimulationObject(7, _poi_location(u, v), 2.0)
    return projector.detect(UAV, poi, ATTITUDE)


@pytest.mark.parametrize(
    ("pixel", "in_frame"),
    [(p, False) for p in OUTSIDE] + [(p, True) for p in INSIDE],
)
def test_peer_acquisition_uses_the_continuous_frame(pixel, in_frame) -> None:
    intrinsics = _intrinsics()
    mount = SimpleNamespace(
        get_k=intrinsics.get_k,
        get_gimbal_data=lambda: GIMBAL,
        is_valid=intrinsics.is_valid,
    )
    acquisition = PeerGeoAcquisition(
        SimpleNamespace(mounts=[mount]),
        GeoRefCalc(UasArgs()),
        {},
        SimpleNamespace(poi_location=None),
        SimpleNamespace(orbit_radius_m=0.0),
        lambda: None,
        Mock(),
    )

    state = acquisition.snapshot(UAV, ATTITUDE, _poi_location(*pixel), class_id=0)

    assert state is not None
    assert state.in_frame is in_frame
    assert (state.reason == "out_of_fov") is not in_frame


@pytest.mark.parametrize(
    ("pixel", "selected"),
    [(p, None) for p in OUTSIDE] + [(p, "1") for p in INSIDE],
)
def test_zoom_selection_uses_the_continuous_frame(pixel, selected) -> None:
    selector = GeoZoomSelector(
        SimpleNamespace(mount=SimpleNamespace(is_valid=_intrinsics().is_valid)),
        lambda _class_id: 2.0,
        lambda: (CameraZoomCalibration("1", FX, FY, CX, CY),),
    )

    zoom, _ = selector.select(
        _ned_for_pixel(*pixel),
        GIMBAL,
        ATTITUDE,
        GeoRefCalc(UasArgs()),
        0,
        1.0,
        RANGE_M,
    )

    assert zoom == selected
