from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pymap3d
import pytest

from navpy.args.uas_args import UasArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_sim import GimbalSiyiSim
from navpy.modules.vision.vision_camera_calibration import read_camera_zoom_calibration
from navpy.modules.vision.vision_profiles import (
    build_tracking_config,
    build_zoom_config,
    get_devices,
    get_min_pixels_for_class,
    resolve_profile,
)
from navpy.modules.vision.vision_class_profile import get_class_detect_size


def _rig(initial_zoom: str) -> SimpleNamespace:
    _, profile, _ = resolve_profile("siyi_zr10", Mock())
    device = get_devices(profile)[0]
    zooms = device["camera"]["intrinsics"]["zooms"]
    camera = CameraIntrinsics(
        zoom_map=zooms,
        image_width=device["camera"]["image_width"],
        image_height=device["camera"]["image_height"],
    )
    assert camera.set_zoom(initial_zoom)
    vehicle = SimpleNamespace(attitude=Attitude(0.0, 90.0, 0.0))
    gimbal = GimbalSiyiSim(
        GimbalData(
            att=Attitude(-14.0, 0.0, 0.0),
            setup=GimbalMountSetup(att=Attitude(90.0, 0.0, 90.0), seq="XYZ"),
            g_seq="XYZ",
        ),
        SimpleNamespace(read=lambda: vehicle.attitude),
        Mock(),
    )
    mount = CameraMount("siyi_zr10", camera, gimbal)
    tracking = build_tracking_config(device, sim=True)
    assert isinstance(tracking, GimbalTrackingSetup)
    navigation = GimbalNavigation(
        mount,
        Mock(),
        tracking=tracking,
        zoom_config=build_zoom_config(device, profile),
        neutral_pitch_deg=-14.0,
    )
    return SimpleNamespace(
        vehicle=vehicle,
        profile=profile,
        mount=mount,
        gimbal=gimbal,
        navigation=navigation,
        geo_ref=GeoRefCalc(UasArgs()),
        initial_zoom=initial_zoom,
    )


def _sample(rig: SimpleNamespace, uav: Location, poi: Location) -> tuple:
    ned = pymap3d.geodetic2ned(
        poi.lat,
        poi.lng,
        poi.alt,
        uav.lat,
        uav.lng,
        uav.alt,
    )
    k = rig.mount.get_k()
    uv = rig.geo_ref.calc_uv(
        ned,
        k,
        rig.mount.get_gimbal_data(),
        rig.vehicle.attitude,
    )
    center_error_px = float(
        np.hypot(float(uv[0]) - k[0, 2], float(uv[1]) - k[1, 2])
    )
    return uv, center_error_px


def _wait_for_zoom(rig: SimpleNamespace, expected: float) -> None:
    deadline_s = time.monotonic() + 3.0
    while time.monotonic() < deadline_s:
        sample = rig.gimbal.get_zoom_level_sample()
        if sample is not None and abs(sample[0] - expected) < 0.05:
            return
        time.sleep(0.02)
    raise AssertionError(
        f"zoom did not converge to {expected}: {rig.gimbal.get_zoom_level()}"
    )


def _drive_until_centered(
    rig: SimpleNamespace,
    uav: Location,
    poi: Location,
) -> None:
    deadline_s = time.monotonic() + 5.0
    while time.monotonic() < deadline_s:
        rig.navigation.update_geo(uav, rig.vehicle.attitude)
        rig.navigation.prepare_geo_acquisition(
            uav,
            rig.vehicle.attitude,
            class_id=0,
            min_pixels=get_min_pixels_for_class(rig.profile, 0),
        )
        try:
            _uv, error_px = _sample(rig, uav, poi)
        except TypeError:
            error_px = float("inf")
        if error_px < 5.0 and rig.navigation.status.geo.zoom_key == "5":
            return
        time.sleep(0.02)
    raise AssertionError("SIYI geo tracking did not center the POI")


def test_geo_tracking_recenters_and_normalizes_zoom_from_different_histories() -> None:
    poi = Location(40.0, 44.0, 100.0, is_absolute=True)
    uav = Location(*pymap3d.ned2geodetic(500.0, 0.0, -100.0, 40.0, 44.0, 100.0),
                   is_absolute=True)
    rigs = [_rig("2"), _rig("8")]
    try:
        for rig in rigs:
            rig.mount.start()
            assert rig.mount.set_zoom(rig.initial_zoom)
            time.sleep(1.5)
            rig.navigation.start_geo_tracking(poi, rig.geo_ref)
            _drive_until_centered(rig, uav, poi)
            _wait_for_zoom(rig, 5.0)

        evidence = []
        for rig in rigs:
            uv, error_px = _sample(rig, uav, poi)
            slant_m = float(np.linalg.norm(pymap3d.geodetic2ned(
                poi.lat, poi.lng, poi.alt, uav.lat, uav.lng, uav.alt,
            )))
            projected_px = (
                float(rig.mount.get_k()[1, 1])
                * get_class_detect_size(0)
                / slant_m
            )
            evidence.append((rig.mount.get_current_zoom(), uv, error_px, projected_px))

        assert float(evidence[0][0]) == pytest.approx(float(evidence[1][0]))
        for zoom, uv, error_px, projected_px in evidence:
            assert float(zoom) == pytest.approx(5.0)
            assert rigs[0].mount.is_valid(float(uv[0]), float(uv[1]))
            assert error_px < 5.0
            assert projected_px >= get_min_pixels_for_class(rigs[0].profile, 0)
    finally:
        for rig in rigs:
            rig.navigation.stop_geo_tracking()
            rig.mount.stop()


def test_geo_selector_has_one_smallest_candidate_for_fixed_geometry() -> None:
    rig = _rig("1")
    calibrations = read_camera_zoom_calibration(rig.mount.camera)
    projected = [
        (entry.zoom, entry.fy * get_class_detect_size(0) / 500.0)
        for entry in calibrations
    ]

    eligible = [
        zoom for zoom, pixels in projected
        if pixels >= get_min_pixels_for_class(rig.profile, 0)
    ]

    # Zoom 5, not 3: the selector reads stored focal lengths without
    # rescaling, so it must be given a table stored against the delivered
    # frame. At this 500 m geometry zoom 3 yields 26.9 px on the delivered
    # image against a 36 px class minimum, and zoom 5 yields 44.8 px.
    assert eligible[0] == "5"
