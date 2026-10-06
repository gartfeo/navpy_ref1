"""The ZR10 profile must describe the frame the camera actually delivers.

The zoom-1 row of this profile was a genuine calibration recorded against the
wrong resolution. The images it came from are 1920x1080 and are still in
`tools/cam/calibration/siyi_zr10/zoom1/`; the profile declared 2560x1440. Since
the runtime rescales stored intrinsics to the delivered frame by width ratio,
that single wrong pair of fields divided every focal length by 2560 where it
should have divided by the real calibration width, and the camera ran about a
third short at the wide end.

Nothing failed while that was true. A closed tracking loop absorbs a
mis-scaled pixel error as loop gain and still converges, so only an open-loop
computed move exposes it -- which is why these tests assert the delivered
numbers rather than merely that the file parses. On the rig, one absolute
command per shot overshot by a median of 29.8 px under the old table; with
these intrinsics and a separate per-axis mount gain the same twelve shots left
a median of 3.1 px. The mount gain is a distinct correction and is not stored
here, so that figure is not attributable to the intrinsics alone.

The values come from a chessboard calibration on the delivered stream itself
(24 views, RMS 0.140 px), in which mount angles never enter the fit, so the
focal length comes from corner positions alone and no gimbal gain error can
reach it.

Consumers do not all rescale. Several read the stored numbers directly and
compare them against delivered-frame pixels, so for this profile the stored
frame and the delivered frame have to be the same one. That is the invariant
these tests hold, and it is a property of this profile rather than a general
rule -- a calibration stored at another size is fine wherever every consumer
scales it.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

import pytest

from navpy.modules.vision.camera_mount_optics import MountOpticsReader
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics

PROFILE_PATH = (
    Path(__file__).resolve().parents[3]
    / "src" / "navpy" / "modules" / "vision" / "vision_profiles.json"
)

# The only mode this camera serves over RTSP.
STREAM_W, STREAM_H = 1280, 720

# Measured on that stream, mount never consulted.
MEASURED_FX, MEASURED_FY = 1407.42, 1401.91
MEASURED_CX, MEASURED_CY = 670.95, 369.78
MEASURED_DIST = (-0.266926, 0.247147, -0.000653, 0.000251, -0.092969)

# What zooms 2-10 delivered before the resolution was corrected, so that
# restating the table at a new size can be shown to leave them alone. They are
# fabricated -- each is the zoom-1 value times the row index, with no
# distortion model -- and correcting the label must not disturb them, because a
# uniform correction repairs the wide end while pushing zoom 10 from 12% low to
# 17% high. Repairing that curve needs its own measurements.
HISTORICAL_FOCAL = {
    "2": (2066.485, 2082.845),
    "3": (3099.730, 3124.265),
    "4": (4132.970, 4165.685),
    "5": (5166.215, 5207.110),
    "6": (6199.455, 6248.530),
    "7": (7232.700, 7289.950),
    "8": (8265.940, 8331.375),
    "9": (9299.185, 9372.795),
    "10": (10332.425, 10414.215),
}


@pytest.fixture(scope="module")
def zr10_camera_config():
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    device = data["profiles"]["siyi_zr10"]["devices"][0]
    assert device["name"] == "siyi_zr10"
    return device["camera"]


def _camera(config) -> CameraIntrinsics:
    return CameraIntrinsics(
        zoom_map=dict(config["intrinsics"]["zooms"]),
        image_width=config["image_width"],
        image_height=config["image_height"],
    )


def _delivered_k(camera: CameraIntrinsics):
    """The matrix the runtime hands to consumers for a delivered frame."""
    reader = MountOpticsReader(camera, threading.RLock())
    return reader.get_k_for_frame(STREAM_W, STREAM_H)


def test_declared_resolution_is_the_delivered_stream(zr10_camera_config):
    """The root cause, and the invariant the unscaled consumers depend on."""
    assert (
        zr10_camera_config["image_width"],
        zr10_camera_config["image_height"],
    ) == (STREAM_W, STREAM_H)


def test_zoom_one_matches_the_measured_calibration(zr10_camera_config):
    camera = _camera(zr10_camera_config)
    assert camera.set_zoom("1")
    matrix = _delivered_k(camera)
    assert matrix[0, 0] == pytest.approx(MEASURED_FX, abs=1.0)
    assert matrix[1, 1] == pytest.approx(MEASURED_FY, abs=1.0)
    assert matrix[0, 2] == pytest.approx(MEASURED_CX, abs=1.0)
    assert matrix[1, 2] == pytest.approx(MEASURED_CY, abs=1.0)


def test_zoom_one_focal_is_far_from_the_mislabelled_value(zr10_camera_config):
    """Name the specific regression, not only the correct answer.

    2066.49 scaled by 1280/2560 gives 1033.2, which is what shipped. Any future
    edit that reintroduces a 2560-wide denominator lands back near it, and a
    failure here says which mistake was made.
    """
    camera = _camera(zr10_camera_config)
    assert camera.set_zoom("1")
    delivered_fx = float(_delivered_k(camera)[0, 0])
    assert delivered_fx / 1033.245 > 1.25


def test_zoom_one_carries_the_measured_distortion(zr10_camera_config):
    """Pin the coefficients: the old vector also parses and is also non-zero."""
    camera = _camera(zr10_camera_config)
    assert camera.set_zoom("1")
    dist = [float(value) for value in camera.get_dist()]
    assert dist[:5] == pytest.approx(MEASURED_DIST, abs=1e-5)


@pytest.mark.parametrize("zoom_key", sorted(HISTORICAL_FOCAL, key=int))
def test_higher_zooms_deliver_the_focal_lengths_they_always_did(
    zr10_camera_config, zoom_key,
):
    """Restating the table at a new size must not move the focal curve.

    This covers delivered fx and fy only. Consumers that read stored values
    without rescaling do see different numbers, deliberately -- that is the
    defect being fixed, not a side effect being hidden.
    """
    camera = _camera(zr10_camera_config)
    assert camera.set_zoom(zoom_key)
    matrix = _delivered_k(camera)
    expected_fx, expected_fy = HISTORICAL_FOCAL[zoom_key]
    assert matrix[0, 0] == pytest.approx(expected_fx, rel=1e-9)
    assert matrix[1, 1] == pytest.approx(expected_fy, rel=1e-9)


def test_every_row_carries_the_measured_principal_point(zr10_camera_config):
    """One principal point across the table, and it is the measured one.

    It is only measured at zoom 1; the upper rows carry it as an assumption,
    on the grounds that the value they held before has no validation for this
    delivered mode and a split would put a step in the aim point mid-zoom.
    """
    camera = _camera(zr10_camera_config)
    for key in zr10_camera_config["intrinsics"]["zooms"]:
        assert camera.set_zoom(key)
        matrix = _delivered_k(camera)
        assert matrix[0, 2] == pytest.approx(MEASURED_CX, abs=1.0), key
        assert matrix[1, 2] == pytest.approx(MEASURED_CY, abs=1.0), key


def test_every_row_shares_one_declared_resolution(zr10_camera_config):
    """Mixed per-row resolutions would corrupt interpolation between rows.

    `lerp_entry` interpolates raw focal values between neighbouring rows and
    then takes the image size from the lower one, so a 1280-based row beside a
    2560-based row reports about 2770 at zoom 1.5 where this table gives about
    1737. Rows may omit the fields and inherit, but must not disagree.
    """
    camera_width = zr10_camera_config["image_width"]
    camera_height = zr10_camera_config["image_height"]
    for key, entry in zr10_camera_config["intrinsics"]["zooms"].items():
        assert entry.get("image_width", camera_width) == camera_width, key
        assert entry.get("image_height", camera_height) == camera_height, key


def test_interpolated_zoom_lands_between_its_neighbours(zr10_camera_config):
    """The symptom a mixed-width table would produce, asserted directly."""
    camera = _camera(zr10_camera_config)
    assert camera.set_zoom(1.5)
    delivered_fy = float(_delivered_k(camera)[1, 1])
    low, high = MEASURED_FY, HISTORICAL_FOCAL["2"][1]
    assert low < delivered_fy < high
    assert delivered_fy == pytest.approx(0.5 * (low + high), rel=1e-6)


def test_principal_point_is_inside_the_frame(zr10_camera_config):
    camera = _camera(zr10_camera_config)
    for key in zr10_camera_config["intrinsics"]["zooms"]:
        assert camera.set_zoom(key)
        matrix = _delivered_k(camera)
        assert 0 < float(matrix[0, 2]) < STREAM_W, key
        assert 0 < float(matrix[1, 2]) < STREAM_H, key
