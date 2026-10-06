from __future__ import annotations

from dataclasses import replace

import inspect
from pathlib import Path

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    TerminalFrameProjector,
    TerminalProjectionConfig,
)
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from navpy.modules.vision.models.detection_components import (
    ConfirmationEvidence,
    DetectionClassification,
    DetectionGeoDiagnostics,
    DetectionIdentity,
    OpticsProvenance,
    PoseProvenance,
    SourceTiming,
    TrackingEvidence,
)
from navpy.modules.vision.models.pixel_observation import (
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.ideal_target_projector import (
    IdealTargetProjector,
    UasFrameConvention,
    ideal_angular_camera_matrix,
)
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.visual_ray_projection import (
    body_ray_to_pixel,
    camera_to_body_from_gimbal,
    unit_vector,
)


def _gimbal() -> GimbalData:
    return GimbalData(att=Attitude(0.0, 0.0, 0.0), name="ideal")


def _target(
    *,
    pixel: tuple[float, float],
    matrix: np.ndarray,
    gimbal: GimbalData,
    direct_los: np.ndarray | None = None,
) -> DetectedObject:
    calibration = PixelCalibration.from_matrix(matrix)
    return DetectedObject(
        identity=DetectionIdentity(2, 1),
        classification=DetectionClassification(DetectionSizeClass.L),
        pixel=PixelObservation(
            pixel[0],
            pixel[1],
            calibration,
            camera_to_body_from_gimbal(gimbal),
            PixelProjectionKind.SPHERICAL_EQUIANGULAR,
            0.0,
            0.0,
            4.0,
            True,
            "ideal",
            # Stated, though these tests are about PIXEL geometry and do not
            # read it. The projector refuses a frame whose yaw rate is absent
            # (`frame_projection.py`), so leaving it at its None default made
            # every projection here return None and the round-trip assertions
            # fail for a reason that had nothing to do with pixels.
            aircraft_yaw_rate_rad_s=0.0,
        ),
        tracking=TrackingEvidence(),
        confirmation=ConfirmationEvidence(),
        pose=PoseProvenance(Attitude(0.0, 179.0, 0.0), gimbal),
        optics=OpticsProvenance(calibration),
        timing=SourceTiming(detection_timestamp_s=4.0),
        geo=DetectionGeoDiagnostics(truth_target_location=direct_los),
    )


def _project(target: DetectedObject) -> np.ndarray:
    frame = TerminalFrameProjector(
        TerminalProjectionConfig("ZYX", True),
    ).project(target.visual_detection(), 0)
    assert frame is not None
    return np.asarray(frame.body_ray)


def test_pixels_are_authoritative_when_truth_los_disagrees() -> None:
    matrix = ideal_angular_camera_matrix(FrameSize(2560, 1440))
    gimbal = _gimbal()
    truth = np.asarray([1.0, 0.0, 0.0])
    first = _target(pixel=(1280.0, 720.0), matrix=matrix, gimbal=gimbal, direct_los=truth)
    second = _target(pixel=(-500_000.0, 250_000.0), matrix=matrix, gimbal=gimbal, direct_los=truth)

    assert _project(first) != pytest.approx(_project(second))
    assert not hasattr(first, "vision_los_uas")


@pytest.mark.parametrize(
    "body_ray",
    [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
        (0.31, -0.47, -0.83),
    ],
)
def test_spherical_pixels_round_trip_front_rear_axes_and_oblique(body_ray) -> None:
    expected = np.asarray(body_ray, dtype=float)
    expected /= np.linalg.norm(expected)
    matrix = ideal_angular_camera_matrix(FrameSize(2560, 1440))
    gimbal = _gimbal()
    pixel = body_ray_to_pixel(
        expected,
        PixelCalibration.from_matrix(matrix),
        camera_to_body_from_gimbal(gimbal),
        PixelProjectionKind.SPHERICAL_EQUIANGULAR,
    )

    assert _project(_target(pixel=pixel, matrix=matrix, gimbal=gimbal)) == pytest.approx(
        expected,
        abs=1e-9,
    )


def test_ideal_camera_read_is_a_defensive_snapshot() -> None:
    source = _gimbal()
    state = IdealCameraState(lambda: source)
    state.prepare()

    leaked = state.read()
    leaked.att.pitch = 61.0
    leaked.setup_dist[0] = 99.0

    reread = state.read()
    assert reread.att.pitch == 0.0
    assert reread.setup_dist == [0.0, 0.0, 0.0]


def test_detected_target_constructor_is_grouped_not_a_flat_property_bag() -> None:
    parameters = [
        parameter
        for name, parameter in inspect.signature(DetectedObject).parameters.items()
        if name != "self"
    ]
    assert len(parameters) <= 12
    assert not any(
        parameter.kind in (parameter.VAR_POSITIONAL, parameter.VAR_KEYWORD)
        for parameter in parameters
    )
    assert "__getattr__" not in DetectedObject.__dict__


def test_point_mass_uses_production_pixel_projector_boundary() -> None:
    sensor = Path("scripts/vision_static_point_mass_sensor.py").read_text(
        encoding="utf-8"
    )
    runner = Path("scripts/vision_static_point_mass_run.py").read_text(
        encoding="utf-8"
    )
    assert "TerminalFrameProjector" not in sensor
    assert "TerminalFrameProjector" in runner
    assert "TerminalVisionFrame(" not in sensor + runner
    assert "body_ray_to_pixel" in sensor
    assert "vision_los_uas" not in sensor + runner


def test_camera_transform_helper_is_not_the_terminal_truth_channel() -> None:
    ray = np.asarray([0.2, -0.3, 0.9], dtype=float)
    transform = camera_to_body_from_gimbal(_gimbal())
    camera = unit_vector(transform.matrix().T @ unit_vector(ray))
    assert np.linalg.norm(camera) == pytest.approx(1.0)


def test_ideal_projector_does_not_hide_programming_contract_errors() -> None:
    class BrokenCamera:
        @staticmethod
        def read():
            raise AttributeError("camera contract drift")

    projector = IdealTargetProjector(
        BrokenCamera(),
        FrameSize(2560, 1440),
        UasFrameConvention("ZYX", True),
        lambda: 1.0,
    )
    target = SimulationObject(1, Location(40.01, 44.01, 0.0), 0)

    with pytest.raises(AttributeError, match="camera contract drift"):
        projector.project(
            Location(40.0, 44.0, 1000.0),
            target,
            Attitude(0.0, 0.0, 0.0),
            navigation_attitude=Attitude(0.0, 0.0, 0.0),
        )


def test_a_missing_yaw_rate_refuses_the_frame_deliberately() -> None:
    """Refusal, but attributable — it used to happen by accident.

    `aircraft_yaw_rate_rad_s` is OPTIONAL on PixelObservation, and
    `aircraft_yaw_rate_rad_s()` returns None whenever the conversion has no
    answer: missing body rates, or cos(pitch) singular near vertical. The
    projector called `_finite` on it regardless; the TypeError was swallowed by
    a broad handler and the whole frame disappeared with no diagnostic,
    indistinguishable from a malformed observation.

    The refusal is kept — the lateral channel needs this term to remove the
    aircraft's own turn — but it is now an explicit branch. The consequence is
    worth stating plainly: near-vertical pitch produces no frame at all, so the
    aircraft goes blind exactly where it is diving.
    """
    matrix = ideal_angular_camera_matrix(FrameSize(2560, 1440))
    gimbal = _gimbal()
    target = _target(pixel=(1280.0, 720.0), matrix=matrix, gimbal=gimbal)
    projector = TerminalFrameProjector(TerminalProjectionConfig("ZYX", True))

    assert projector.project(target.visual_detection(), 0) is not None

    without = replace(
        target,
        pixel=replace(target.pixel, aircraft_yaw_rate_rad_s=None),
    )
    assert projector.project(without.visual_detection(), 0) is None
