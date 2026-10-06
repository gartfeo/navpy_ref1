"""Focused simulator detector, projection, and compatibility tests."""

from __future__ import annotations

import inspect
import threading
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
from pymap3d import ned2geodetic

from navpy.args.uas_args import UasArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.models.detect_data import (
    DetectedObject,
    DetectStatus,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.detector_sim import DetectorSim, SimFrameGenerator
from navpy.modules.vision.sim.finite_poi_projector import FinitePoiProjector
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.ideal_poi_projector import (
    IdealPoiProjector,
    UasFrameConvention,
    ideal_angular_camera_matrix,
)
from navpy.modules.vision.sim.sim_camera_ports import (
    CaptureCameraPort,
    FrameSize,
    ProjectionCameraPort,
)
from navpy.modules.vision.sim.sim_confirmation_capture import ConfirmationCapture
from navpy.modules.vision.sim.sim_detector_controls import (
    SimDetectorIdentity,
    SimGeoControls,
    SimTrackingControls,
    SimZoomControls,
)
from navpy.modules.vision.sim.sim_detector_state import SimCaptureState
from navpy.modules.vision.sim.sim_poi_projector import SimPoiProjector
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.visual_ray_projection import observation_body_ray
from tests.detection_factory import make_detected_poi


def _mount(*, attitude: Attitude = Attitude(0.0, 0.0, 0.0)):
    mount = Mock()
    mount.name = "ideal"
    mount.image_width = 2560
    mount.image_height = 1440
    mount.get_gimbal_data.return_value = GimbalData(
        att=attitude,
        max_detect_distance=1.0,
        name="ideal",
    )
    return mount


@dataclass(frozen=True)
class ProjectorRig:
    projector: SimPoiProjector
    ideal_camera: IdealCameraState


def _projector(mount, *, ideal: bool, pois=(), calc_uv=None) -> ProjectorRig:
    geo_ref = GeoRefCalc(UasArgs())
    frame_size = FrameSize(mount.image_width, mount.image_height)
    camera = ProjectionCameraPort(
        mount.get_k,
        mount.get_gimbal_data,
        frame_size,
        mount.is_valid,
    )
    finite = FinitePoiProjector(
        camera,
        calc_uv or geo_ref.calc_uv,
        lambda: tuple(pois),
        lambda: 12.5,
    )
    ideal_camera = IdealCameraState(mount.get_gimbal_data)
    ideal_projector = IdealPoiProjector(
        ideal_camera,
        frame_size,
        UasFrameConvention(geo_ref.uas_seq, geo_ref.degrees),
        lambda: 12.5,
    )
    if ideal:
        ideal_camera.prepare()
    return ProjectorRig(
        SimPoiProjector(ideal, finite, ideal_projector),
        ideal_camera,
    )


def _poi_from_ned(
    current: Location,
    north_m: float,
    east_m: float,
    down_m: float,
    *,
    uid: int = 7,
) -> SimulationObject:
    lat, lng, alt = ned2geodetic(
        north_m,
        east_m,
        down_m,
        current.lat,
        current.lng,
        current.alt,
    )
    return SimulationObject(uid, Location(lat, lng, alt), 2.0)


def test_finite_projection_rebases_paired_gimbal_readback_to_frame_pose() -> None:
    mount = _mount()
    mount.get_k.return_value = np.array(
        [[1000.0, 0.0, 500.0], [0.0, 1000.0, 250.0], [0.0, 0.0, 1.0]]
    )
    paired = Attitude(1.0, 20.0, 2.0)
    mount.get_gimbal_data.return_value = GimbalData(
        att=Attitude(0.0, -20.0, 0.0),
        reference_aircraft_attitude=paired,
        max_detect_distance=5000.0,
    )
    mount.is_valid.return_value = True
    calc_uv = Mock(return_value=(500.0, 250.0))
    current = Location(40.0, 44.0, 100.0, is_absolute=True)
    poi = _poi_from_ned(current, 1000.0, 0.0, 0.0)
    projector = _projector(
        mount,
        ideal=False,
        pois=[poi],
        calc_uv=calc_uv,
    ).projector

    detection = projector.project_poi(
        current,
        poi,
        Attitude(3.0, 35.0, 4.0),
    )

    assert detection is not None
    assert calc_uv.call_args.args[3] == Attitude(3.0, 35.0, 4.0)
    assert detection.pose.aircraft_attitude == Attitude(3.0, 35.0, 4.0)


def test_stabilized_readback_is_rebased_to_the_current_camera_pose() -> None:
    """Aircraft motion after a gimbal tick must not invent POI motion."""
    mount = _mount()
    mount.get_k.return_value = np.array(
        [[1000.0, 0.0, 500.0], [0.0, 1000.0, 250.0], [0.0, 0.0, 1.0]]
    )
    mount.get_gimbal_data.return_value = GimbalData(
        att=Attitude(0.0, 20.0, 0.0),
        reference_aircraft_attitude=Attitude(0.0, 10.0, 0.0),
        max_detect_distance=5000.0,
    )
    mount.is_valid.return_value = True
    current = Location(40.0, 44.0, 100.0, is_absolute=True)
    bearing_rad = np.deg2rad(30.0)
    poi = _poi_from_ned(
        current,
        1000.0 * np.cos(bearing_rad),
        1000.0 * np.sin(bearing_rad),
        0.0,
    )
    finite = _projector(mount, ideal=False, pois=[poi]).projector

    detection = finite.project_poi(
        current,
        poi,
        Attitude(0.0, 20.0, 0.0),
    )

    assert detection is not None
    body_ray = observation_body_ray(detection.pixel)
    assert np.rad2deg(np.arctan2(body_ray[1], body_ray[0])) == pytest.approx(
        10.0,
        abs=0.1,
    )
    assert detection.pose.aircraft_attitude.yaw == pytest.approx(20.0)


class TestSimFrameGenerator:
    def test_unavailable_assets_are_safe(self):
        generator = SimFrameGenerator("/nonexistent/dir", (1920, 1080))
        assert generator.is_available is False
        assert generator.generate_frame([(0.5, 0.5, 0.5)]) is None

    @patch("cv2.imread")
    @patch("os.path.exists", return_value=True)
    def test_assets_make_generator_available(self, _exists, imread):
        background = np.zeros((1080, 1920, 3), dtype=np.uint8)
        dock = np.zeros((100, 100, 4), dtype=np.uint8)
        imread.side_effect = [background, dock] + [None] * 7
        assert SimFrameGenerator("assets", (1920, 1080)).is_available is True

    def test_frame_generation_returns_bbox(self):
        generator = SimFrameGenerator.__new__(SimFrameGenerator)
        generator._frame_size = (1920, 1080)
        generator._background = np.zeros((1080, 1920, 3), dtype=np.uint8)
        generator._sprites = {
            "dock": np.ones((100, 100, 4), dtype=np.uint8) * 128,
        }
        frame, boxes = generator.generate_frame([(0.5, 0.5, 0.3)])
        assert frame.shape == (1080, 1920, 3)
        assert len(boxes) == 1 and len(boxes[0]) == 4

    def test_fallback_location_sprite_and_unknown_fallback(self):
        generator = SimFrameGenerator.__new__(SimFrameGenerator)
        generator._frame_size = (1920, 1080)
        generator._background = np.zeros((1080, 1920, 3), dtype=np.uint8)
        generator._sprites = {
            "dock": np.zeros((100, 100, 4), dtype=np.uint8),
            "building": np.zeros((80, 120, 4), dtype=np.uint8),
        }
        _, building = generator.generate_frame(
            [(0.5, 0.5, 1.0)],
            location_type="building",
        )
        _, fallback = generator.generate_frame(
            [(0.5, 0.5, 1.0)],
            location_type="unknown",
        )
        assert building[0][2:] == (120, 80)
        assert fallback[0][2:] == (100, 100)
        assert generator.sprite_width_for("building") == 120
        assert generator.sprite_height_for("building") == 80


class TestConfirmationCapture:
    def test_centered_best_frame_stays_separate_from_live_tracking_bbox(self):
        mount = SimpleNamespace(image_width=1000, image_height=1000)
        generator = Mock(is_available=True)
        centered = np.full((10, 10, 3), 10, dtype=np.uint8)
        off_center = np.full((10, 10, 3), 200, dtype=np.uint8)
        generator.generate_frame.side_effect = [
            (centered, [(500.0, 500.0, 120.0, 80.0)]),
            (off_center, [(900.0, 900.0, 180.0, 140.0)]),
        ]
        state = SimCaptureState(renderer=generator)
        capture = ConfirmationCapture(
            CaptureCameraPort(lambda: np.eye(3), FrameSize(1000, 1000)),
            state,
            Mock(),
        )
        detection = make_detected_poi(
            obj_id=7,
            tracking_bbox_cxcywh=(500.0, 500.0, 40.0, 40.0),
        )
        capture.capture([detection], [(0.5, 0.5, 1.0)])
        capture.capture([detection], [(0.9, 0.9, 1.0)])
        assert np.array_equal(detection.confirmation.frame, centered)
        assert detection.confirmation.bbox_cxcywh == (500.0, 500.0, 120.0, 80.0)
        assert detection.tracking.bbox_cxcywh == (500.0, 500.0, 40.0, 40.0)

    def test_capture_state_reset_clears_frames_and_reenables_capture(self):
        state = SimCaptureState(enabled=False, best_frames={7: object()})
        state.reset()
        assert state.enabled is True
        assert state.best_frames == {}


class TestIdealPoiProjection:
    def test_far_poi_behind_aircraft_is_unbounded_and_detected(self):
        mount = _mount(attitude=Attitude(-20.0, 90.0, 5.0))
        mount.get_k.side_effect = AssertionError("physical intrinsics are unused")
        mount.is_valid.side_effect = AssertionError("ideal sensor has no FOV gate")
        current = Location(40.0, 44.0, 1000.0)
        poi = _poi_from_ned(current, -50_001.0, 100.0, 20.0)
        projector = _projector(mount, ideal=True, pois=[poi]).projector

        result = projector.detect(
            current,
            poi,
            Attitude(0.0, 0.0, 0.0),
            timestamp_s=12.5,
            navigation_attitude=Attitude(-3.0, 179.0, 4.0),
        )

        assert result.status is DetectStatus.DETECTED
        detection = result.poi
        assert detection.pixel.u_px < 0.0 or detection.pixel.u_px > mount.image_width
        assert detection.geo.reference_height_m is None
        assert detection.tracking.bbox_cxcywh is None
        assert detection.pose.aircraft_attitude.yaw == 0.0
        assert detection.pose.aircraft_attitude.pitch == -3.0
        assert detection.timing.detection_timestamp_s == 12.5
        assert detection.confirmation.supports_frame is False
        assert not hasattr(detection, "vision_los_uas")

    def test_camera_is_static_across_gimbal_readback_changes(self):
        mount = _mount(attitude=Attitude(-6.0, 18.0, 3.0))
        current = Location(40.0, 44.0, 1000.0)
        poi = _poi_from_ned(current, 1000.0, 300.0, 100.0)
        projector = _projector(mount, ideal=True, pois=[poi]).projector
        first = projector.project_ideal_poi(
            current,
            poi,
            Attitude(0.0, 0.0, 0.0),
            timestamp_s=1.0,
            navigation_attitude=Attitude(-2.0, 120.0, 3.0),
        )
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(70.0, -120.0, 45.0),
            name="ideal",
        )
        second = projector.project_ideal_poi(
            current,
            poi,
            Attitude(0.0, 0.0, 0.0),
            timestamp_s=2.0,
            navigation_attitude=Attitude(-2.0, -90.0, 3.0),
        )
        assert (second.pixel.u_px, second.pixel.v_px) == pytest.approx(
            (first.pixel.u_px, first.pixel.v_px),
        )
        assert mount.get_gimbal_data.call_count == 1

    def test_missing_navigation_attitude_fails_closed(self):
        mount = _mount()
        current = Location(40.0, 44.0, 1000.0)
        poi = _poi_from_ned(current, 500.0, 100.0, 20.0)
        projector = _projector(mount, ideal=True, pois=[poi]).projector

        detection = projector.project_ideal_poi(
            current,
            poi,
            Attitude(17.0, 123.0, -9.0),
            navigation_attitude=None,
        )

        assert detection is None

    def test_reset_reacquires_static_camera_without_virtual_pointing(self):
        mount = _mount()
        current = Location(40.0, 44.0, 1000.0)
        poi = _poi_from_ned(current, 500.0, 0.0, 0.0)
        rig = _projector(mount, ideal=True, pois=[poi])
        rig.projector.project_ideal_poi(
            current,
            poi,
            Attitude(0, 0, 0),
            navigation_attitude=Attitude(0, 0, 0),
        )
        rig.ideal_camera.reset()
        rig.ideal_camera.prepare()
        rig.projector.project_ideal_poi(
            current,
            poi,
            Attitude(0, 0, 0),
            navigation_attitude=Attitude(0, 0, 0),
        )
        assert mount.get_gimbal_data.call_count == 2

    def test_angular_matrix_has_180_degree_nominal_span(self):
        matrix = ideal_angular_camera_matrix(FrameSize(2560, 1440))
        assert matrix[0, 0] * np.pi == pytest.approx(2560.0)
        assert matrix[1, 1] * np.pi == pytest.approx(1440.0)


class TestFinitePoiProjection:
    def _finite_projector(self, focal_y: float, max_distance: float = 5000.0):
        mount = _mount()
        mount.image_width = 1000
        mount.image_height = 800
        mount.get_k.return_value = np.array([
            [focal_y, 0.0, 500.0],
            [0.0, focal_y, 400.0],
            [0.0, 0.0, 1.0],
        ])
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            max_detect_distance=max_distance,
        )
        projector = _projector(
            mount,
            ideal=False,
            calc_uv=Mock(return_value=(500.0, 400.0)),
        ).projector
        return mount, projector

    def test_visible_poi_gets_live_tracking_bbox(self):
        mount, projector = self._finite_projector(2000.0)
        poi = SimulationObject(7, Location(40.0, -74.0, 0.0), 2.0)
        detection = projector.project_poi(
            Location(40.0, -74.0, 100.0),
            poi,
            Attitude(0.0, 0.0, 0.0),
            timestamp_s=42.25,
        )
        assert detection.tracking.bbox_cxcywh is not None
        assert detection.timing.detection_timestamp_s == 42.25
        assert detection.pose.status == "sim_frame_pose"

    def test_subthreshold_legacy_range_has_no_tracking_bbox(self):
        mount, projector = self._finite_projector(50.0)
        poi = SimulationObject(7, Location(40.0, -74.0, 0.0), 2.0)
        detection = projector.project_poi(
            Location(40.0, -74.0, 300.0),
            poi,
            Attitude(0.0, 0.0, 0.0),
        )
        assert detection is not None
        assert detection.tracking.bbox_cxcywh is None

    def test_geometry_projection_does_not_apply_fov_gate_but_detect_does(self):
        mount, projector = self._finite_projector(2000.0)
        mount.is_valid.return_value = False
        poi = SimulationObject(7, Location(40.0, -74.0, 0.0), 2.0)
        location = Location(40.0, -74.0, 100.0)
        assert projector.project_poi(
            location, poi, Attitude(0, 0, 0),
        ) is not None
        assert projector.detect(
            location, poi, Attitude(0, 0, 0),
        ).status is DetectStatus.OutOfView


class TestSimulatorControls:
    def test_tracking_zoom_and_geo_are_separate_owners(self):
        navigation = Mock()
        navigation.is_zoom_stable = False
        navigation.zoom_result = object()
        navigation.freeze_final_approach_zoom_at_min.return_value = True
        navigation.tracking_obj_id = 7
        navigation.is_detection_armed = True
        navigation.is_geo_armed = True
        navigation.loss_hold_sec = 0.25
        capture = SimCaptureState(enabled=False)
        tracking = SimTrackingControls(navigation, capture)
        zoom = SimZoomControls(navigation, tracking, capture)
        geo = SimGeoControls(navigation)

        tracking.start_tracking(7)
        zoom.freeze_final_approach_zoom_at_min()
        geo.prepare_geo_acquisition(Location(0, 0, 0), Attitude(0, 0, 0), 0, 10)

        assert capture.enabled is False
        assert tracking.is_detection_armed is True
        assert tracking.loss_hold_sec == 0.25
        assert zoom.is_zoom_stable is False
        assert zoom.get_zoom_result(7) is navigation.zoom_result
        assert geo.is_geo_armed is True
        navigation.start_tracking.assert_called_once_with(7)
        navigation.freeze_final_approach_zoom_at_min.assert_called_once_with()

    def test_tracking_start_restores_capture_state_when_navigation_raises(self):
        navigation = Mock()
        navigation.start_tracking.side_effect = RuntimeError("arm failed")
        capture = SimCaptureState(enabled=False)
        tracking = SimTrackingControls(navigation, capture)

        with pytest.raises(RuntimeError, match="arm failed"):
            tracking.start_tracking(7)

        assert capture.enabled is False

    def test_missing_navigation_is_explicit_null_behavior(self):
        capture = SimCaptureState()
        tracking = SimTrackingControls(None, capture)
        zoom = SimZoomControls(None, tracking, capture)
        geo = SimGeoControls(None)
        tracking.start_tracking(1)
        tracking.stop_tracking()
        assert tracking.is_detection_armed is False
        assert tracking.loss_hold_sec is None
        assert zoom.is_zoom_stable is True
        assert zoom.get_zoom_result() is None
        assert geo.prepare_geo_acquisition(
            Location(0, 0, 0), Attitude(0, 0, 0), 0, 1.0,
        ) is False


class TestDetectorSimAdapter:
    def test_adapter_is_not_a_thread_and_owns_exactly_one_field(self):
        ports = Mock()
        with patch(
            "navpy.modules.vision.sim.detector_sim.build_sim_detector",
            return_value=ports,
        ):
            detector = DetectorSim(Mock(), Mock(), Mock(), Mock(), Mock())
        assert not issubclass(DetectorSim, threading.Thread)
        assert vars(detector) == {"_parts": ports}
        assert "__getattr__" not in DetectorSim.__dict__

    def test_public_adapter_has_no_variadic_constructor_dependencies(self):
        signature = inspect.signature(DetectorSim.__init__)
        kinds = {parameter.kind for parameter in signature.parameters.values()}
        assert inspect.Parameter.VAR_POSITIONAL not in kinds
        assert inspect.Parameter.VAR_KEYWORD not in kinds

    def test_source_identity_is_lazy_and_uses_gimbal_data_name(self):
        mount = Mock()
        mount.name = "c720hd"
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            name="gimbal_0",
        )
        identity = SimDetectorIdentity(mount)

        mount.get_gimbal_data.assert_not_called()
        assert identity.source_name == "gimbal_0"
        mount.get_gimbal_data.assert_called_once_with()

    def test_source_identity_rejects_empty_gimbal_name(self):
        mount = Mock()
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            name="",
        )

        with pytest.raises(RuntimeError, match="non-empty name"):
            SimDetectorIdentity(mount).source_name


class TestFallbackLocationTypePropagation:
    def test_poi_and_detection_preserve_location_type(self):
        poi = SimulationObject(0, Location(0, 0, 0), 2, location_type="building")
        detection = make_detected_poi(
            obj_id=0,
            x_error=100,
            y_error=100,
            reference_height_m=2,
            k=np.eye(3),
            g_data=GimbalData(att=Attitude(0, 0, 0)),
            uas_att=Attitude(0, 0, 0),
            location_type="building",
        )
        assert poi.location_type == "building"
        assert detection.classification.location_type == "building"


def test_sim_projection_files_do_not_use_virtual_gimbal_pointing():
    forbidden = (
        "calc_gimbal_lock_att_loc",
        "calc_gimbal_lock_readback",
        "start_geo_tracking",
        "set_att(",
    )
    modules = (
        "navpy.modules.vision.sim.ideal_poi_projector",
        "navpy.modules.vision.sim.sim_poi_projector",
    )
    for module_name in modules:
        module = __import__(module_name, fromlist=["unused"])
        source = inspect.getsource(module)
        assert not any(term in source for term in forbidden)
