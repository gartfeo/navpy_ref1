"""Own-detection orbit centre from the ChArUco board pose (HITL step 22).

Synthetic scenes: the scene board (5 x 7 squares of 400 mm) lies flat where
a gimbal camera on an aircraft looks; its corners go through the real
camera/gimbal/aircraft chain to camera points and through OpenCV's
projection with distortion to pixels, plus 0.45 px corner noise (the RMS
measured on rendered, blurred, JPEG-compressed boards with navpy's ChArUco
detector at 16-30 px squares). The fit's centre is moved to an absolute
location exactly as acquisition does, and compared with the board's truth.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pymap3d
import pytest

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.config.uas_args import UasArgs
from navpy.core.kernel.location import Location
from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.perception.camera.frame_publication import FramePublicationStore
from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
from navpy.core.perception.pipeline.capture_position import (
    MAX_CAPTURE_POSITION_SHIFT_S,
    capture_location,
)
from navpy.core.perception.pipeline.real_board_pose import TrackBoardPose
from navpy.core.perception.pipeline.real_frame_association import FrameAssociationBuilder
from navpy.core.ports.board_pose import BoardPoseFit, BoardPoseRejection
from navpy.core.ports.detection import BoardCorners, Detection
from navpy.core.ports.detection_components import DetectionGeoDiagnostics, PoseProvenance
from navpy.core.ports.frame_capture_ports import CaptureStamp, CaptureStampKind
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.tracks import TrackedObject
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.skills.geo_approach.board_orbit_centre import (
    BoardOrbitCentre,
    board_orbit_centre,
)
from navpy.core.skills.geo_approach.self_detected_approach import (
    WAITING_CIRCLE_REFERENCE_HEIGHT_M,
    SelfDetectedApproach,
    SelfDetectedApproachPorts,
)
from navpy.plugins.detection.charuco import board_pose as board_pose_module
from navpy.plugins.detection.charuco.board_pose import (
    MAX_REPROJECTION_P95_PX,
    CharucoBoardPoseSolver,
)
from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec
from navpy.plugins.detection.charuco.charuco_detector import (
    CharucoBoardDetector,
    build_charuco_board,
)
from tests.detection_factory import make_detected_poi
from tests.modules.vision.test_capture_time_association_e2e import (
    BASE_S,
    _attitude_history,
    _calibration,
)

SPEC = CharucoBoardSpec(square_mm=400.0, marker_mm=296.0)
CENTRE_ERROR_LIMIT_M = 0.5  # plan step 22 done limit
CORNER_NOISE_PX = 0.45      # measured ChArUco corner RMS (module docstring)
DIST = np.array([-0.05, 0.02, 0.0005, -0.0003, 0.0])
GEO = GeoRefCalc(UasArgs())
CAMERA = Location(40.15, 44.40, 1180.0, is_absolute=True)
HOME = Location(40.149, 44.401, 1100.0, is_absolute=True)  # 80 m below the camera
ATTITUDE = Attitude(pitch=3.0, yaw=30.0, roll=-8.0)
GIMBAL = GimbalData(att=Attitude(pitch=-25.0, yaw=12.0, roll=0.0))


def _k(f: float) -> np.ndarray:
    return np.array([[f, 0.0, 640.0], [0.0, f, 360.0], [0.0, 0.0, 1.0]])


K10 = _k(10332.4)   # zoom 10 at 1280 x 720 (siyi_zr10 intrinsics)
K5 = _k(5166.2)
K3 = _k(3099.7)


def _board_points_m() -> np.ndarray:
    """The 24 interior corners by id, board-centred, in metres (own model)."""
    cols, rows = SPEC.squares_x - 1, SPEC.squares_y - 1
    width, height = (value / 1000.0 for value in SPEC.size_mm)
    square = SPEC.square_mm / 1000.0
    return np.array([
        [(i % cols + 1) * square - width / 2, (i // cols + 1) * square - height / 2, 0.0]
        for i in range(cols * rows)
    ])


def _affine(gimbal: GimbalData, attitude: Attitude) -> tuple[np.ndarray, np.ndarray]:
    origin = GEO.camera_point_to_ned(np.zeros(3), gimbal, attitude)
    columns = [GEO.camera_point_to_ned(axis, gimbal, attitude) - origin for axis in np.eye(3)]
    return np.column_stack(columns), origin


def _scene(slant_m, *, gimbal=GIMBAL, attitude=ATTITUDE, heading_deg=20.0, face_up=True):
    """Board centre NED (from the camera) and its corners as camera points."""
    a, origin = _affine(gimbal, attitude)
    direction = np.array([0.03, -0.02, 1.0])
    centre_ned = a @ (slant_m * direction / np.linalg.norm(direction)) + origin
    assert centre_ned[2] > 0.0  # the camera looks down at the board
    h = math.radians(heading_deg)
    # Board x east-ish, y south-ish, z (into the printed face) down: face up.
    x_axis = np.array([-math.sin(h), math.cos(h), 0.0])
    y_axis = np.array([-math.cos(h), -math.sin(h), 0.0])
    z_axis = np.cross(x_axis, y_axis)
    if not face_up:
        y_axis, z_axis = -y_axis, -z_axis
    rotation = np.column_stack([x_axis, y_axis, z_axis])
    assert z_axis[2] == pytest.approx(1.0 if face_up else -1.0)
    world = centre_ned + _board_points_m() @ rotation.T
    camera = np.linalg.solve(a, (world - origin).T).T
    return centre_ned, camera


def _pixels(camera_points, k, rng=None, noise_px=CORNER_NOISE_PX):
    image, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), k, DIST)
    image = image.reshape(-1, 2)
    if rng is not None:
        image = image + rng.normal(0.0, noise_px, image.shape)
    return image


def _corners(image, ids=None) -> BoardCorners:
    ids = tuple(range(len(image))) if ids is None else tuple(ids)
    return BoardCorners(ids, tuple((float(u), float(v)) for u, v in image))



def _error_m(location: Location, centre_ned: np.ndarray, origin: Location = CAMERA) -> float:
    # An orbit centre is home-relative (like the ground ray); a camera fix is absolute.
    alt = location.alt if location.is_absolute else location.alt + HOME.alt
    north, east, down = pymap3d.geodetic2ned(
        location.lat, location.lng, alt, origin.lat, origin.lng, origin.alt,
    )
    return float(np.linalg.norm(np.array([north, east, down]) - centre_ned))


def _poi(fit, *, camera=CAMERA, attitude=ATTITUDE, gimbal=GIMBAL, atomic=True):
    poi = make_detected_poi(uas_att=attitude, g_data=gimbal, pose_is_frame_atomic=atomic)
    poi.geo = DetectionGeoDiagnostics(board_pose=fit, capture_camera_location=camera)
    poi.pose = PoseProvenance(aircraft_attitude=attitude, gimbal_data=gimbal, is_frame_atomic=atomic)
    return poi


SOLVER = CharucoBoardPoseSolver(SPEC)


@pytest.mark.parametrize(("slant_m", "k"), [(100.0, K10), (60.0, K5), (25.0, K3)])
def test_projected_corners_give_the_centre_within_limits(slant_m, k):
    rng = np.random.default_rng(int(slant_m))
    centre_ned, camera_points = _scene(slant_m)
    errors, p95s = [], []
    for _ in range(40):
        fit = SOLVER.solve(_corners(_pixels(camera_points, k, rng)), k, DIST)
        assert isinstance(fit, BoardPoseFit), fit
        centre = board_orbit_centre(_poi(fit), GEO.camera_point_to_ned, HOME)
        errors.append(_error_m(centre.location, centre_ned))
        p95s.append(fit.reprojection_p95_px)
    assert np.percentile(errors, 95) <= CENTRE_ERROR_LIMIT_M
    assert max(p95s) <= MAX_REPROJECTION_P95_PX


def test_noise_free_corners_recover_the_centre_exactly():
    centre_ned, camera_points = _scene(70.0)
    fit = SOLVER.solve(_corners(_pixels(camera_points, K10)), K10, DIST)
    centre = board_orbit_centre(_poi(fit), GEO.camera_point_to_ned, HOME)
    assert _error_m(centre.location, centre_ned) < 0.01
    assert fit.reprojection_p95_px < 0.01


def test_far_board_is_refused_as_uncertain():
    # Step 23's start geometry (170 m radius, 80 m above: about 188 m slant)
    # at zoom 10: the PnP depth is not good to 0.5 m there.
    _, camera_points = _scene(188.0)
    rng = np.random.default_rng(7)
    fit = SOLVER.solve(_corners(_pixels(camera_points, K10, rng)), K10, DIST)
    assert fit == BoardPoseRejection("centre_uncertain", 24)


def _clean_pixels():
    _, camera_points = _scene(40.0)
    return _pixels(camera_points, K5)


@pytest.mark.parametrize(("ids", "reason"), [
    (range(5), "too_few_corners"),
    (range(4), "too_few_corners"),
    ((0, 1, 2, 3, 0, 1), "bad_corner_ids"),
    ((0, 1, 2, 3, 4, 99), "bad_corner_ids"),
    ((0, 4, 8, 12, 16, 20), "collinear_corners"),
])
def test_sparse_or_inconsistent_corners_are_refused(ids, reason):
    image = _clean_pixels()
    picked = [image[i % 24] for i in ids]
    result = SOLVER.solve(_corners(picked, ids), K5, DIST)
    assert result == BoardPoseRejection(reason, len(tuple(ids)))


def test_non_finite_input_is_refused():
    image = _clean_pixels()
    image[3, 0] = math.nan
    assert SOLVER.solve(_corners(image), K5, DIST).reason == "non_finite_input"
    assert SOLVER.solve(_corners(_clean_pixels()), K5 * math.inf, DIST).reason == "non_finite_input"


def test_bad_fit_is_refused_on_reprojection():
    image = _clean_pixels()
    image[[2, 9, 17]] += 12.0
    assert SOLVER.solve(_corners(image), K5, DIST).reason == "reprojection"


def test_board_seen_from_behind_is_refused():
    _, camera_points = _scene(40.0, face_up=False)
    result = SOLVER.solve(_corners(_pixels(camera_points, K5)), K5, DIST)
    assert result.reason == "board_faces_away"


def _rival_solve(gap_m: float):
    real = cv2.solvePnPGeneric

    def rival(*args, **kwargs):
        solved, rvecs, tvecs, errors = real(*args, **kwargs)
        best = int(np.argmin(np.ravel(errors)))
        moved = tvecs[best].copy()
        moved[0] += gap_m
        return solved, (rvecs[best], rvecs[best]), (tvecs[best], moved), np.array([[0.4], [0.5]])
    return rival


def test_ambiguous_fit_with_a_distant_rival_centre_is_refused():
    with patch.object(board_pose_module.cv2, "solvePnPGeneric", _rival_solve(1.0)):
        result = SOLVER.solve(_corners(_clean_pixels()), K5, DIST)
    assert result.reason == "ambiguous"


def test_rival_with_the_same_centre_is_not_ambiguous():
    with patch.object(board_pose_module.cv2, "solvePnPGeneric", _rival_solve(0.05)):
        result = SOLVER.solve(_corners(_clean_pixels()), K5, DIST)
    assert isinstance(result, BoardPoseFit)
    assert result.ambiguity_ratio == pytest.approx(1.25)


# Delayed frames from a moving camera -------------------------------------

SPEED_MPS = 24.0
TRACK_DEG = 30.0
FRAME_DELAY_S = 0.15  # the newest fix is this much newer than the capture


def _velocity() -> tuple[float, float]:
    return (SPEED_MPS * math.cos(math.radians(TRACK_DEG)),
            SPEED_MPS * math.sin(math.radians(TRACK_DEG)))


def _fix_after(camera_at_capture: Location, delay_s: float, capture_boot_s: float):
    vn, ve = _velocity()
    lat, lon, alt = pymap3d.ned2geodetic(
        vn * delay_s, ve * delay_s, 0.0,
        camera_at_capture.lat, camera_at_capture.lng, camera_at_capture.alt,
    )
    return SimpleNamespace(lat=lat, lon=lon, alt=alt, vn=vn, ve=ve, vd=0.0,
                           time_boot_s=capture_boot_s + delay_s)


def test_delayed_frame_uses_the_camera_position_and_pose_at_capture():
    capture_boot_s = 412.0
    centre_ned, camera_points = _scene(70.0)
    fit = SOLVER.solve(_corners(_pixels(camera_points, K10)), K10, DIST)
    fix = _fix_after(CAMERA, FRAME_DELAY_S, capture_boot_s)
    at_capture = capture_location(fix, capture_boot_s)
    centre = board_orbit_centre(_poi(fit, camera=at_capture), GEO.camera_point_to_ned, HOME)
    assert _error_m(centre.location, centre_ned) < 0.05

    # Mutation: the newest fix instead of the capture-time position.
    latest = Location(fix.lat, fix.lon, fix.alt, is_absolute=True)
    stale = board_orbit_centre(_poi(fit, camera=latest), GEO.camera_point_to_ned, HOME)
    assert _error_m(stale.location, centre_ned) > SPEED_MPS * FRAME_DELAY_S - 0.1

    # Mutation: the newest attitude (the aircraft rolled 6 degrees since).
    turned = Attitude(pitch=ATTITUDE.pitch, yaw=ATTITUDE.yaw, roll=ATTITUDE.roll + 6.0)
    late_pose = board_orbit_centre(_poi(fit, camera=at_capture, attitude=turned),
                                   GEO.camera_point_to_ned, HOME)
    assert _error_m(late_pose.location, centre_ned) > CENTRE_ERROR_LIMIT_M


@pytest.mark.parametrize("shift_s", [MAX_CAPTURE_POSITION_SHIFT_S + 0.01,
                                     -(MAX_CAPTURE_POSITION_SHIFT_S + 0.01)])
def test_a_fix_too_far_from_the_capture_is_not_used(shift_s):
    fix = _fix_after(CAMERA, shift_s, 100.0)
    assert capture_location(fix, 100.0) is None


@pytest.mark.parametrize("bad", ["vn", "time_boot_s", "lat"])
def test_a_fix_without_finite_values_is_not_used(bad):
    fix = _fix_after(CAMERA, 0.1, 100.0)
    setattr(fix, bad, math.nan)
    assert capture_location(fix, 100.0) is None
    assert capture_location(None, 100.0) is None
    assert capture_location(_fix_after(CAMERA, 0.1, 100.0), None) is None


def _mount(width, height):
    return CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(0, 0, 0)), k=_k(1000.0), dist=np.zeros(5),
        gimbal_timestamp_s=None, gimbal_is_static=True, zoom_command=None,
        zoom_sample_id="static:fixed", zoom_sample_age_s=0.0,
    )


def _associate(position_sample_reader, exposure_s=BASE_S + 1.0, bracketed=True):
    read_s = exposure_s + 0.180
    store = FramePublicationStore(clock=lambda: read_s + 0.001)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    store.publish(frame, CaptureStamp(read_s, CaptureStampKind.READ))
    history = _attitude_history(exposure_s + (0.2 if bracketed else -0.5))
    builder = FrameAssociationBuilder(
        attitude_sample_reader=lambda: history[-1],
        mount_state_reader=_mount,
        location_reader=lambda: CAMERA,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=_calibration(),
        source_id="rtsp-cam",
        position_sample_reader=position_sample_reader,
    )
    with patch("navpy.core.perception.pipeline.real_frame_association.rt.perf_counter",
               return_value=read_s + 0.005):
        return builder.capture(store.snapshot(), frame)


def test_association_places_the_camera_at_the_capture_boot_time():
    exposure = BASE_S + 1.0
    capture_boot = exposure - BASE_S + 500.0  # the history's boot clock
    fix = _fix_after(CAMERA, FRAME_DELAY_S, capture_boot)
    association = _associate(lambda: fix, exposure)
    assert association.pose_is_frame_atomic
    error = _error_m(association.capture_location, np.zeros(3))
    # Within the capture estimate's residual bound (6 ms) at 24 m/s.
    assert error <= SPEED_MPS * 0.006 + 1e-3
    assert association.c_g_loc == CAMERA  # the diagnostic latest fix is unchanged


def test_association_without_a_bracketed_capture_has_no_capture_location():
    fix = _fix_after(CAMERA, 0.0, 500.0)
    assert _associate(lambda: fix, bracketed=False).capture_location is None
    assert _associate(None).capture_location is None

    def broken():
        raise RuntimeError("telemetry gone")
    assert _associate(broken).capture_location is None


# Centre guards -------------------------------------------------------------

def _fit():
    _, camera_points = _scene(40.0)
    return SOLVER.solve(_corners(_pixels(camera_points, K5)), K5, DIST)


@pytest.mark.parametrize(("poi_kwargs", "reason"), [
    ({"atomic": False}, "pose_not_at_capture"),
    ({"camera": None}, "no_capture_location"),
    ({"camera": Location(40.15, 44.40, 30.0, is_absolute=False)}, "no_capture_location"),
])
def test_centre_needs_capture_time_state(poi_kwargs, reason):
    centre = board_orbit_centre(_poi(_fit(), **poi_kwargs), GEO.camera_point_to_ned, HOME)
    assert centre == BoardOrbitCentre(None, reason)


@pytest.mark.parametrize("home", [None, Location(40.15, 44.40, 0.0, is_absolute=False),
                                  Location(40.15, 44.40, math.nan, is_absolute=True)])
def test_centre_needs_an_absolute_home(home):
    centre = board_orbit_centre(_poi(_fit()), GEO.camera_point_to_ned, home)
    assert centre == BoardOrbitCentre(None, "no_home")


def test_centre_is_home_relative_for_the_orbit_planner():
    """Codex review: the planner's fallback path subtracts the POI height from
    the aircraft's home-relative height, so the centre must be home-relative."""
    centre_ned, camera_points = _scene(60.0)
    fit = SOLVER.solve(_corners(_pixels(camera_points, K5)), K5, DIST)
    centre = board_orbit_centre(_poi(fit), GEO.camera_point_to_ned, HOME).location
    aircraft_rel = CAMERA.alt - HOME.alt
    assert centre.is_absolute is False
    assert aircraft_rel - centre.alt == pytest.approx(centre_ned[2], abs=0.05)


def test_centre_passes_on_refusals_and_absent_poses():
    rejection = BoardPoseRejection("reprojection", 12)
    assert board_orbit_centre(_poi(rejection), GEO.camera_point_to_ned, HOME) == BoardOrbitCentre(
        None, "reprojection")
    assert board_orbit_centre(_poi(None), GEO.camera_point_to_ned, HOME) is None
    bad = board_orbit_centre(_poi(_fit()), lambda *args: np.array([math.nan, 0.0, 0.0]), HOME)
    assert bad == BoardOrbitCentre(None, "non_finite_centre")


# Detector, track matching and the waiting orbit ------------------------------

def test_detector_reports_the_corners_it_found():
    board = build_charuco_board(SPEC)
    image = board.generateImage((5 * 60 + 80, 7 * 60 + 80), marginSize=40)
    frame = np.full((720, 1280), 255, dtype=np.uint8)
    frame[100:100 + image.shape[0], 200:200 + image.shape[1]] = image
    detections = CharucoBoardDetector(SPEC).detect(frame)
    assert len(detections) == 1
    corners = detections[0].board
    assert len(corners.ids) == SPEC.interior_corner_count
    corner_mm = np.asarray(board.getChessboardCorners())[list(corners.ids), :2]
    expected = corner_mm / SPEC.square_mm * 60.0 + np.array([240.0, 140.0])
    assert np.max(np.abs(np.asarray(corners.points_px) - expected)) < 1.0


def _track(missed=0, cx=640.0, class_id=0):
    return TrackedObject(id=4, cx=cx, cy=360.0, w=100.0, h=140.0, confidence=0.9,
                         class_id=class_id, age=5, hits=5, missed=missed, is_confirmed=True,
                         timestamp=0.0, vx=0.0, vy=0.0)


def _board_detection(cx=642.0):
    return Detection(cx=cx, cy=361.0, w=98.0, h=138.0, confidence=1.0, class_id=0,
                     board=_corners(_clean_pixels()))


def test_track_gets_the_pose_of_its_own_board_on_its_own_frame():
    mount = SimpleNamespace(k=K5, dist=DIST)
    attach = TrackBoardPose(SOLVER)
    assert isinstance(attach.for_track(_track(), [_board_detection()], mount), BoardPoseFit)
    assert attach.for_track(_track(missed=1), [_board_detection()], mount) is None
    assert attach.for_track(_track(cx=900.0), [_board_detection()], mount) is None
    assert attach.for_track(_track(class_id=1), [_board_detection()], mount) is None
    plain = Detection(cx=642.0, cy=361.0, w=98.0, h=138.0, confidence=1.0, class_id=0)
    assert attach.for_track(_track(), [plain], mount) is None
    assert TrackBoardPose(None).for_track(_track(), [_board_detection()], mount) is None


def _approach(board):
    ground = Location(40.2, 44.5, 0.0)
    plan = SimpleNamespace(orbit_radius=170.0)
    ports = SelfDetectedApproachPorts(
        final_approach_active=lambda: True,
        detector_is_simulation=lambda: False,
        ground_location=Mock(return_value=ground),
        current_relative=lambda: None,
        plan_orbit=Mock(return_value=(plan, 80.0)),
        absolute_location=lambda location: location,
        save_loiter_radius=lambda: True,
        request_guided=lambda: None,
        loiter_poi=Mock(),
        needs_own_orbit=lambda poi: False,
        board_centre=lambda poi: board,
    )
    task = SimpleNamespace(orbit_radius_m=0.0, orbit_approach_alt_rel_m=None,
                           navigation_poi_location=None)
    logger = Mock()
    approach = SelfDetectedApproach(ports, task, ApproachKind.ORBIT, logger)
    approach.prepare(make_detected_poi())
    return ports, task, logger, ground


def test_waiting_orbit_is_centred_on_the_board_pose():
    centre = Location(40.21, 44.51, 1100.0, is_absolute=True)
    ports, task, _, _ = _approach(BoardOrbitCentre(centre))
    ports.loiter_poi.assert_called_once_with(centre, 170.0, 80.0)
    ports.ground_location.assert_not_called()
    assert task.navigation_poi_location is None  # nothing stored for NAV


def test_refused_board_pose_falls_back_to_the_ground_ray_with_a_warning():
    ports, task, logger, ground = _approach(BoardOrbitCentre(None, "reprojection"))
    ports.ground_location.assert_called_once()
    assert ports.ground_location.call_args.kwargs["reference_height_m"] == (
        WAITING_CIRCLE_REFERENCE_HEIGHT_M)
    ports.loiter_poi.assert_called_once_with(ground, 170.0, 80.0)
    assert "reprojection" in logger.warning.call_args.args[0]
    assert task.navigation_poi_location is None


def test_no_board_pose_keeps_the_ground_ray():
    ports, _, logger, ground = _approach(None)
    ports.loiter_poi.assert_called_once_with(ground, 170.0, 80.0)
    logger.warning.assert_not_called()
