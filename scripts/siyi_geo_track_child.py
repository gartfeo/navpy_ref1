"""Track one known geo POI with the SIYI simulator, without detection."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pymap3d

WORKTREE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKTREE / "src"))

from navpy.args.conn_args import ConnArgs  # noqa: E402
from navpy.args.logger_args import LoggerArgs  # noqa: E402
from navpy.args.navpy_argparse import make_parser  # noqa: E402
from navpy.logger.logger_factory import initialize_logger  # noqa: E402
from navpy.modules.common.models.location import Location  # noqa: E402
from navpy.modules.common.scheduler_cadence import SchedulerCadence  # noqa: E402
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation  # noqa: E402
from navpy.modules.vehicle.pose_streams import request_pose_streams  # noqa: E402
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402
from navpy.modules.vision.vision_camera_calibration import (  # noqa: E402
    read_camera_zoom_calibration,
)
from navpy.modules.vision.vision_profile_composition import (  # noqa: E402
    build_vision_profile,
)
from navpy.modules.vision.vision_profiles import (  # noqa: E402
    build_tracking_config,
    build_zoom_config,
)
from navpy.modules.vision.vision_class_profile import (  # noqa: E402
    get_class_detect_size,
    get_min_pixels_for_class,
)


REQUIRED_POST_ACQUISITION_SAMPLES = 1000


@dataclass(frozen=True)
class GeoTrackMetrics:
    samples: int
    in_frame_samples: int
    post_acquisition_losses: int
    post_acquisition_max_center_error_px: float
    settled_center_error_px: float
    post_acquisition_max_center_angle_deg: float
    selected_zoom: str | None
    zoom_level: float | None
    projected_poi_px: float | None
    acquisition_reached: bool
    acquisition_sample: int | None
    post_acquisition_samples: int
    expected_zoom: str | None


@dataclass(frozen=True)
class _SiyiRig:
    mount: object
    tracker: GimbalNavigation
    geo_ref: object
    profile: object


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", required=True)
    parser.add_argument("--sysid", required=True, type=int)
    parser.add_argument("--poi-lat", required=True, type=float)
    parser.add_argument("--poi-lon", required=True, type=float)
    parser.add_argument("--poi-alt", required=True, type=float)
    parser.add_argument("--scoring-start-seq", required=True, type=int, dest='scoring_start_seq')
    parser.add_argument("--timeout", required=True, type=float)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--ready", required=True, type=Path)
    return parser


def _args(options: argparse.Namespace) -> argparse.Namespace:
    return make_parser(description="SIYI direct geo tracking child").parse_args([
        "-c", options.connection,
        "-ss", str(options.sysid),
        "-ll", "DEBUG",
        "-lsd", "Vehicle",
        "--vision-profile", "siyi_zr10",
        "--detector-type", "sim",
        "-ut", "false",
        "-udt", "false",
        "--pitch-controller", "vision-nav-pn",
        "-pld", "-1",
        "-plrd", "-1",
        "-dt", "-1",
        "-da", "0",
    ])


def _wait_for_scoring_interval(vehicle: object, sequence: int, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if (
            vehicle.is_armed
            and vehicle.mission_items_next is not None
            and vehicle.mission_items_next >= sequence
        ):
            return
        time.sleep(0.02)
    raise TimeoutError(f"mission did not reach tracking sequence {sequence}")


def _command_poi_loiter(vehicle: object, poi: Location) -> None:
    """Put the aircraft in the POI-centred geometry used for confirmation."""
    location = vehicle.location(True)
    if location is None:
        raise RuntimeError("relative aircraft location unavailable for loiter")
    radius = abs(float(vehicle.get_param_or_default("WP_LOITER_RAD", 90.0)))
    if not math.isfinite(radius) or radius <= 0.0:
        raise RuntimeError(f"invalid WP_LOITER_RAD for SIYI test: {radius!r}")
    vehicle.goto_loiter(
        Location(poi.lat, poi.lng, location.alt, is_absolute=False),
        radius,
    )


def _expected_zoom(rig: _SiyiRig, slant_m: float, minimum: float) -> str:
    calibrations = read_camera_zoom_calibration(rig.mount.camera)
    for candidate in calibrations:
        if candidate.fy * get_class_detect_size(0) / slant_m >= minimum:
            return candidate.zoom
    return calibrations[-1].zoom


def _zoom_matches_command(rig: _SiyiRig, command: str | None) -> bool:
    if command is None:
        return False
    return abs(float(command) - float(rig.mount.gimbal.get_zoom_level())) < 0.1


def _sample(rig: _SiyiRig, vehicle: object, poi: Location) -> dict:
    mount = rig.mount
    location = vehicle.location(False)
    attitude = vehicle.attitude
    if location is None or attitude is None:
        return {"valid": False}
    width = mount.image_width
    height = mount.image_height
    if width is None or height is None:
        return {"valid": False}
    frame = mount.capture_frame_state(width, height)
    if frame is None:
        return {"valid": False}
    ned = pymap3d.geodetic2ned(
        poi.lat, poi.lng, poi.alt,
        location.lat, location.lng, location.alt,
    )
    k = frame.k
    projection_attitude = (
        frame.gimbal_data.reference_aircraft_attitude or attitude
    )
    uv = rig.geo_ref.calc_uv(
        ned,
        k,
        frame.gimbal_data,
        projection_attitude,
    )
    if uv[0] is None or uv[1] is None:
        return {"valid": True, "in_frame": False, "reason": "behind_cam"}
    u, v = float(uv[0]), float(uv[1])
    slant_m = float(np.linalg.norm(ned))
    return {
        "valid": True,
        "in_frame": bool(mount.is_valid(u, v)),
        "center_error_px": math.hypot(u - float(k[0, 2]), v - float(k[1, 2])),
        "u_px": u,
        "v_px": v,
        "fx_px": float(k[0, 0]),
        "fy_px": float(k[1, 1]),
        "cx_px": float(k[0, 2]),
        "cy_px": float(k[1, 2]),
        "projected_poi_px": (
            float(k[1, 1]) * get_class_detect_size(0) / max(slant_m, 1.0)
        ),
        "slant_m": slant_m,
        "zoom_level": float(mount.gimbal.get_zoom_level()),
        "measured_zoom_command": frame.zoom_command,
    }


def run(options: argparse.Namespace) -> GeoTrackMetrics:
    args = _args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = cadence = rig = None
    poi = Location(
        options.poi_lat,
        options.poi_lon,
        options.poi_alt,
        is_absolute=True,
    )
    samples = in_frame = losses = 0
    post_acquisition_errors: list[float] = []
    post_acquisition_angles: list[float] = []
    acquired = False
    acquisition_sample: int | None = None
    commanded_zoom: str | None = None
    post_acquisition_samples = 0
    last: dict = {}
    try:
        vehicle = create_vehicle(ConnArgs(args), logger)
        cadence = SchedulerCadence(vehicle.sim_speedup)
        assembly = build_vision_profile(
            "siyi_zr10",
            vehicle,
            logger,
            cadence,
        )
        if len(assembly.mount_specs) != 1:
            raise RuntimeError("SIYI certificate requires exactly one mount")
        spec = assembly.mount_specs[0]
        tracker = GimbalNavigation(
            spec.mount,
            logger,
            tracking=build_tracking_config(spec.device, sim=True),
            zoom_config=build_zoom_config(spec.device, assembly.profile),
        )
        rig = _SiyiRig(spec.mount, tracker, assembly.geo_ref, assembly.profile)
        request_pose_streams(vehicle)
        rig.mount.start()
        rig.tracker.start_geo_tracking(poi, rig.geo_ref)
        options.ready.write_text("SIYI_GEO_READY\n", encoding="utf-8")
        print("SIYI_GEO_READY", flush=True)
        _wait_for_scoring_interval(vehicle, options.scoring_start_seq, options.timeout)
        _command_poi_loiter(vehicle, poi)
        min_pixels = get_min_pixels_for_class(rig.profile, 0)
        deadline_s = time.monotonic() + options.timeout
        while time.monotonic() < deadline_s:
            rig.mount.raise_if_failed()
            location = vehicle.location(False)
            attitude = vehicle.attitude
            if location is not None and attitude is not None:
                rig.tracker.update_geo(location, attitude)
                zoom_changed = rig.tracker.prepare_geo_acquisition(
                    location, attitude, 0, min_pixels,
                )
                if zoom_changed:
                    commanded_zoom = rig.mount.camera.get_zoom_key()
                last = _sample(rig, vehicle, poi)
                if last.get("valid"):
                    samples += 1
                    if last.get("in_frame"):
                        in_frame += 1
                        error_px = float(last["center_error_px"])
                        reached = (
                            error_px < 5.0
                            and float(last["projected_poi_px"]) >= min_pixels
                        )
                        if reached and not acquired:
                            acquired = True
                            acquisition_sample = samples
                        if acquired:
                            post_acquisition_errors.append(error_px)
                            angular_error_deg = math.degrees(math.hypot(
                                (float(last["u_px"]) - float(last["cx_px"]))
                                / float(last["fx_px"]),
                                (float(last["v_px"]) - float(last["cy_px"]))
                                / float(last["fy_px"]),
                            ))
                            post_acquisition_angles.append(angular_error_deg)
                            post_acquisition_samples += 1
                    elif acquired:
                        losses += 1
                        post_acquisition_samples += 1
                    if (
                        post_acquisition_samples >= REQUIRED_POST_ACQUISITION_SAMPLES
                        and _zoom_matches_command(rig, commanded_zoom)
                    ):
                        break
            time.sleep(cadence.wall_period_for_scheduler_period(0.02))
        return GeoTrackMetrics(
            samples=samples,
            in_frame_samples=in_frame,
            post_acquisition_losses=losses,
            post_acquisition_max_center_error_px=max(
                post_acquisition_errors,
                default=float("inf"),
            ),
            settled_center_error_px=(
                post_acquisition_errors[-1]
                if post_acquisition_errors
                else float("inf")
            ),
            post_acquisition_max_center_angle_deg=max(
                post_acquisition_angles,
                default=float("inf"),
            ),
            selected_zoom=commanded_zoom,
            zoom_level=last.get("zoom_level"),
            projected_poi_px=last.get("projected_poi_px"),
            acquisition_reached=acquired,
            acquisition_sample=acquisition_sample,
            post_acquisition_samples=post_acquisition_samples,
            expected_zoom=(
                None
                if not last.get("slant_m")
                else _expected_zoom(rig, float(last["slant_m"]), min_pixels)
            ),
        )
    finally:
        if rig is not None:
            try:
                rig.tracker.stop_geo_tracking()
            finally:
                rig.mount.stop()
        if cadence is not None:
            cadence.close()
        if vehicle is not None:
            vehicle.close()
        logger.close()


def main() -> int:
    options = _parser().parse_args()
    try:
        metrics = run(options)
        zoom_matches = (
            metrics.selected_zoom is not None
            and metrics.zoom_level is not None
            and abs(float(metrics.selected_zoom) - metrics.zoom_level) < 0.1
        )
        passed = (
            metrics.acquisition_reached
            and metrics.post_acquisition_samples
            >= REQUIRED_POST_ACQUISITION_SAMPLES
            and metrics.post_acquisition_losses == 0
            and metrics.selected_zoom == metrics.expected_zoom
            and zoom_matches
        )
        payload = {"passed": passed, "metrics": asdict(metrics)}
    except BaseException as error:
        payload = {"passed": False, "error": f"{type(error).__name__}: {error}"}
    options.result.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("SIYI_GEO_RESULT " + json.dumps(payload), flush=True)
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
