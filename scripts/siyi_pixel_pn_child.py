"""Run pure-vision PN from SIYI pixels while geo only points the gimbal."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
# BOTH roots, because this file is loaded two ways and each needs a different
# one. Run directly by the eval harness (`eval_direct_pixel_pn.py` sets
# PYTHONPATH to src only) it needs the WORKTREE on the path for `scripts.*` to
# resolve; imported by pytest as `scripts.siyi_pixel_pn_child` it needs src for
# `navpy.*`. Adding only src made the test suite fail to COLLECT; adding only
# the worktree would break the harness at launch.
for _import_root in (WORKTREE, WORKTREE / "src"):
    _path = str(_import_root)
    while _path in sys.path:
        sys.path.remove(_path)
    sys.path.insert(0, _path)

from navpy.args.conn_args import ConnArgs  # noqa: E402
from navpy.args.navigation_args import NavigationArgs  # noqa: E402
from navpy.args.logger_args import LoggerArgs  # noqa: E402
from navpy.args.mission_planner_args import MissionPlannerArgs  # noqa: E402
from navpy.args.navpy_argparse import make_parser  # noqa: E402
from navpy.args.uas_args import UasArgs  # noqa: E402
from navpy.logger.navigation_logger import ClosestSnap  # noqa: E402
from navpy.logger.logger_factory import initialize_logger  # noqa: E402
from navpy.modules.common.models.location import Location  # noqa: E402
from navpy.modules.common.scheduler_cadence import SchedulerCadence  # noqa: E402
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc  # noqa: E402
from navpy.modules.navigation.navigation import Navigation  # noqa: E402
from navpy.modules.navigation.mission_planner import MissionPlanner  # noqa: E402
from navpy.modules.vehicle.flight_mode import FlightMode  # noqa: E402
from navpy.modules.vehicle.vehicle_contract import IVehicle  # noqa: E402
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation  # noqa: E402
from navpy.modules.vision.sim.siyi_geo_pixel_source import (  # noqa: E402
    SiyiGeoPixelSource,
)
from navpy.modules.vision.models.detect_data import DetectedObject  # noqa: E402
from navpy.modules.vision.vision_profile_composition import build_vision_profile  # noqa: E402
from navpy.modules.vision.vision_profiles import (  # noqa: E402
    build_tracking_config,
    build_zoom_config,
)
from navpy.modules.vision.vision_class_profile import get_min_pixels_for_class  # noqa: E402
# QUALIFIED, not bare. A bare `from pixel_pn_flight_control_trace import ...`
# resolves only when scripts/ happens to be on sys.path -- true when this file
# is RUN, false when pytest imports it as `scripts.siyi_pixel_pn_child`. The
# bare form made the whole repository test suite fail to COLLECT, so nothing
# outside tests/scripts was being verified at all.
from scripts.pixel_pn_flight_control_trace import (  # noqa: E402
    FlightControlTrace,
)


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
    parser.add_argument("--scoring-active", required=True, type=Path, dest='scoring_active')
    return parser


def _navpy_args(options: argparse.Namespace) -> argparse.Namespace:
    return make_parser(description="direct pixel PN child").parse_args([
        "-c",
        options.connection,
        "-ss",
        str(options.sysid),
        "-ll",
        "DEBUG",
        "-lsd",
        "Vehicle",
        "-ut",
        "false",
        "-udt",
        "false",
        "--pitch-controller",
        "vision-nav-pn",
        "-pld",
        "-1",
        "-plrd",
        "-1",
        "-dt",
        "-1",
        "-da",
        "0",
    ])


def _wait_for_scoring_interval(vehicle: IVehicle, scoring_start_seq: int, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.is_armed and (
            vehicle.mission_items_next is not None
            and vehicle.mission_items_next >= scoring_start_seq
        ):
            return
        time.sleep(0.02)
    raise TimeoutError(
        f"mission did not reach scoring start sequence {scoring_start_seq}"
    )


def _wait_for_mode(vehicle: IVehicle, mode: FlightMode, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.get_mode is mode:
            return
        time.sleep(0.02)
    raise TimeoutError(f"vehicle did not enter {mode.value}")


def _command_poi_loiter(vehicle: IVehicle, poi: Location) -> None:
    location = vehicle.location(True)
    if location is None:
        raise RuntimeError("relative aircraft location unavailable for loiter")
    radius = abs(float(vehicle.get_param_or_default("WP_LOITER_RAD", 90.0)))
    if not math.isfinite(radius) or radius <= 0.0:
        raise RuntimeError(f"invalid WP_LOITER_RAD: {radius!r}")
    vehicle.goto_loiter(
        Location(poi.lat, poi.lng, location.alt, is_absolute=False),
        radius,
    )


def _command_poi_approach(vehicle: IVehicle, poi: Location) -> None:
    location = vehicle.location(True)
    if location is None:
        raise RuntimeError("relative aircraft location unavailable for approach")
    vehicle.goto(
        Location(poi.lat, poi.lng, location.alt, is_absolute=False)
    )


def _wait_for_visual_confirmation(
    navigation: Navigation,
    source: SiyiGeoPixelSource,
    timeout_s: float,
) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        detection = source.latest_detection
        if (
            detection is not None
            and navigation.final_approach.can_confirm_detection(detection)
            and navigation.final_approach.record_confirmed_detection(detection)
        ):
            return
        time.sleep(0.01)
    raise TimeoutError("SIYI POI never entered final-approach command authority")


def _initialize_confirmed_navigation(
    navigation: Navigation,
    source: SiyiGeoPixelSource,
    timeout_s: float,
) -> None:
    """Reset final-approach state before recording the frame that seeds handoff."""
    navigation.init()
    _wait_for_visual_confirmation(navigation, source, timeout_s)


def _result_payload(
    passed: bool, snap: ClosestSnap, source: SiyiGeoPixelSource
) -> dict[str, object]:
    metrics = source.metrics
    return {
        "passed": passed,
        "snap_3d_m": None if not math.isfinite(snap.dist) else snap.dist,
        "snap_horizontal_m": (
            None if not math.isfinite(snap.h_dist) else snap.h_dist
        ),
        "snap_vertical_m": (
            None if not math.isfinite(snap.v_dist) else snap.v_dist
        ),
        "source": {
            "projected_frames": metrics.visible_frames,
            "projection_failures": metrics.sight_losses,
            "delivered_frames": metrics.delivered_frames,
            "delivery_rejections": metrics.delivery_rejections,
            "first_sight_loss_distance_m": metrics.first_sight_loss_distance_m,
            "longest_sight_loss_run": metrics.longest_sight_loss_run,
            "max_gimbal_age_s": metrics.max_gimbal_age_s,
            "max_reference_attitude_delta_deg": (
                metrics.max_reference_attitude_delta_deg
            ),
            "max_visual_truth_ray_error_deg": (
                metrics.max_visual_truth_ray_error_deg
            ),
        },
    }


def run(options: argparse.Namespace) -> dict[str, object]:
    args = _navpy_args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = None
    cadence = None
    navigation = None
    source = None
    final_approach_recorded = False
    trace = FlightControlTrace(options.result.parent / "flight_control_trace.csv")
    try:
        vehicle = create_vehicle(ConnArgs(args), logger)
        cadence = SchedulerCadence(vehicle.sim_speedup)
        navigation_args = NavigationArgs(args, vehicle, logger)
        geo_ref = GeoRefCalc(UasArgs())
        navigation = Navigation(
            vehicle,
            geo_ref,
            None,
            MissionPlanner(logger, MissionPlannerArgs(args)),
            logger,
            navigation_args,
            scheduler_cadence=cadence,
        )
        poi = Location(
            options.poi_lat,
            options.poi_lon,
            options.poi_alt,
            is_absolute=True,
        )
        def deliver(poi_detection: DetectedObject) -> bool:
            nonlocal final_approach_recorded
            if not final_approach_recorded:
                final_approach_recorded = navigation.final_approach.record_confirmed_detection(
                    poi_detection
                )
                if not final_approach_recorded:
                    return False
            return navigation.nav(poi_detection)

        assembly = build_vision_profile("siyi_zr10", vehicle, logger, cadence)
        if len(assembly.mount_specs) != 1:
            raise RuntimeError("SIYI pixel navigation requires exactly one mount")
        spec = assembly.mount_specs[0]
        tracker = GimbalNavigation(
            spec.mount,
            logger,
            tracking=build_tracking_config(spec.device, sim=True),
            zoom_config=build_zoom_config(spec.device, assembly.profile),
        )
        source = SiyiGeoPixelSource(
            vehicle,
            poi,
            spec.mount,
            tracker,
            assembly.geo_ref,
            cadence,
            min_pixels=get_min_pixels_for_class(assembly.profile, 0),
            deliver=deliver,
        )
        navigation.bind_final_approach_source_dispatch(source.dispatch_available)
        spec.mount.start()
        tracker.start_geo_tracking(poi, assembly.geo_ref)
        source.start()
        navigation.start()
        print("SIYI_PIXEL_READY", flush=True)
        _wait_for_scoring_interval(vehicle, options.scoring_start_seq, options.timeout)
        acquisition_deadline_s = time.monotonic() + options.timeout
        while time.monotonic() < acquisition_deadline_s and not source.ready:
            spec.mount.raise_if_failed()
            time.sleep(cadence.wall_period_for_scheduler_period(0.02))
        if not source.ready:
            raise TimeoutError("SIYI did not acquire the geo-pointed POI")
        if not vehicle.set_mode(FlightMode.GUIDED):
            raise RuntimeError("GUIDED mode request was rejected")
        _wait_for_mode(vehicle, FlightMode.GUIDED, 5.0)
        _initialize_confirmed_navigation(navigation, source, options.timeout)
        final_approach_recorded = True
        source.activate()
        options.scoring_active.write_text("SIYI_PIXEL_NAV\n", encoding="utf-8")
        print("SIYI_PIXEL_NAV", flush=True)
        deadline_s = time.monotonic() + options.timeout
        while time.monotonic() < deadline_s:
            navigation.raise_if_failed()
            trace.sample(vehicle)
            if navigation.final_approach.poi_passed_override() is True:
                snap = navigation.reset()
                return _result_payload(True, snap, source)
            time.sleep(0.01)
        snap = navigation.reset()
        return _result_payload(False, snap, source)
    finally:
        trace.write()
        if source is not None:
            source.close()
        if navigation is not None:
            navigation.stop()
        if 'tracker' in locals():
            tracker.stop_geo_tracking()
        if 'spec' in locals():
            spec.mount.stop()
        if cadence is not None:
            cadence.close()
        if vehicle is not None:
            vehicle.close()
        logger.close()


def main() -> int:
    options = _parser().parse_args()
    try:
        payload = run(options)
    except BaseException as error:
        payload = {"passed": False, "error": f"{type(error).__name__}: {error}"}
    options.result.parent.mkdir(parents=True, exist_ok=True)
    options.result.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("SIYI_PIXEL_RESULT " + json.dumps(payload), flush=True)
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
