"""Run pure-vision PN from a known POI rendered as ideal pixels."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
# Running as a script puts this directory on the path for free; importing it as
# `scripts.direct_pixel_pn_child`, as the tests do, does not.
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(SCRIPTS)]

from navpy.args.conn_args import ConnArgs  # noqa: E402
from navpy.args.navigation_args import NavigationArgs  # noqa: E402
from navpy.args.logger_args import LoggerArgs  # noqa: E402
from navpy.args.mission_planner_args import MissionPlannerArgs  # noqa: E402
from navpy.args.navpy_argparse import make_parser  # noqa: E402
from navpy.args.uas_args import UasArgs  # noqa: E402
from navpy.logger.navigation_snap_types import ClosestSnap  # noqa: E402
from navpy.logger.logger_factory import initialize_logger  # noqa: E402
from navpy.modules.common.models.location import Location  # noqa: E402
from navpy.modules.common.scheduler_cadence import SchedulerCadence  # noqa: E402
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc  # noqa: E402
from navpy.modules.navigation.navigation import Navigation  # noqa: E402
from navpy.modules.navigation.mission_planner import MissionPlanner  # noqa: E402
from navpy.modules.vehicle.flight_mode import FlightMode  # noqa: E402
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402
from navpy.modules.vehicle.vehicle_interface import IVehicle  # noqa: E402
from navpy.modules.vision.models.detect_data import DetectedObject  # noqa: E402
from navpy.modules.vision.sim.direct_pixel_trace import (  # noqa: E402
    command_loop_observer,
)
from navpy.modules.vision.sim.direct_poi_pixel_source import (  # noqa: E402
    DirectPoiPixelSource,
)
from pixel_pn_admission_ledger import subscribe_admission_ledger  # noqa: E402
from pixel_pn_child_teardown import finish_child  # noqa: E402
from pixel_pn_failure_report import failure_payload  # noqa: E402
from pixel_pn_flight_control_trace import FlightControlTrace  # noqa: E402
from pixel_pn_run_identity import start_identity  # noqa: E402
from pixel_pn_final_approach_speed import settle_final_approach_speed  # noqa: E402


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
    # Cruise-to-gate at one speed, fly the scored leg at another. Both default
    # to off, which leaves the flight at whatever speed SITL was launched with.
    parser.add_argument("--del-speedup", type=float, default=0.0)
    # Where to step the speed down. It has to be a sequence BEFORE the
    # scoring interval one: at 20x the aircraft covers hundreds of metres per wall
    # second, so settling the clock at the scoring interval point would eat most of
    # the scored leg. Stepping down on the gate leg costs only cruise distance.
    parser.add_argument("--slow-seq", type=int, default=0)
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


def _wait_for_scoring_interval(
    vehicle: IVehicle,
    scoring_start_seq: int,
    timeout_s: float,
    purpose: str = "scoring window",
) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.is_armed and (
            vehicle.mission_items_next is not None
            and vehicle.mission_items_next >= scoring_start_seq
        ):
            return
        time.sleep(0.02)
    raise TimeoutError(
        f"mission did not reach {purpose} sequence {scoring_start_seq}"
    )


def _step_to_final_approach_speed(
    vehicle: IVehicle, options: argparse.Namespace
) -> None:
    """Cruise fast, then fly the scored leg at the speed the run claims."""
    if options.del_speedup <= 0.0 or options.slow_seq <= 0:
        return
    _wait_for_scoring_interval(
        vehicle, options.slow_seq, options.timeout, purpose="speed step-down"
    )
    change = settle_final_approach_speed(vehicle, options.del_speedup)
    print(f"DIRECT_PIXEL_SPEED {change.describe()}", flush=True)
    if not change.settled:
        # Refused here rather than scored later: a scored leg flown at cruise
        # speed is indistinguishable from a normal run until the freshness
        # gate rejects it minutes afterwards.
        raise RuntimeError(f"final-approach speed did not settle: {change.describe()}")


def _wait_for_mode(vehicle: IVehicle, mode: FlightMode, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if vehicle.get_mode is mode:
            return
        time.sleep(0.02)
    raise TimeoutError(f"vehicle did not enter {mode.value}")


def _result_payload(
    passed: bool, snap: ClosestSnap, source: DirectPoiPixelSource
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
            "projected_frames": metrics.projected_frames,
            "projection_failures": metrics.projection_failures,
            "delivered_frames": metrics.delivered_frames,
            "delivery_rejections": metrics.delivery_rejections,
            "delivery_exceptions": metrics.delivery_exceptions,
            "advanced_truth_frames": metrics.advanced_truth_frames,
            "association_refusals": metrics.association_refusals,
            "pose_skew_ms_mean": metrics.pose_skew_ms_mean,
            "pose_skew_ms_max": metrics.pose_skew_ms_max,
            "pose_skew_delta_ms_std": metrics.pose_skew_delta_ms_std,
            "advance": source.advance_diagnostics,
        },
    }


def run(options: argparse.Namespace) -> dict[str, object]:
    # First, while nothing is built: an interrupt here has nothing to close.
    identity = start_identity(options, Path(__file__))
    args = _navpy_args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = None
    cadence = None
    navigation = None
    source = None
    admission = None
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

        source = DirectPoiPixelSource(
            vehicle,
            poi,
            cadence,
            aircraft_sequence=geo_ref.uas_seq,
            aircraft_degrees=geo_ref.degrees,
            deliver=deliver,
        )
        navigation.bind_final_approach_source_dispatch(
            source.dispatch_available,
            # Record-only. None when tracing is off, and the worker holds
            # a no-op observer in that case rather than a branch.
            command_loop_observer(source.determinism_trace),
        )
        # Before start(), so the ledger holds every ruling the source can see.
        admission = subscribe_admission_ledger(
            vehicle, source.determinism_trace
        )
        source.start()
        navigation.start()
        print("DIRECT_PIXEL_READY", flush=True)
        _step_to_final_approach_speed(vehicle, options)
        _wait_for_scoring_interval(vehicle, options.scoring_start_seq, options.timeout)
        if not vehicle.set_mode(FlightMode.GUIDED):
            raise RuntimeError("GUIDED mode request was rejected")
        _wait_for_mode(vehicle, FlightMode.GUIDED, 5.0)
        navigation.init()
        source.activate()
        options.scoring_active.write_text("DIRECT_PIXEL_NAV\n", encoding="utf-8")
        print("DIRECT_PIXEL_NAV", flush=True)
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
        # Every step attempted whatever the others did, the determinism
        # evidence written after all of them, and only then anything raised.
        finish_child(
            options.result.parent,
            flight_trace=trace,
            source=source,
            navigation=navigation,
            cadence=cadence,
            vehicle=vehicle,
            admission=admission,
            logger=logger,
            identity=identity,
        )


def publish_result(path: Path, payload: dict) -> None:
    """Write-then-replace, because the file's EXISTENCE is the signal.

    The evaluators poll for this path and parse it the moment it appears; a
    bare write_text can be observed half-written, which reads as a corrupt
    result from a healthy flight.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.parent / (path.name + ".tmp")
    staging.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(staging, path)


def main() -> int:
    options = _parser().parse_args()
    try:
        payload = run(options)
    except BaseException as error:
        # Built so that it cannot raise: this is the one path that must always
        # publish, and once the summary is lost the failure is all there is.
        payload = failure_payload(error)
    publish_result(options.result, payload)
    print("DIRECT_PIXEL_RESULT " + json.dumps(payload), flush=True)
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
