"""SCRATCH DIAGNOSTIC (Step-0, Rule-1 isolation) -- DO NOT COMMIT, DELETE AFTER USE.

Direct GEO POI baseline: same mission/geometry/measurement as
direct_pixel_pn_child.py but navigation flies the legacy 'pn' controller on the
truth geo POI (-udt true).  Purpose: isolate whether the airframe +
ArduPilot attitude stack + a full-state law reach small CPA in each wind
condition.  The legacy path may use full state (GPS, yaw, altitude); it is the
isolation baseline, not a constraint-compliant law.

Measurement parity with direct_pixel_pn_child.py (per review):
- terminates at the FIRST visual pass, using the real VisualPassDetector state
  machine (armed forward frame, aft frames -> suppress, latch passed) driven by
  the same projected body ray the vision path uses; scores exactly one approach
- same result payload, same source metrics, same trace
Differences (justified):
- -udt true, --pitch-controller pn (the variable under test)
- pass detection runs in this child instead of inside the vision runtime,
  because the legacy runtime has no final-approach service (final_approach is None)

WHAT AN A/B AGAINST direct_pixel_pn_child.py CANNOT SHOW.  The two children
differ in at least FOUR variables at once, so no single-variable attribution is
available from the comparison:

1. truth POI on/off -- `-udt true` here (line 87-88) vs `-udt false` in
   direct_pixel_pn_child.py:61-62
2. law -- `--pitch-controller pn` here (line 89-90) vs `vision-nav-pn` in
   direct_pixel_pn_child.py:63-64
3. state privileges -- the legacy path consumes truth POI location and
   vehicle location (src/navpy/modules/navigation/legacy_destination_resolver.py:175-182)
4. final-approach runtime -- the vision path runs the final-approach service; the legacy
   path has none (see the pass-detection note above)

A sub-metre result from this child therefore proves ONE thing: the airframe,
the ArduPilot attitude stack, and the mission geometry can fly that wind
condition to sub-metre CPA.  It does NOT isolate the vision law's structure
from the vision input path or the final-approach runtime, and must not be reported as
if it did.  Isolating the law requires geo-derived LOS fed into the SAME vision
law with the other three variables held fixed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

WORKTREE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKTREE / "src"))

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
from navpy.modules.navigation.nav.vision_nav.visual_pass import (  # noqa: E402
    VisualPassDetector,
)
from navpy.modules.vehicle.flight_mode import FlightMode  # noqa: E402
from navpy.modules.vehicle.vehicle_factory import create_vehicle  # noqa: E402
from navpy.modules.vehicle.vehicle_interface import IVehicle  # noqa: E402
from navpy.modules.vision.models.detect_data import DetectedObject  # noqa: E402
from navpy.modules.vision.sim.direct_poi_pixel_source import (  # noqa: E402
    DirectPoiPixelSource,
)
from navpy.modules.vision.visual_ray_projection import (  # noqa: E402
    observation_body_ray,
)
from pixel_pn_flight_control_trace import FlightControlTrace  # noqa: E402


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
    return make_parser(description="scratch direct geo child").parse_args([
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
        "true",
        "--pitch-controller",
        "pn",
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
    vehicle: IVehicle, scoring_start_seq: int, timeout_s: float
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
        f"mission did not reach scoring start sequence {scoring_start_seq}"
    )


def _wait_for_mode(
    vehicle: IVehicle, mode: FlightMode, timeout_s: float
) -> None:
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
        },
    }


class _PassGate:
    """First-pass gate mirroring the vision runtime's VisualPassDetector use.

    Feeds the REAL VisualPassDetector state machine with a duck-typed frame
    carrying the same body-ray forward component the vision path uses.  The
    detector's plan() reads only continuity_key and body_x.
    """

    def __init__(self) -> None:
        self._detector = VisualPassDetector()
        self._lock = threading.Lock()
        self._failure: BaseException | None = None

    def observe(self, detection: DetectedObject) -> bool:
        """Return True when commands must be suppressed (aft frame seen)."""
        try:
            # DetectedObject carries the pixel observation in .pixel and the
            # identity in .identity (see detect_data.py visual_detection()).
            ray = observation_body_ray(detection.pixel)
            identity = detection.identity
            frame = SimpleNamespace(
                continuity_key=(
                    "scratch_geo", 0, identity.task_id, identity.obj_id,
                ),
                body_x=float(ray[0]),
            )
        except BaseException as error:  # fail fast, never a silent timeout
            with self._lock:
                self._failure = error
            raise
        with self._lock:
            plan = self._detector.plan(frame)
            self._detector.commit(plan)
            return plan.suppress_command

    def raise_if_failed(self) -> None:
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(
                    f"pass gate failed: {self._failure!r}"
                ) from self._failure

    @property
    def passed(self) -> bool:
        with self._lock:
            return self._detector.passed


def run(options: argparse.Namespace) -> dict[str, object]:
    args = _navpy_args(options)
    logger = initialize_logger(LoggerArgs(args), options.sysid)
    vehicle = None
    cadence = None
    navigation = None
    source = None
    trace = FlightControlTrace(options.result.parent / "flight_control_trace.csv")
    pass_gate = _PassGate()
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
            # Legacy mode has no vision-nav confirm gate (final_approach is
            # None).  Mirror the vision runtime's pass handling: aft frames
            # suppress commands; the nav loop exits once passed latches.
            if pass_gate.observe(poi_detection):
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
        navigation.bind_final_approach_source_dispatch(source.dispatch_available)
        source.start()
        navigation.start()
        print("DIRECT_PIXEL_READY", flush=True)
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
            pass_gate.raise_if_failed()
            trace.sample(vehicle)
            if pass_gate.passed:
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
    print("DIRECT_PIXEL_RESULT " + json.dumps(payload), flush=True)
    return 0 if payload.get("passed") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
