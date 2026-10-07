"""Upload a straight north-line mission for wind-relative navigation tests.

Every waypoint sits on one meridian, so the *intended* inbound ground track is
due north and SIM_WIND_DIR lines up with head (0), tail (180), and cross
(90/270).  Intended, not guaranteed: ArduPlane ignores the NAV_TAKEOFF item's
lat/lon and establishes climb-out course from the aircraft's initial heading,
and crosswind pushes the aircraft sideways during the climb, so the leg can
start off the meridian.  Score wind against the measured ground track
(``eval_ground_track.GroundTrackRecorder``), never against an assumed 000.
A collinear route makes the track close to north and easy to reason about; it
does not make it exact.

The route is HOME, NAV_TAKEOFF, NAV_LOITER_TO_ALT, a final-approach start
gate, then the POI.  Two NAV_WAYPOINTs, so the evaluator wants ``--scoring-start-wp 2
--poi-wp 2``: ordinal 1 is the gate and ordinal 2 is the POI
(NAV_LOITER_TO_ALT is not a NAV_WAYPOINT and is not counted).  Naming ordinal 1
would score the gate as the POI and start the final approach at the loiter exit.

The loiter and the gate both exist to make the final-approach start repeatable.  NAV_TAKEOFF
ends on altitude alone, so a bare takeoff advances the mission wherever the
climb happens to finish -- measured 576 m from the POI in a 10 m/s tailwind
against 1377 m in the same headwind, which moves the scoring interval geometry far
more than the wind moves the navigation.  NAV_LOITER_TO_ALT holds the aircraft
until it has both the altitude and the outbound heading, at any wind speed, and
the gate then fixes the final-approach start *position*: the child starts navigation when
MISSION_CURRENT reaches the POI's sequence, which happens when the aircraft
reaches the gate.  Scored leg = POI offset - gate offset, constant.

The waypoint altitude defaults to the cruise altitude, so the run-in is level.
The descent onto the POI belongs to the navigation law, not to the mission --
a mission that descends would supply vertical navigation the law is supposed to
produce itself.  ``--waypoint-alt`` exists to make the mission descend anyway,
which changes what the run measures.  The POI's own altitude comes from the
evaluator's ``--poi-alt``, never from the mission.

Transfer runs through ``VehicleMav.upload_mission`` rather than a local
MISSION_COUNT loop, so this shares the project's one tested mission state
machine (retries, fresh-response matching, mission_type) instead of a second
copy that can drift from it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable
from pathlib import Path

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_MISSIONPLANNER,
)

WORKTREE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE)]

from gcs.backend import instance_ports as ip  # noqa: E402
from navpy.modules.vehicle.vehicle_mav import VehicleMav  # noqa: E402
from scripts.north_line_route import (  # noqa: E402,F401
    CLIMB_COMPLETE_OFFSET_M,
    DEFAULT_ALT_M,
    DEFAULT_GATE_OFFSET_M,
    DEFAULT_HOME,
    DEFAULT_LOITER_OFFSET_M,
    DEFAULT_LOITER_RADIUS_M,
    DEFAULT_TAKEOFF_OFFSET_M,
    DEFAULT_WAYPOINT_OFFSET_M,
    METRES_PER_DEG_LAT,
    TAKEOFF_PITCH_DEG,
    build_loader,
    check_offsets,
    describe_item,
)

# Home is pinned at SITL launch and the vehicle reports it back to the metre,
# so any real disagreement means a wrong launch, not measurement noise. The
# allowance covers GPS/rounding jitter only.
MAX_HOME_DRIFT_M = 25.0
# SITL reports home as 0,0 until it has a GPS fix; this covers that wait,
# not a slow link.
HOME_WAIT_TIMEOUT_S = 120.0


def _wait_for_home(
    vehicle: VehicleMav,
    echo: Callable[[str], None],
    timeout_s: float = HOME_WAIT_TIMEOUT_S,
) -> None:
    """Block until the vehicle has established home.

    A freshly launched SITL reports home as 0,0 until it has a GPS fix, and
    ArduPilot synthesises mission item 0 from that.  Checking home against the
    requested location before then compares against a placeholder and reports a
    5500 km error on a perfectly good vehicle.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        home = vehicle.home_location
        if home is not None and (home.lat, home.lng) != (0.0, 0.0):
            return
        time.sleep(0.5)
    raise RuntimeError(
        f"vehicle never established home within {timeout_s:g}s; it reports "
        "0,0, which means it has no GPS fix yet"
    )


def upload_north_line(
    device: str,
    sysid: int,
    *,
    home: tuple[float, float] = DEFAULT_HOME,
    takeoff_offset: float = DEFAULT_TAKEOFF_OFFSET_M,
    waypoint_offset: float = DEFAULT_WAYPOINT_OFFSET_M,
    loiter_offset: float = DEFAULT_LOITER_OFFSET_M,
    gate_offset: float = DEFAULT_GATE_OFFSET_M,
    alt_m: float = DEFAULT_ALT_M,
    waypoint_alt_m: float | None = None,
    echo: Callable[[str], None] = print,
) -> int:
    """Upload the line to ``sysid`` and verify it by reading it back.

    Raises ValueError for an unusable route and RuntimeError if the vehicle
    does not end up holding what was sent.  Shared with the evaluator so a
    swept run flies the same geometry this script uploads by hand.
    """
    check_offsets(takeoff_offset, waypoint_offset, loiter_offset, gate_offset)
    loader = build_loader(
        home,
        takeoff_offset,
        waypoint_offset,
        alt_m,
        alt_m if waypoint_alt_m is None else waypoint_alt_m,
        sysid,
        loiter_offset,
        gate_offset,
    )
    # skip_mission_download: the stored mission is about to be replaced, so
    # paying for a download of the mission we are discarding buys nothing.
    #
    # mav_comp_id: VehicleMav sends as source_system == target_system, so the
    # default onboard-computer component would make us byte-identical to the
    # running NavPy companion.  The router then delivers the autopilot's
    # MISSION_REQUEST to the companion's link instead of ours and the upload
    # stalls at "autopilot ignores MISSION_COUNT".  A distinct component id
    # keeps our replies routed here.
    vehicle = VehicleMav(
        device,
        sysid,
        skip_mission_download=True,
        mav_comp_id=MAV_COMP_ID_MISSIONPLANNER,
    )
    try:
        _wait_for_home(vehicle, echo)
        vehicle.load_mission_items(loader)
        if not vehicle.upload_mission():
            raise RuntimeError("mission upload failed")
        # Read back from the vehicle: the ACK says it was received, only a
        # download says what it actually stored.
        count = vehicle.download_mission()
        if count != loader.count():
            raise RuntimeError(
                f"read-back count {count} != uploaded {loader.count()}"
            )
        # Mission item 0 is not the item we uploaded: ArduPilot synthesises it
        # from AHRS home, so this reads back where the vehicle actually is
        # homed.  Home is fixed when SITL launches and no upload can move it,
        # which makes this the one place a run can catch a vehicle launched at
        # the wrong place -- silently flying the right shape over the wrong
        # ground is the failure this guards against.
        home_item = vehicle.get_mission_item(0)
        drift_m = math.hypot(
            (home_item.x / 1e7 - home[0]) * METRES_PER_DEG_LAT,
            (home_item.y / 1e7 - home[1])
            * METRES_PER_DEG_LAT
            * math.cos(math.radians(home[0])),
        )
        if drift_m > MAX_HOME_DRIFT_M:
            raise RuntimeError(
                f"vehicle home {home_item.x / 1e7:.7f},{home_item.y / 1e7:.7f} "
                f"is {drift_m:.0f} m from the requested "
                f"{home[0]:.7f},{home[1]:.7f}. Home is set when SITL launches, "
                f"not by this upload: relaunch with --home {home[0]},{home[1]},0,0"
            )
        echo(f"verified {count} items read back from the vehicle:")
        for sequence in range(count):
            item = vehicle.get_mission_item(sequence)
            echo(
                f"  seq {sequence} {describe_item(item.command, sequence):<8} "
                f"{item.x / 1e7:.7f} {item.y / 1e7:.7f} alt {item.z:g}"
            )
        return count
    finally:
        vehicle.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chat", type=int, required=True)
    parser.add_argument("--sysid", type=int, required=True)
    parser.add_argument("--home-lat", type=float, default=DEFAULT_HOME[0])
    parser.add_argument("--home-lon", type=float, default=DEFAULT_HOME[1])
    parser.add_argument("--alt", type=float, default=DEFAULT_ALT_M)
    parser.add_argument(
        "--waypoint-alt",
        type=float,
        default=None,
        help=(
            "relative altitude at the waypoint (default: same as --alt, so the "
            "run-in is level). Set it lower to make the mission itself descend; "
            "the final-approach descent toward the POI is navigation's job, not the "
            "mission's, so lowering this changes what the experiment measures."
        ),
    )
    parser.add_argument(
        "--takeoff-offset",
        type=float,
        default=DEFAULT_TAKEOFF_OFFSET_M,
        help="metres north of home where the climb-out ends",
    )
    parser.add_argument(
        "--waypoint-offset",
        type=float,
        default=DEFAULT_WAYPOINT_OFFSET_M,
        help="metres north of home for the single waypoint",
    )
    parser.add_argument(
        "--device",
        default=None,
        help=(
            "MAVLink connection string; defaults to this chat's Mission Planner "
            "UDP port. Override when that port is already held by another GCS."
        ),
    )
    args = parser.parse_args()

    device = args.device or f"udp:0.0.0.0:{ip.mission_planner_port(args.chat)}"
    print(f"connecting {device} sysid={args.sysid}")
    try:
        upload_north_line(
            device,
            args.sysid,
            home=(args.home_lat, args.home_lon),
            takeoff_offset=args.takeoff_offset,
            waypoint_offset=args.waypoint_offset,
            alt_m=args.alt,
            waypoint_alt_m=args.waypoint_alt,
        )
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
