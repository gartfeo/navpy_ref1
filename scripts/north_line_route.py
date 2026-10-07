"""Geometry of the straight north-line mission: constants, loader, checks.

Split out of ``scripts/upload_north_line_mission.py`` (which owns transfer and
verification) so each module stays within the SOLID size limits; the uploader
re-exports everything here, so existing importers keep one entry point.  See
that module's docstring for the full route rationale.
"""

from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_LOITER_TO_ALT,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
)
from pymavlink.mavwp import MAVWPLoader

METRES_PER_DEG_LAT = 111320.0
# Open water in the central Black Sea (Euxine abyssal plain), ~120 km from the
# nearest coast and clear of the sea's three small islands, which all sit near
# shore.  Terrain is the reason: over land the original site climbed 207 m in
# 5 km, so a level mission leg flew into a hillside and no vertical result meant
# anything.  Sea is exactly flat and, unlike a high salt flat such as Salar de
# Uyuni, is at sea-level air density, so lift, stall speed and turn rate stay
# where the aircraft model expects them.
DEFAULT_HOME = (43.0, 34.0)
DEFAULT_ALT_M = 400.0
TAKEOFF_PITCH_DEG = 15.0
# North offsets, in metres from home, for the takeoff and the single waypoint.
# Home sits south of both, so the whole route is one straight northward line
# with no turn-around: the aircraft climbs out on the same heading it will hold
# through the final-approach leg, and the L1 capture transient never has a corner to
# recover from.
#
# The takeoff point's latitude does not steer anything: ArduPlane ends
# NAV_TAKEOFF on altitude alone and ignores the item's lat/lon.  Measured in
# SITL at 400 m: MISSION_CURRENT advanced to the waypoint at ~1240 m north
# while the takeoff item sat at 2500 m.  It is kept only so the plan reads as
# a climb-out followed by a leg.
DEFAULT_TAKEOFF_OFFSET_M = 1000.0
# This is the only offset that sets the scored leg.  The companion takes over
# when MISSION_CURRENT reaches the waypoint's sequence, which happens where the
# climb ends -- measured at ~1240 m north for a 400 m climb -- so the leg is
# this offset minus that, not this offset minus the takeoff offset.  2150 m
# leaves an scored leg just under a kilometre.
CLIMB_COMPLETE_OFFSET_M = 1240.0
# Where the aircraft circles until it has both altitude and the outbound
# heading. Its distance does not need to clear the climb -- the loiter waits
# however long the climb takes -- so this is only "far enough out to be tidy".
DEFAULT_LOITER_OFFSET_M = 1000.0
DEFAULT_LOITER_RADIUS_M = 150.0
# The final approach starts when the aircraft reaches this gate, so the scored leg is
# waypoint minus gate: a fixed distance in every wind condition.
DEFAULT_GATE_OFFSET_M = 1800.0
DEFAULT_WAYPOINT_OFFSET_M = 2700.0


def build_loader(
    home: tuple[float, float],
    takeoff_offset_m: float,
    waypoint_offset_m: float,
    alt_m: float,
    waypoint_alt_m: float,
    sysid: int,
    loiter_offset_m: float = DEFAULT_LOITER_OFFSET_M,
    gate_offset_m: float = DEFAULT_GATE_OFFSET_M,
    loiter_radius_m: float = DEFAULT_LOITER_RADIUS_M,
) -> MAVWPLoader:
    """Home, takeoff, loiter-to-alt, a final-approach start gate, then the POI.

    The loiter and the gate exist to make the final-approach start repeatable.  NAV_TAKEOFF
    ends on altitude alone, so with a bare takeoff the ground position where
    the mission advances moves with the wind -- measured 576 m out in a 10 m/s
    tailwind against 1377 m in the same headwind, which changes the scoring interval
    geometry far more than the wind changes the navigation.

    NAV_LOITER_TO_ALT fixes that structurally rather than by tuning a distance:
    ArduPlane will not leave it until the POI altitude is reached AND the
    aircraft is lined up with the next waypoint (verify_loiter_to_alt ->
    verify_loiter_heading).  No wind speed can outrun it, whereas a gate chosen
    to clear one measured climb silently fails at a stronger wind.

    The gate then pins the final-approach start *position*: the loiter exit still varies by
    about the loiter radius, and the companion starts navigation when MISSION_CURRENT
    reaches the POI's sequence, which happens when the aircraft *reaches*
    the gate -- a fixed latitude.  The gate leg also gives the loiter exit
    transient somewhere to settle before the measured leg begins.
    """
    lat, lon = home
    loader = MAVWPLoader(sysid, 0)
    rows = [
        (lat, lon, 0.0, MAV_CMD_NAV_WAYPOINT),
        (
            lat + takeoff_offset_m / METRES_PER_DEG_LAT,
            lon,
            alt_m,
            MAV_CMD_NAV_TAKEOFF,
        ),
        (
            lat + loiter_offset_m / METRES_PER_DEG_LAT,
            lon,
            alt_m,
            MAV_CMD_NAV_LOITER_TO_ALT,
        ),
        (
            lat + gate_offset_m / METRES_PER_DEG_LAT,
            lon,
            alt_m,
            MAV_CMD_NAV_WAYPOINT,
        ),
        (
            lat + waypoint_offset_m / METRES_PER_DEG_LAT,
            lon,
            waypoint_alt_m,
            MAV_CMD_NAV_WAYPOINT,
        ),
    ]
    for sequence, (row_lat, row_lon, row_alt, command) in enumerate(rows):
        loader.add_latlonalt(row_lat, row_lon, row_alt)
        waypoint = loader.wp(sequence)
        waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        waypoint.command = command
        waypoint.autocontinue = 1
        waypoint.current = 1 if sequence == 1 else 0
        if command == MAV_CMD_NAV_TAKEOFF:
            waypoint.param1 = TAKEOFF_PITCH_DEG
        if command == MAV_CMD_NAV_LOITER_TO_ALT:
            # Only the radius is carried: ArduPilot's parser keeps param2 as
            # cmd.p1 and drops param1, because verify_loiter_to_alt always runs
            # verify_loiter_heading.  The heading gate is unconditional, so
            # setting a "heading required" flag here would only look meaningful.
            waypoint.param2 = loiter_radius_m
        # add_latlonalt stores degrees; mission_item_int_send needs 1e7 ints.
        waypoint.x = int(round(row_lat * 1e7))
        waypoint.y = int(round(row_lon * 1e7))
    return loader


def describe_item(command: int, sequence: int) -> str:
    if sequence == 0:
        return "HOME"
    return {
        MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
        MAV_CMD_NAV_LOITER_TO_ALT: "LOITER_ALT",
    }.get(command, "WAYPOINT")


def check_offsets(
    takeoff_offset: float,
    waypoint_offset: float,
    loiter_offset: float = DEFAULT_LOITER_OFFSET_M,
    gate_offset: float = DEFAULT_GATE_OFFSET_M,
) -> None:
    """Raise ValueError unless the route runs strictly northward.

    Every navigated point has to increase in latitude.  Checking the POI
    against the takeoff offset alone is not enough: the takeoff item's position
    is ignored by ArduPlane, while the loiter and the gate are real navigated
    points, so a POI south of the gate reverses the final leg while passing
    a takeoff-only check.

    There is deliberately no climb-completion check any more.  It existed to
    keep the POI clear of an altitude-triggered takeoff, and
    NAV_LOITER_TO_ALT now guarantees the climb has finished before the aircraft
    proceeds -- a structural guarantee that holds at any wind speed, unlike a
    distance measured at one.
    """
    ordered = (
        ("loiter", loiter_offset),
        ("gate", gate_offset),
        ("POI", waypoint_offset),
    )
    for (before_name, before), (after_name, after) in zip(ordered, ordered[1:]):
        if after <= before:
            raise ValueError(
                f"{after_name} offset {after:g} m must be north of the "
                f"{before_name} offset {before:g} m, or the route doubles back "
                "and the ground track is no longer northward"
            )
    if loiter_offset < takeoff_offset:
        raise ValueError(
            f"loiter offset {loiter_offset:g} m is south of the takeoff offset "
            f"{takeoff_offset:g} m"
        )
