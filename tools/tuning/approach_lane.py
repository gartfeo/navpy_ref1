"""The approach run's search lane (hitl_final_approach.md section 5, the dock location rule).

The lane is fixed and never derived from a dock: from the autopilot's home it goes LANE_START_M along LANE_BEARING_DEG,
then back and forth over LANE_LENGTH_M at LANE_HEIGHT_M above home, repeated with DO_JUMP, then loiters. navpy's
detection starts at the lane's first waypoint (-mwp). All values are my choices for stage 1, not plan values.
"""
from __future__ import annotations

from dataclasses import dataclass

from tools.tuning.dock_pass import (HOME_SEQ, M, PASS_AIRSPEED_M_S, SPEED_SEQ, Point, _e7, along, upload_mission,
                                    wait_reached)
from tools.tuning.vehicle import Vehicle

LANE_BEARING_DEG = 0.0          # north from home
LANE_START_M = 400.0            # past the climb-out
LANE_LENGTH_M = 2000.0
LANE_HEIGHT_M = 80.0            # above home
LANE_REPEATS = 30               # DO_JUMP repeats of the out-and-back: about 3 min each at cruise, more than any run
FIRST_LANE_SEQ = SPEED_SEQ + 1
FIRST_LEG_TIMEOUT_S = 180.0     # from the takeoff to the lane's first waypoint (wait_reached)


@dataclass(frozen=True)
class Lane:
    home: Point
    start: Point
    end: Point

    def describe(self) -> dict:
        return {"bearing_deg": LANE_BEARING_DEG, "start_m": LANE_START_M, "length_m": LANE_LENGTH_M,
                "height_m": LANE_HEIGHT_M, "repeats": LANE_REPEATS, "airspeed_m_s": PASS_AIRSPEED_M_S,
                "first_seq": FIRST_LANE_SEQ,
                "start": {"lat": self.start.lat, "lon": self.start.lon},
                "end": {"lat": self.end.lat, "lon": self.end.lon}}


def build_lane(home: Point) -> Lane:
    start = along(home, LANE_BEARING_DEG, LANE_START_M)
    return Lane(home, start, along(start, LANE_BEARING_DEG, LANE_LENGTH_M))


def lane_items(lane: Lane) -> list[tuple]:
    """home, speed, the lane's start and end, DO_JUMP back to the start, then a loiter at the start."""
    rel = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    start, end = FIRST_LANE_SEQ, FIRST_LANE_SEQ + 1
    return [(HOME_SEQ, M.MAV_FRAME_GLOBAL_INT, M.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, _e7(lane.home.lat),
             _e7(lane.home.lon), 0.0),
            (SPEED_SEQ, rel, M.MAV_CMD_DO_CHANGE_SPEED, 0, PASS_AIRSPEED_M_S, -1, 0, 0, 0, 0.0),
            (start, rel, M.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, _e7(lane.start.lat), _e7(lane.start.lon), LANE_HEIGHT_M),
            (end, rel, M.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, _e7(lane.end.lat), _e7(lane.end.lon), LANE_HEIGHT_M),
            (end + 1, rel, M.MAV_CMD_DO_JUMP, start, LANE_REPEATS, 0, 0, 0, 0, 0.0),
            (end + 2, rel, M.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, _e7(lane.start.lat), _e7(lane.start.lon),
             LANE_HEIGHT_M)]


def fly_lane(vehicle: Vehicle, lane: Lane) -> None:
    """Upload the lane, AUTO, and wait for its first waypoint: from there on navpy may detect."""
    upload_mission(vehicle, lane_items(lane))
    vehicle.set_mode("AUTO")
    wait_reached(vehicle, FIRST_LANE_SEQ, FIRST_LEG_TIMEOUT_S)


__all__ = ["FIRST_LANE_SEQ", "LANE_HEIGHT_M", "Lane", "build_lane", "fly_lane", "lane_items"]
