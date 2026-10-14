"""dock-pass: straight AUTO passes over the Unreal dock's ChArUco board, so the HITL camera stream and navpy's
detector are flown past the board at several heights and from several directions (sitl_driver dock-pass).

A test harness, not a mission module: the board's known position (Unreal scene data) shapes only this card's
flight path. navpy's navigation and mission modules are not involved and get no dock information; the detector
sees only the camera's pixels (tools/unreal/npyc_live_check.py), and the scoring against the truth is offline
(tools/unreal/dock_pass_report.py).

Each pass is three waypoints on a line through the board along the pass heading: the approach point, where the
turn onto the line happens; the entry point, from which the line is flown settled (the pass window opens when it is
reached); and the exit point past the board, where the window closes. The camera is fixed to the body, looking
forward and down, so the board leaves the picture as the aircraft flies over it; the exit point only ends the line.
Heights are above the board; the mission flies them as heights above home (board MSL - home MSL + height).
After the last pass the aircraft flies RTL.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil

from tools.tuning.flight_steps import MESSAGE_TIMEOUT_S, FlightCheckFailed
from tools.tuning.vehicle import MODE, Vehicle

M = mavutil.mavlink
# The ChArUco board on the Yeghvard dock (Unreal log "NavPy ChArUco board centre", 2026-10-13): lat, lon, and the
# WGS84 ellipsoid height 1320.365 m less the 23.5 m undulation of docs/design/hitl_unreal_camera.md, i.e. MSL as the
# bridge and the board use it. The board lies flat on the deck, 2.4 m east-west by 3.2 m north-south (deck and board
# yaw 0: Unreal +X is east). Relative to the Yeghvard origin (40.3117412, 44.4552112, 1292.8 MSL): 250 m east,
# 35 m north, 4.07 m up.
DOCK_BOARD_YEGHVARD = "40.31205630,44.45815163,1296.865"
# The card: (height above the board in m, pass heading in deg from north), flown in this order. The fixed body camera
# looks 35 deg down with a 59 deg vertical field, so at height h the board is in the picture only from about
# h / tan(61 deg) to h / tan(3 deg) ahead, and ChArUco needs it within about 20 m (bench: 20 of 20 frames at 12-16 m,
# hitl_unreal_camera.md). The first SITL card (40, 25 and 15 m, 2026-10-14) saw the board in 2534 frames and detected
# it once: from 15 m up the board is in the picture only beyond about 17 m. So one pass each at 40, 25 and 15 m for the
# trend, and the low height, 10 m, from all four directions.
PASS_CARD: tuple[tuple[float, float], ...] = ((40.0, 0.0), (25.0, 90.0), (15.0, 180.0), (15.0, 270.0),
                                              (10.0, 0.0), (10.0, 90.0), (10.0, 180.0), (10.0, 270.0))
# Distances along the pass line, from the board. APPROACH_M - ENTRY_M = 450 m (about 19 s at the cruise) lets L1 and
# TECS settle on the line after a turn of up to 180 deg (L1 period about 20 s) before the window opens; from ENTRY_M
# the board is far beyond ChArUco range (tens of metres) at every height, so no detection falls before the window.
APPROACH_M = 900.0
ENTRY_M = 450.0
EXIT_M = 150.0
PASS_AIRSPEED_M_S = 24.0          # the aircraft's cruise; DO_CHANGE_SPEED at the mission's start
# The run's floor during the passes, above home. The lowest pass flies about 13 m above home (10 m above a board
# 4.07 m above the origin, less the rail: the board's home was 1.1 m above the origin, 2026-10-14); TECS held the SITL
# passes within a few metres. FLOOR_MARGIN_M is the least gap accepted between the floor and the lowest pass, so a
# floor that the passes themselves would trip is refused: before the flight against the origin, after the takeoff
# against the autopilot's home.
DOCK_PASS_MIN_HEIGHT_M = 7.0
FLOOR_MARGIN_M = 5.0
# A board further than this from the origin is a wrong --dock or --origin (Yeghvard: 252 m).
MAX_DOCK_DISTANCE_M = 3000.0
# Waiting for a waypoint: the leg at a ground speed no aircraft of this card flies below (AIRSPEED_MIN-like), plus a
# turn of up to 360 deg at the cruise; past this the run fails instead of flying on.
MIN_GROUND_SPEED_M_S = 12.0
TURN_ALLOWANCE_S = 60.0
END_MODE = "RTL"
END_RTL_S = 15.0                  # RTL flown before the run ends, to see it taken and climbing
MISSION_ITEM_TIMEOUT_S = 5.0      # the longest wait for the autopilot's next request while uploading
M_PER_DEG_LAT = 111_320.0         # metres per degree of latitude (and of longitude at the equator)
HOME_SEQ = 0                      # ArduPilot's mission item 0 is home; it overwrites it
SPEED_SEQ = 1
HOME_POSITION_ID = 242            # MAVLINK_MSG_ID_HOME_POSITION, asked for with MAV_CMD_REQUEST_MESSAGE


@dataclass(frozen=True)
class Point:
    lat: float
    lon: float


@dataclass(frozen=True)
class Pass:
    height_m: float               # above the board
    heading_deg: float
    alt_rel_home_m: float
    approach: Point
    entry: Point
    exit: Point
    first_seq: int                # the approach point's mission seq; entry and exit follow


@dataclass(frozen=True)
class Card:
    board: Point
    board_msl_m: float
    home: Point
    home_msl_m: float
    passes: tuple[Pass, ...]
    floor_m: float
    airspeed_m_s: float

    def describe(self) -> dict:
        return {"board": {"lat": self.board.lat, "lon": self.board.lon, "msl_m": self.board_msl_m},
                "home": {"lat": self.home.lat, "lon": self.home.lon, "msl_m": self.home_msl_m},
                "floor_m": self.floor_m, "airspeed_m_s": self.airspeed_m_s,
                "approach_m": APPROACH_M, "entry_m": ENTRY_M, "exit_m": EXIT_M,
                "passes": [{"height_m": p.height_m, "heading_deg": p.heading_deg, "alt_rel_home_m": p.alt_rel_home_m,
                            "seq": {"approach": p.first_seq, "entry": p.first_seq + 1, "exit": p.first_seq + 2}}
                           for p in self.passes]}


def offset(point: Point, north_m: float, east_m: float) -> Point:
    """point moved by north_m, east_m on a local flat earth (metres over a few kilometres)."""
    return Point(point.lat + north_m / M_PER_DEG_LAT,
                 point.lon + east_m / (M_PER_DEG_LAT * math.cos(math.radians(point.lat))))


def distance_m(a: Point, b: Point) -> float:
    north = (b.lat - a.lat) * M_PER_DEG_LAT
    east = (b.lon - a.lon) * M_PER_DEG_LAT * math.cos(math.radians(a.lat))
    return math.hypot(north, east)


def along(board: Point, heading_deg: float, metres: float) -> Point:
    """The point metres along the pass heading from the board (negative: before it)."""
    rad = math.radians(heading_deg)
    return offset(board, metres * math.cos(rad), metres * math.sin(rad))


def parse_triplet(text: str, name: str) -> tuple[float, float, float]:
    parts = [float(v) for v in text.split(",")]
    if len(parts) < 3:
        raise ValueError(f"{name} needs LAT,LON,ALT_MSL: {text!r}")
    return parts[0], parts[1], parts[2]


def build_card(dock: str, origin: str, floor_m: float, card: tuple[tuple[float, float], ...] = PASS_CARD,
               airspeed_m_s: float = PASS_AIRSPEED_M_S) -> Card:
    """The card for the board at dock (LAT,LON,ALT_MSL) flown from origin (LAT,LON,ALT_MSL[,HDG], the run's home).
    ValueError when the board is implausibly far from home or floor_m would be tripped by the lowest pass."""
    if not math.isfinite(floor_m):
        raise ValueError(f"--min-height must be a number, not {floor_m}")
    blat, blon, bmsl = parse_triplet(dock, "--dock")
    hlat, hlon, hmsl = parse_triplet(origin, "--origin")
    board, home = Point(blat, blon), Point(hlat, hlon)
    if distance_m(home, board) > MAX_DOCK_DISTANCE_M:
        raise ValueError(f"the board is {distance_m(home, board):.0f} m from the origin, more than "
                         f"{MAX_DOCK_DISTANCE_M:g} m: check --dock and --origin")
    passes, seq = [], SPEED_SEQ + 1
    for height, heading in card:
        passes.append(Pass(height, heading, round(bmsl - hmsl + height, 2), along(board, heading, -APPROACH_M),
                           along(board, heading, -ENTRY_M), along(board, heading, EXIT_M), seq))
        seq += 3
    lowest = min(p.alt_rel_home_m for p in passes)
    if floor_m > lowest - FLOOR_MARGIN_M:
        raise ValueError(f"--min-height {floor_m:g} m is within {FLOOR_MARGIN_M:g} m of the lowest pass "
                         f"({lowest:g} m above home)")
    return Card(board, bmsl, home, hmsl, tuple(passes), floor_m, airspeed_m_s)


def mission_items(card: Card) -> list[tuple]:
    """(seq, frame, command, param1-4, lat_e7, lon_e7, alt) per mission item: home, the speed, then each pass's
    approach, entry and exit waypoints at the pass's height above home, then a loiter at the last exit."""
    rel = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    items = [(HOME_SEQ, M.MAV_FRAME_GLOBAL_INT, M.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0,
              _e7(card.home.lat), _e7(card.home.lon), 0.0),
             (SPEED_SEQ, rel, M.MAV_CMD_DO_CHANGE_SPEED, 0, card.airspeed_m_s, -1, 0, 0, 0, 0.0)]
    for p in card.passes:
        for k, point in enumerate((p.approach, p.entry, p.exit)):
            items.append((p.first_seq + k, rel, M.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, _e7(point.lat), _e7(point.lon),
                          p.alt_rel_home_m))
    # A last item after the last exit that keeps AUTO: a mission ending at its last waypoint switches to RTL at once
    # ("Mission complete"), and so does a NAV_RETURN_TO_LAUNCH item (ArduPlane's do_RTL sets the mode); in both SITL
    # cards (2026-10-14) that RTL heartbeat was read before the last MISSION_ITEM_REACHED. fly_passes sets RTL itself.
    last = card.passes[-1]
    items.append((len(items), rel, M.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, _e7(last.exit.lat), _e7(last.exit.lon),
                  last.alt_rel_home_m))
    return items


def _e7(deg: float) -> int:
    return int(round(deg * 1e7))


def upload_mission(vehicle: Vehicle, items: list[tuple]) -> None:
    """The mission protocol: MISSION_COUNT, one MISSION_ITEM_INT per request (MISSION_REQUEST_INT or the older
    MISSION_REQUEST), then the autopilot's MISSION_ACK, which must be MAV_MISSION_ACCEPTED and come after every item was
    sent. Only messages addressed to this link's system id count: Mission Planner on the mirror (mp_mirror.py) shares
    the link, and the autopilot answers its requests to its own id."""
    conn = vehicle.mav
    ours = conn.mav.srcSystem

    def for_us(msg: object) -> bool:
        return getattr(msg, "mission_type", 0) == M.MAV_MISSION_TYPE_MISSION and msg.target_system == ours

    conn.mav.mission_count_send(conn.target_system, conn.target_component, len(items), M.MAV_MISSION_TYPE_MISSION)
    sent: set[int] = set()
    while True:
        msg = vehicle.wait_message(("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"), MISSION_ITEM_TIMEOUT_S,
                                   for_us)
        if msg is None:
            raise FlightCheckFailed("mission upload: no request or answer from the autopilot")
        if msg.get_type() == "MISSION_ACK":
            if msg.type != M.MAV_MISSION_ACCEPTED:
                raise FlightCheckFailed(f"mission upload refused (MAV_MISSION_RESULT {msg.type})")
            if len(sent) != len(items):
                raise FlightCheckFailed(f"mission upload: accepted after {len(sent)} of {len(items)} items")
            return
        if not 0 <= msg.seq < len(items):
            raise FlightCheckFailed(f"mission upload: the autopilot asked for item {msg.seq} of {len(items)}")
        seq, frame, command, p1, p2, p3, p4, x, y, z = items[msg.seq]
        conn.mav.mission_item_int_send(conn.target_system, conn.target_component, seq, frame, command,
                                       0, 1, p1, p2, p3, p4, x, y, z, M.MAV_MISSION_TYPE_MISSION)
        sent.add(seq)


def home_position(vehicle: Vehicle) -> tuple[float, float, float]:
    """The autopilot's home (lat, lon, MSL m) from HOME_POSITION. Heights above home are flown from it, and ArduPlane
    sets it from the GPS at arming: on the rail, so not exactly at the run's --origin."""
    conn = vehicle.mav
    conn.mav.command_long_send(conn.target_system, conn.target_component, M.MAV_CMD_REQUEST_MESSAGE, 0,
                               HOME_POSITION_ID, 0, 0, 0, 0, 0, 0)
    msg = vehicle.wait_message("HOME_POSITION", MESSAGE_TIMEOUT_S)
    if msg is None:
        raise FlightCheckFailed("no HOME_POSITION from the autopilot")
    return msg.latitude * 1e-7, msg.longitude * 1e-7, msg.altitude * 1e-3


def board_time_ms(vehicle: Vehicle) -> int:
    """The autopilot's boot time now (ATTITUDE.time_boot_ms): the clock the camera frames are stamped with."""
    att = vehicle.wait_message("ATTITUDE", MESSAGE_TIMEOUT_S)
    if att is None:
        raise FlightCheckFailed("no ATTITUDE for the pass window's board time")
    return int(att.time_boot_ms)


def wait_reached(vehicle: Vehicle, seq: int, timeout_s: float) -> None:
    """Until MISSION_ITEM_REACHED for mission item seq. FlightCheckFailed when a later item is reported first (the item
    was skipped, e.g. a ground station set the current item), when AUTO is left (a failsafe or the pilot), or after
    timeout_s simulated seconds."""
    deadline = time.time() + timeout_s / max(1, vehicle.speedup)
    while (left := deadline - time.time()) > 0:
        msg = vehicle.wait_message(("MISSION_ITEM_REACHED", "HEARTBEAT"), min(MESSAGE_TIMEOUT_S, left),
                                   _from_autopilot)
        if msg is None:
            continue
        if time.time() > deadline:       # a receive may outlast the time left
            break
        if msg.get_type() == "HEARTBEAT":
            if msg.custom_mode != MODE["AUTO"]:
                raise FlightCheckFailed(f"AUTO left (mode {msg.custom_mode}) before mission item {seq}")
        elif msg.seq == seq:
            return
        elif msg.seq > seq:
            raise FlightCheckFailed(f"mission item {msg.seq} reached before item {seq}")
    raise FlightCheckFailed(f"mission item {seq} not reached within {timeout_s:.0f} s")


def _from_autopilot(msg: object) -> bool:
    """Not a ground station's HEARTBEAT (Mission Planner's reaches the link through the mirror's board side)."""
    return msg.get_type() != "HEARTBEAT" or msg.autopilot != M.MAV_AUTOPILOT_INVALID


def leg_timeout_s(length_m: float) -> float:
    return length_m / MIN_GROUND_SPEED_M_S + TURN_ALLOWANCE_S


def fly_passes(vehicle: Vehicle, card: Card, flown: list[dict]) -> None:
    """Upload the card, fly it in AUTO and append each pass to flown as it ends, with the board times its window
    opened and closed (entry and exit reached), so the passes flown survive a later failure; then RTL for END_RTL_S."""
    upload_mission(vehicle, mission_items(card))
    vehicle.set_mode("AUTO")
    previous = None
    for p in card.passes:
        start = previous or card.home
        wait_reached(vehicle, p.first_seq, leg_timeout_s(distance_m(start, p.approach)))
        wait_reached(vehicle, p.first_seq + 1, leg_timeout_s(APPROACH_M - ENTRY_M))
        opened = board_time_ms(vehicle)
        wait_reached(vehicle, p.first_seq + 2, leg_timeout_s(ENTRY_M + EXIT_M))
        closed = board_time_ms(vehicle)
        flown.append({"height_m": p.height_m, "heading_deg": p.heading_deg, "alt_rel_home_m": p.alt_rel_home_m,
                      "window_board_ms": [opened, closed]})
        print(f"[driver] pass {len(flown)}/{len(card.passes)}: {p.height_m:g} m above the board, heading "
              f"{p.heading_deg:g} deg, board time {opened}-{closed} ms", flush=True)
        previous = p.exit
    vehicle.set_mode(END_MODE)
    vehicle.fly_for(END_RTL_S)
