"""The plant's pose to the Unreal vehicle receiver, stamped with the board time it belongs to (optional).

With --unreal-pose HOST:PORT ALT_OFFSET_M the bridge sends the pose it answered the board with to the NavPyUnrealBridge
vehicle receiver (UDP, JSON schema navpy.unreal.vehicle_state.v1), so Unreal renders the camera view from
it. Every datagram carries the frame's board time (board_time_us: the HIL_ACTUATOR_CONTROLS time_usec,
the board's own clock, the same clock as ATTITUDE time_boot_ms) and a sequence number that rises by one
per datagram within the run's session id. The receiver copies the board time of the pose it rendered
into each camera frame (NPYC v2 header), so a frame says which pose it shows on the board clock.

One datagram per POSE_PERIOD_US slot of board time (board_time_us // POSE_PERIOD_US): the first frame
in each new slot is sent. Slots, not "period since the last send", so a board loop slightly shorter than
the period (SITL measured 19.992 ms frames) does not halve the rate; board time going back (a reboot)
sends at once, even within the same slot number. The send happens after the board's reply and never waits: the socket is
non-blocking, and a full buffer, a closed receiver port (Windows reports it as a reset on the next
send) or any other socket error only counts a dropped pose. Unreal being down never touches the
board link or the link-health gate.

This is simulator truth for rendering only: it goes to Unreal and nowhere else, never into navpy's
navigation (Pure Vision Approach Constraint). Altitude: the receiver in Cesium mode takes alt_m as a
WGS84 ellipsoid height, so the plant's MSL altitude gets one named offset (ellipsoid minus MSL at the
origin, the second value of --unreal-pose), required and without a default, as the aas-hitl truth
bridge does.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import uuid
from dataclasses import dataclass
from typing import Optional, Protocol

from tools.tuning.hitl_bridge.physics import PlantState

VEHICLE_STATE_SCHEMA = "navpy.unreal.vehicle_state.v1"     # NavPyVehicleStateReceiver.cpp VehicleStateSchema
# 50 Hz of board time: above the 30 Hz camera, so the newest pose a frame can be rendered from is at
# most one period old, and well below the board's 200 Hz loop, so the receiver's game thread is not
# flooded. The pose sent is the state answering that frame (no interpolation).
POSE_PERIOD_US = 20_000
# Unreal's JSON reader keeps numbers as doubles: integers stay exact up to 2**53.
MAX_JSON_INTEGER = 2 ** 53 - 1


def parse_endpoint(text: str) -> tuple[str, int]:
    """HOST:PORT -> (IPv4 address, port), the name resolved once here; ValueError for anything else."""
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"{text!r}: expected HOST:PORT")
    try:
        return socket.gethostbyname(host), int(port)
    except OSError as exc:
        raise ValueError(f"{text!r}: {exc}") from None


def encode_vehicle_state(state: PlantState, board_time_us: int, sequence: int, session_id: str,
                         alt_offset_m: float, sys_id: int) -> bytes:
    """One vehicle_state.v1 datagram; ValueError for a non-finite or out-of-range value (never sent)."""
    values = {"lat": state.lat_deg, "lon": state.lon_deg, "alt": state.alt_msl_m + alt_offset_m,
              "roll": math.degrees(state.roll_rad), "pitch": math.degrees(state.pitch_rad),
              "yaw": math.degrees(state.yaw_rad)}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError("vehicle state has a non-finite value")
    if not (-90.0 <= values["lat"] <= 90.0 and -180.0 <= values["lon"] <= 180.0):
        raise ValueError("vehicle state latitude/longitude out of range")
    if not (0 <= board_time_us <= MAX_JSON_INTEGER and 0 <= sequence <= MAX_JSON_INTEGER):
        raise ValueError("board time or sequence outside the exact JSON integer range")
    payload = {
        "schema": VEHICLE_STATE_SCHEMA, "sys_id": int(sys_id), "session_id": session_id,
        "sequence": int(sequence), "board_time_us": int(board_time_us),
        "position_lla": {"lat_deg": values["lat"], "lon_deg": values["lon"], "alt_m": values["alt"]},
        "attitude_deg": {"roll": values["roll"], "pitch": values["pitch"], "yaw": values["yaw"]},
    }
    return json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")


@dataclass
class PoseSendStats:
    sent: int = 0
    dropped: int = 0        # the socket refused or would block: the pose is lost, nothing waits
    refused: int = 0        # a value the receiver cannot take (non-finite, out of range): never sent

    def __str__(self) -> str:
        return f"{self.sent} sent, {self.dropped} dropped, {self.refused} refused"


class UnrealPoseSender:
    """Send the answered pose at most once per POSE_PERIOD_US of board time; never blocks."""

    def __init__(self, endpoint: tuple[str, int], alt_offset_m: float, sys_id: int = 1,
                 period_us: int = POSE_PERIOD_US, sock: Optional[socket.socket] = None) -> None:
        if not math.isfinite(alt_offset_m):
            raise ValueError("--unreal-alt-offset-m must be a finite number of metres")
        if period_us <= 0:
            raise ValueError("pose period must be positive")
        # Resolved once here: a name lookup inside sendto could block the frame loop.
        self.endpoint = parse_endpoint(f"{endpoint[0]}:{endpoint[1]}")
        self.alt_offset_m = float(alt_offset_m)
        self.sys_id = int(sys_id)
        self.period_us = int(period_us)
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.last_slot: Optional[int] = None
        self.last_time_us: Optional[int] = None
        self.stats = PoseSendStats()
        self._sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def due(self, board_time_us: int) -> bool:
        went_back = self.last_time_us is not None and board_time_us < self.last_time_us
        self.last_time_us = board_time_us
        return went_back or board_time_us // self.period_us != self.last_slot

    def __call__(self, board_time_us: int, state: PlantState) -> None:
        if not self.due(board_time_us):
            return
        try:
            datagram = encode_vehicle_state(state, board_time_us, self.sequence, self.session_id,
                                            self.alt_offset_m, self.sys_id)
        except ValueError:
            self.stats.refused += 1
            return
        self.last_slot = board_time_us // self.period_us
        self.sequence += 1
        try:
            self._sock.sendto(datagram, self.endpoint)
        except OSError:             # BlockingIOError, ConnectionResetError (ICMP unreachable), ...
            self.stats.dropped += 1
            return
        self.stats.sent += 1

    def summary(self) -> str:
        return f", Unreal poses {self.stats}"

    def close(self) -> None:
        self._sock.close()


class PoseOut(Protocol):
    """What the bridge holds for --unreal-pose: a sender, or NO_POSE_OUT without the option."""

    def __call__(self, board_time_us: int, state: PlantState) -> None: ...

    def summary(self) -> str: ...

    def close(self) -> None: ...


class _NoPoseOut:
    def __call__(self, board_time_us: int, state: PlantState) -> None:
        pass

    def summary(self) -> str:
        return ""

    def close(self) -> None:
        pass


NO_POSE_OUT: PoseOut = _NoPoseOut()


class _UnrealPoseOption(argparse.Action):
    """--unreal-pose HOST:PORT ALT_OFFSET_M, checked while parsing: the offset always comes with the endpoint."""

    def __call__(self, parser: argparse.ArgumentParser, namespace: argparse.Namespace, values: object,
                 option_string: Optional[str] = None) -> None:
        endpoint, offset = values  # type: ignore[misc]
        try:
            alt_offset_m = float(offset)
            if not math.isfinite(alt_offset_m):
                raise ValueError(f"altitude offset {offset!r} is not a finite number")
            setattr(namespace, self.dest, (parse_endpoint(endpoint), alt_offset_m))
        except ValueError as exc:
            parser.error(f"--unreal-pose: {exc}")


def add_unreal_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """The bridge's --unreal-pose option; returns the parser."""
    parser.add_argument("--unreal-pose", nargs=2, metavar=("HOST:PORT", "ALT_OFFSET_M"), default=None,
                        action=_UnrealPoseOption,
                        help="send the answered pose with its board time to the Unreal vehicle receiver (UDP, one "
                             f"per {POSE_PERIOD_US // 1000} ms of board time; never blocks the board). ALT_OFFSET_M: "
                             "WGS84 ellipsoid height minus MSL altitude at the origin (Cesium takes ellipsoid "
                             "heights); required, no default")
    return parser


def unreal_pose_option(args: argparse.Namespace) -> PoseOut:
    """The Unreal pose sender for --unreal-pose, or NO_POSE_OUT without it."""
    if args.unreal_pose is None:
        return NO_POSE_OUT
    endpoint, alt_offset_m = args.unreal_pose
    return UnrealPoseSender(endpoint, alt_offset_m)
