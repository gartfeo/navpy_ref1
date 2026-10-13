"""Mission Planner watching a board run: the runner's MAVLink link to the board mirrored on a UDP port.

The runner holds the board's MAVLink port (COM15) for the whole run, so Mission Planner cannot open
it. With --mp-mirror [PORT] every MAVLink packet the runner reads from the board is also sent to UDP
127.0.0.1:PORT, and every packet that comes back from that port goes to the board. In Mission
Planner: connection type UDP, Connect, local port PORT (DEFAULT_PORT unless given).

- The board reboots during the parameter load (and after an erase): board.Board attaches every new
  connection (attach) and detaches it before closing it (detach). Mission Planner's packets are
  dropped, and counted, while the link is down; it then sees the new boot's heartbeats.
- Packets reach Mission Planner as the runner reads them. The runner reads throughout the run
  except while it starts the HITL processes and between its arming attempts; the board's packets
  then arrive late, in a burst.
- Mission Planner cannot fly the aircraft or change the runner's link (BLOCKED_MESSAGES): its RC
  overrides and manual control are dropped (the board also takes them only from SYSID_MYGCS, which
  sitl/hitl.parm sets to the runner's 250, GCS_Common.cpp:3990 and :7034; Mission Planner is 255),
  so are its writes of SYSID_MYGCS (BLOCKED_PARAMS), and its stream-rate requests (REQUEST_DATA_STREAM,
  MAV_CMD_SET_MESSAGE_INTERVAL), which would replace the runner's rates on this port (request_streams)
  and, from REQUEST_DATA_STREAM, be saved in SR0_* (ArduPlane persists them: GCS_Mavlink.h:43,
  GCS_Param.cpp:155-197). Everything else passes (modes, arming, parameters, missions: the
  operator's choice).
"""
from __future__ import annotations

import socket
import threading
from collections import Counter
from typing import Optional, Protocol

from pymavlink.dialects.v20 import ardupilotmega as mavlink2      # parses MAVLink 1 and 2 packets

LOOPBACK = "127.0.0.1"
# Ports in use on this PC (UDP and TCP): the GCS stacks (src/gcs/backend/instance_ports.py and the
# SITL instances they start), the bench, the HITL bridge and its JSBSim program (9620-9622).
RESERVED_PORT_BANDS = (("GCS frontend", 3000, 3084), ("SITL", 5501, 5521), ("SITL", 5760, 8300),
                       ("GCS backend", 8000, 8084), ("Mission Planner and GCS monitors", 14550, 16634),
                       ("eval telemetry", 24000, 24254), ("bench", 17000, 19999), ("HITL bridge", 9620, 9622))
DEFAULT_PORT = 9640                    # next to the HITL bridge's ports, outside every band above
MAX_DATAGRAM = 65535
BLOCKED_MESSAGES = frozenset({"RC_CHANNELS_OVERRIDE", "MANUAL_CONTROL", "REQUEST_DATA_STREAM"})
BLOCKED_PARAMS = frozenset({"SYSID_MYGCS"})
BLOCKED_COMMANDS = frozenset({mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL})
COMMAND_MESSAGES = ("COMMAND_LONG", "COMMAND_INT")
UNPARSED = "BAD_DATA"                  # pymavlink's name for bytes that are no packet (a message id it
                                       # does not know still parses, as UNKNOWN_<id>, and is forwarded)
# close() wakes the receiver at once with a datagram of its own; this only bounds a stuck receiver
CLOSE_TIMEOUT_S = 1.0


class Connection(Protocol):
    """The part of a pymavlink connection the mirror uses."""
    message_hooks: list

    def write(self, buf: bytes) -> object:
        """Send raw bytes to the board."""


def reserved_band(port: int) -> Optional[str]:
    """The name of the band this port is in, or None."""
    return next((name for name, low, high in RESERVED_PORT_BANDS if low <= port <= high), None)


def blocked(msg: mavlink2.MAVLink_message) -> bool:
    """A packet from Mission Planner that must not reach the board (see the module notes)."""
    kind = msg.get_type()
    return (kind in BLOCKED_MESSAGES or (kind == "PARAM_SET" and msg.param_id in BLOCKED_PARAMS)
            or (kind in COMMAND_MESSAGES and msg.command in BLOCKED_COMMANDS))


class MavlinkMirror:
    """The board's packets to UDP 127.0.0.1:port; the packets from that port to the board."""

    def __init__(self, port: int) -> None:
        self.port = port
        self.stats: Counter = Counter()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((LOOPBACK, 0))              # Mission Planner answers to this address
        self._parser = mavlink2.MAVLink(None)
        self._parser.robust_parsing = True
        self._lock = threading.Lock()               # the receiver's write against detach()
        self._conn: Optional[Connection] = None
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._receive, name="mp-mirror", daemon=True)
        self._thread.start()

    # ---- the board's side ----------------------------------------------------------
    def attach(self, conn: Connection) -> None:
        """Mirror this (new) connection: its packets go out, Mission Planner's come in."""
        conn.message_hooks.append(self._from_board)
        with self._lock:
            self._conn = conn

    def detach(self) -> None:
        """The connection is about to close: Mission Planner's packets are dropped until the next attach."""
        with self._lock:
            self._conn = None

    def _from_board(self, conn: Connection, msg: mavlink2.MAVLink_message) -> None:
        if msg.get_type() == UNPARSED:
            return
        try:
            self._sock.sendto(msg.get_msgbuf(), (LOOPBACK, self.port))
            self.stats["to_mission_planner"] += 1
        except OSError:
            self.stats["send_errors"] += 1

    # ---- Mission Planner's side ---------------------------------------------------------
    def _receive(self) -> None:
        while not self._closed.is_set():
            try:
                data, (_, sender_port) = self._sock.recvfrom(MAX_DATAGRAM)
            except ConnectionResetError:        # Windows: an earlier send found no Mission Planner listening
                continue
            except OSError:                     # the socket was closed
                return
            if self._closed.is_set():           # close()'s wake-up
                return
            if sender_port != self.port:
                self.stats["foreign"] += 1      # not Mission Planner's port
                continue
            for msg in self._parser.parse_buffer(data) or []:
                self._to_board(msg)

    def _to_board(self, msg: mavlink2.MAVLink_message) -> None:
        if msg.get_type() == UNPARSED:
            self.stats["unparsed"] += 1
            return
        if blocked(msg):
            self.stats[f"blocked_{msg.get_type()}"] += 1
            return
        with self._lock:
            if self._conn is None:
                self.stats["dropped_link_down"] += 1
                return
            try:
                self._conn.write(msg.get_msgbuf())
                self.stats["from_mission_planner"] += 1
            except OSError:
                self.stats["dropped_write_error"] += 1

    def close(self) -> dict:
        """Stop the receiver and close the socket; returns the packet counts for the run's record."""
        self.detach()
        self._closed.set()
        try:
            self._sock.sendto(b"", self._sock.getsockname())     # wakes the receiver's recvfrom
        except OSError:
            pass
        self._thread.join(timeout=CLOSE_TIMEOUT_S)
        self._sock.close()
        return {"port": self.port, **self.stats}
