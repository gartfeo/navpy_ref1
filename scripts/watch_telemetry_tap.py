"""A one-way telemetry tap, so a ground station can watch a scored run.

Attaching a ground station to a run's Mission Planner port destroys the run's
SCORE. Not its flight: this was measured as a controlled pair on 2026-08-21,
identical `--cruise-speedup 20 --speedups 1` cases differing only in whether
the viewer sat behind this tap.

    viewer            verdict   coordinate CPA   child snap   0.5s gaps
    direct on 14590   FAIL      3.459 m          0.228 m      3 + 66 spatial
    behind the tap    PASS      0.201 m          0.201 m      none

The child's own snap and its 93.5% fresh-observation fraction were healthy in
BOTH arms, so the navigation law never saw the problem. What broke is the
evaluator's position stream: at 2 Hz and 26.6 m/s the scorer only sees the
aircraft every 13.3 m, so its "miss" is a sampling artefact.

0.500 s is 2 Hz exactly, which is a stream-rate setting rather than congestion.
The evaluator asks for 30 Hz GLOBAL_POSITION_INT with
`MAV_CMD_SET_MESSAGE_INTERVAL` (`eval_navigation_telemetry.py`), ArduPilot applies
that PER CHANNEL, and mavlink-router presents all of its UDP endpoints to the
vehicle as ONE channel. So a second station on that port does not get its own
rate -- it OVERWRITES the evaluator's, and Mission Planner's default
`REQUEST_DATA_STREAM` rate for `MAV_DATA_STREAM_POSITION` is 2 Hz. In the
direct arm the tap's counterpart blocked 60 of exactly that message.

A second, harsher form of the same collision: an already-attached station can
stop the evaluator establishing its stream at all, which fails the case at
setup with `30 Hz coordinate stream was not acknowledged`.

Mission Planner holds its UDP port even while idle, so close it before starting
the tap -- otherwise the tap cannot bind and the run is unprotected.

This tap stands between them. It takes the vehicle's side of the port and
forwards every packet onward to the viewer, and it forwards NOTHING back. The
viewer sees a live vehicle; the vehicle never hears the viewer, so no stream
rate, mode, parameter or command from the viewer can reach it.

    router --> :14590 [tap] --> :17590 --> Mission Planner
                        X   <--            (dropped; configured-peer replies counted)

First launch the eval stack so it claims a chat slot; read its assigned chat
index and Mission Planner port. Then start the tap on that port before the
scored run and point Mission Planner at a free viewer port outside NavPy's
managed port bands. Do not start this example before chat 40 is claimed:
the port check would skip chat 40 and chat 41 uses 14591 for Mission Planner.

    python scripts/watch_telemetry_tap.py --listen 14590 --viewer 17590
"""

from __future__ import annotations

import argparse
import errno
import socket
import sys
import time
from dataclasses import dataclass, field

# A MAVLink frame names its message in the header, so the tap can say what the
# viewer TRIED to send without carrying a MAVLink dependency: v1 is
# `0xFE len seq sysid compid msgid`, v2 is
# `0xFD len flags flags seq sysid compid msgid[3]` little-endian.
_MAVLINK_V1_MAGIC = 0xFE
_MAVLINK_V2_MAGIC = 0xFD
# The two the viewer uses to seize the stream rate, named because they are the
# whole reason this tap exists.
_MESSAGE_NAMES = {
    66: "REQUEST_DATA_STREAM",
    76: "COMMAND_LONG",
    511: "SET_MESSAGE_INTERVAL",
}
_REPORT_INTERVAL_S = 10.0


def message_id(frame: bytes) -> int | None:
    """The message id in a MAVLink frame header, or None if it is not one."""
    if len(frame) >= 8 and frame[0] == _MAVLINK_V1_MAGIC:
        return frame[5]
    if len(frame) >= 12 and frame[0] == _MAVLINK_V2_MAGIC:
        return int.from_bytes(frame[7:10], "little")
    return None


def message_label(msgid: int | None) -> str:
    if msgid is None:
        return "unparsed"
    return _MESSAGE_NAMES.get(msgid, str(msgid))


@dataclass
class TapCounters:
    """What crossed the tap, and what was stopped at it."""

    forwarded: int = 0
    forwarded_bytes: int = 0
    blocked: int = 0
    blocked_by_message: dict[str, int] = field(default_factory=dict)
    #: ICMP port, host, or network-unreachable reports from the OS. This is a
    #: best-effort absence signal, not an acknowledgement of viewer delivery.
    viewer_gone: int = 0
    send_blocked: int = 0

    def record_forward(self, size: int) -> None:
        self.forwarded += 1
        self.forwarded_bytes += size

    def record_viewer_gone(self) -> None:
        self.viewer_gone += 1

    def record_send_blocked(self) -> None:
        self.send_blocked += 1

    def record_block(self, frame: bytes) -> None:
        self.blocked += 1
        label = message_label(message_id(frame))
        self.blocked_by_message[label] = self.blocked_by_message.get(label, 0) + 1

    def summary(self) -> str:
        if not self.blocked:
            blocked = "none"
        else:
            blocked = ", ".join(
                f"{name}x{count}"
                for name, count in sorted(
                    self.blocked_by_message.items(),
                    key=lambda item: -item[1],
                )
            )
        gone = (
            f" | viewer unreachable reports x{self.viewer_gone} (cumulative)"
            if self.viewer_gone else ""
        )
        busy = f" | local send blocked x{self.send_blocked}" if self.send_blocked else ""
        return (
            f"forwarded {self.forwarded} frames "
            f"({self.forwarded_bytes / 1024.0:.0f} KiB) | "
            f"blocked {self.blocked} from the viewer: {blocked}{gone}{busy}"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--listen",
        type=int,
        required=True,
        help="the run's own Mission Planner port, which the tap takes over",
    )
    parser.add_argument(
        "--viewer",
        type=int,
        required=True,
        help="port the ground station listens on; the tap pushes to it",
    )
    parser.add_argument(
        "--viewer-host",
        default="127.0.0.1",
        help="host the ground station is on (default the local machine)",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after this long; 0 runs until interrupted",
    )
    return parser


def run(options: argparse.Namespace) -> TapCounters:
    """Forward vehicle -> viewer, drop viewer -> vehicle, until told to stop."""
    counters = TapCounters()
    deadline = None if options.seconds <= 0.0 else time.monotonic() + options.seconds
    viewer_address = (options.viewer_host, options.viewer)
    if options.listen == options.viewer:
        raise ValueError("viewer port must differ from the listen port")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as inbound, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as outbound:
        if sys.platform == "win32":
            # A rival using SO_REUSEADDR can take packets from a normal bind.
            # The tap must retain exclusive ownership.
            inbound.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        # A connected UDP socket reports ICMP errors on Linux as well as
        # Windows, and receives replies only from the configured viewer.
        # It still has no delivery acknowledgement.
        outbound.connect(viewer_address)
        outbound.settimeout(0.0)
        try:
            inbound.bind(("0.0.0.0", options.listen))
        except OSError as error:
            raise RuntimeError(
                f"tap cannot hold listen port {options.listen}; close the existing listener"
            ) from error
        inbound.settimeout(0.2)
        print(
            f"tap: :{options.listen} -> {options.viewer_host}:{options.viewer} "
            "(one way; nothing returns to the vehicle)",
            flush=True,
        )
        next_report = time.monotonic() + _REPORT_INTERVAL_S
        while deadline is None or time.monotonic() < deadline:
            try:
                frame, _ = inbound.recvfrom(65535)
            except socket.timeout:
                frame = None
            except OSError as error:
                raise RuntimeError("tap inbound receive failed") from error
            if frame:
                try:
                    outbound.send(frame)
                except BlockingIOError:
                    counters.record_send_blocked()
                except (ConnectionRefusedError, ConnectionResetError):
                    # The OS may report a previous send's unreachable port
                    # here. Other socket failures remain visible to the caller.
                    counters.record_viewer_gone()
                except OSError as error:
                    if error.errno not in (errno.EHOSTUNREACH, errno.ENETUNREACH):
                        raise RuntimeError("tap viewer send failed") from error
                    counters.record_viewer_gone()
                else:
                    # A UDP send to a port with nobody on it still succeeds, so
                    # this counts frames HANDED to the OS, not frames received.
                    counters.record_forward(len(frame))
            # Replies from the configured viewer endpoint arrive here and go
            # no further. Connected UDP filters other senders; those packets
            # also cannot reach the vehicle but are not in the block count.
            while True:
                try:
                    reply = outbound.recv(65535)
                except (BlockingIOError, socket.timeout):
                    break
                except (ConnectionRefusedError, ConnectionResetError):
                    counters.record_viewer_gone()
                    break
                except OSError as error:
                    if error.errno not in (errno.EHOSTUNREACH, errno.ENETUNREACH):
                        raise RuntimeError("tap viewer receive failed") from error
                    counters.record_viewer_gone()
                    break
                counters.record_block(reply)
            if time.monotonic() >= next_report:
                print(f"tap: {counters.summary()}", flush=True)
                next_report = time.monotonic() + _REPORT_INTERVAL_S
    print(f"tap: {counters.summary()}", flush=True)
    return counters


def main() -> int:
    options = _parser().parse_args()
    try:
        run(options)
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
