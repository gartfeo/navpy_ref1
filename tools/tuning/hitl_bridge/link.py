"""MAVLink 2 link of the HITL bridge over a byte transport (transport.py), with reconnect.

The port is dedicated to the firmware HIL backend (HIL link contract v2, fork
libraries/SITL/examples/HIL/readme.md, SERIALn_PROTOCOL 100): on the Pixhawk 6X it is
SERIAL8 = USB OTG2, the second CDC interface (a second COM port on Windows); in SITL a
SERIALn that SITL manages (SERIAL0-7) mapped with --serialN=tcp:<port>, which the bridge
connects to. When the bridge runs in WSL next to the JSBSim program, the board's COM port
stays on Windows and relay.py carries its bytes to the bridge's listener (device
tcpin:<port>). Messages are common-dialect MAVLink 2 with CRC checked; MAVLink 1 frames and
frames carrying the bridge's own system id (loopback) are ignored and counted. The bridge
sends as sysid 251 (the contract's convention; the board's SYSID_THISMAV must differ),
MAV_COMP_ID_ONBOARD_COMPUTER.

A board reboot makes the USB port disappear and come back (Windows re-enumerates it),
SITL drops its TCP client, and the relay may reconnect. Any OSError from the transport
closes it; the next receive() reopens it (a listener accepts the next client) with a
fresh parser. down_for() tells how long the link has been without a connection, since it
was made or since the last connection ended, for the bridge's exit after a grace time (so a
bridge that never gets a client ends too).
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

from pymavlink.dialects.v20 import common as mavlink2

from tools.tuning.hitl_bridge.transport import NoClientYet, Transport

BRIDGE_SYSID = 251
BRIDGE_COMPID = mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
ARMED_FLAG = mavlink2.MAV_MODE_FLAG_SAFETY_ARMED   # bit 128 of HIL_ACTUATOR_CONTROLS.mode
# HIL_ACTUATOR_CONTROLS.flags bit 0: the board runs in lockstep and waits for every reply (SITL
# only; common.xml: "1: indicate simulation using lockstep"). Contract v2: flags is 0 or this.
LOCKSTEP_FLAG = 1
ACTUATORS_TYPE = "HIL_ACTUATOR_CONTROLS"
BAD_DATA_TYPE = "BAD_DATA"          # pymavlink robust parsing: bad prefix byte, CRC or length

# A rebooting board is absent for seconds (bootloader, then the firmware's CDC ports),
# so reopening once per second finds it soon after it is back without spinning.
DEFAULT_RECONNECT_INTERVAL_S = 1.0


@dataclass
class LinkStats:
    connects: int = 0
    disconnects: int = 0
    open_failures: int = 0
    rx_actuators: int = 0
    rx_other: int = 0           # other message types, ignored
    rx_mavlink1: int = 0        # MAVLink 1 frames, ignored (contract v2: MAVLink 2 only)
    rx_loopback: int = 0        # frames with the bridge's own system id, ignored
    rx_bad: int = 0             # framing/CRC failures reported by the parser
    tx_states: int = 0


class MavlinkLink:
    """Receive HIL_ACTUATOR_CONTROLS, send HIL_STATE_QUATERNION, reconnect on loss."""

    def __init__(self, transport: Transport, reconnect_interval_s: float = DEFAULT_RECONNECT_INTERVAL_S,
                 sleep: Callable[[float], None] = time.sleep, log: Callable[[str], None] = print,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._transport = transport
        self._reconnect_interval_s = reconnect_interval_s
        self._sleep = sleep
        self._log = log
        self._clock = clock
        self.connected = False
        self.stats = LinkStats()
        self._down_since: Optional[float] = clock()  # None while connected
        self._mav = self._new_codec()

    def _new_codec(self) -> mavlink2.MAVLink:
        mav = mavlink2.MAVLink(self, srcSystem=BRIDGE_SYSID, srcComponent=BRIDGE_COMPID)
        mav.robust_parsing = True   # report bad frames as BAD_DATA instead of raising
        return mav

    def write(self, data: bytes) -> None:
        """File interface for MAVLink.send."""
        self._transport.write(data)

    def down_for(self) -> float:
        """Seconds without a connection, since the link was made or the last connection ended; 0 while connected."""
        return 0.0 if self._down_since is None else self._clock() - self._down_since

    def connect(self, should_stop: Callable[[], bool]) -> bool:
        """Open the transport, retrying until it opens or should_stop(); True when connected.

        A listener waiting for its client (NoClientYet) already waited READ_TIMEOUT_S, so it
        is asked again at once, without the reconnect sleep and without a log line.
        """
        reported = False
        while not self.connected:
            if should_stop():
                return False
            try:
                self._transport.open()
            except NoClientYet:
                continue
            except OSError as exc:
                if not reported:        # once per outage, not once per attempt
                    self._log(f"link: cannot open ({exc}); retrying every {self._reconnect_interval_s} s")
                    reported = True
                self.stats.open_failures += 1
                self._sleep(self._reconnect_interval_s)
                continue
            self.connected = True
            self._down_since = None
            self.stats.connects += 1
            self._mav = self._new_codec()      # drop any half-received frame from before
            self._log(f"link: connected (#{self.stats.connects})")
        return True

    def _drop(self, exc: OSError) -> None:
        self._transport.close()
        self.connected = False
        self._down_since = self._clock()
        self.stats.disconnects += 1
        self._log(f"link: lost ({exc})")

    def receive(self, should_stop: Callable[[], bool]) -> list:
        """The HIL_ACTUATOR_CONTROLS messages in the next chunk of bytes, in order."""
        if not self.connect(should_stop):
            return []
        try:
            data = self._transport.read()
        except OSError as exc:
            self._drop(exc)
            return []
        actuators = []
        for msg in self._mav.parse_buffer(data) or []:
            kind = msg.get_type()
            if kind == BAD_DATA_TYPE:
                self.stats.rx_bad += 1
            elif msg.get_msgbuf()[0] != mavlink2.PROTOCOL_MARKER_V2:
                self.stats.rx_mavlink1 += 1
            elif msg.get_srcSystem() == BRIDGE_SYSID:
                self.stats.rx_loopback += 1
            elif kind != ACTUATORS_TYPE:
                self.stats.rx_other += 1
            else:
                actuators.append(msg)
        self.stats.rx_actuators += len(actuators)
        return actuators

    def send_state(self, time_usec: int, fields: dict) -> bool:
        """Send one HIL_STATE_QUATERNION; False when the link is down or the write failed."""
        if not self.connected:
            return False
        try:
            self._mav.send(self._mav.hil_state_quaternion_encode(time_usec=time_usec, **fields))
        except OSError as exc:
            self._drop(exc)
            return False
        self.stats.tx_states += 1
        return True

    def close(self) -> None:
        """End of the run: close the connection and release a listening port."""
        self._transport.shutdown()
        self.connected = False
