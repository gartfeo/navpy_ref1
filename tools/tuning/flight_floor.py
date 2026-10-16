"""The run's height floor (HeightFloor: below it the run fails) and the recovery after a breach (recover_above)."""
from __future__ import annotations

import argparse
from types import TracebackType
from typing import Callable, Optional

from tools.tuning.flight_steps import CLIMB_CHECK_S, CLIMB_RATE_MIN_MPS, CLIMB_SETTLE_S, FlightCheckFailed
from tools.tuning.vehicle import Vehicle

# After the takeoff has reached TKOFF_ALT the run fails below this height above home. Every step of these
# runs starts at TKOFF_ALT (120 m in the tuning runs) or above the tuner's FTUN_ALT_MIN (40-80 m); none is
# meant to come near the ground. In round 2 a vs-autotune card started at 36 m, struck the ground and the
# run still exited 0. 30 m leaves room for the pull-out of a manoeuvre that has gone wrong.
MIN_HEIGHT_M = 30.0
# After a floor breach the run has failed, but the aircraft is not left diving in the mode that failed until the
# vehicle's stop ends the flight (on a board: a disarm, which on a real aircraft would be a crash): RTL is ArduPlane's
# own recovery, it climbs to RTL_ALTITUDE and circles home. The stop comes once it is back above the floor, or after
# the time a LOITER climb of the same height is given (climb_to's limit).
RECOVERY_MODE = "RTL"


def add_floor_arg(p: argparse.ArgumentParser, default_m: float = MIN_HEIGHT_M) -> None:
    p.add_argument("--min-height", type=float, default=default_m, metavar="M",
                   help="m above home: once the takeoff has reached TKOFF_ALT the run fails below this "
                        "(default %(default)g)")


class HeightFloor:
    """Inside the with-block the run fails (FlightCheckFailed) once the aircraft is below floor_m above home,
    and on leaving the block after such a breach the aircraft recovers in RECOVERY_MODE (recover_above).

    The check is a hook on the MAVLink connection (pymavlink mavfile.message_hooks, called for every
    received message in post_message), so every GLOBAL_POSITION_INT is seen whichever helper reads the
    link. It trips once and then lets messages pass, and leaving the block removes it: the vehicle's
    stop (a board's disarm and log download) must not trip it again. On leaving, record["height"] holds
    the floor, the lowest height seen and the breach (None when the floor held), and after a breach
    record["recovery"] what the recovery did (an error there is recorded; the breach is what is raised).

    before_recovery (a HITL approach: navpy_handoff.NavpyHandoff.before_rtl) runs first after a breach and its record
    is kept in record["navpy_handoff"]: navpy is stopped and its router endpoint closed before the RTL. When it has
    sent the RTL itself on the board's own port (its "rtl" is "direct": the router is gone), recover_above is not run;
    nor when it raised, as navpy's endpoint is then not known closed (NavpyHandoff records its errors instead)."""

    def __init__(self, vehicle: Vehicle, floor_m: float, record: dict,
                 before_recovery: Optional[Callable[[], dict]] = None) -> None:
        self._vehicle = vehicle
        self._record = record
        self._before_recovery = before_recovery
        self._conn = None
        self.floor_m = floor_m
        self.lowest_m: float | None = None
        self.breach: str | None = None

    def __enter__(self) -> HeightFloor:
        self._conn = self._vehicle.mav
        self._conn.message_hooks.append(self._check)
        return self

    def __exit__(self, kind: type[BaseException] | None, exc: BaseException | None,
                 trace: TracebackType | None) -> None:
        if self._check in self._conn.message_hooks:
            self._conn.message_hooks.remove(self._check)
        self._record["height"] = {"floor_m": self.floor_m, "lowest_m": self.lowest_m, "breach": self.breach}
        if self.breach is not None:
            if self._before_recovery is not None:
                try:
                    handoff = self._record["navpy_handoff"] = self._before_recovery()
                except Exception as exc:     # noqa: BLE001 - navpy's endpoint not known closed: no RTL behind it
                    self._record["navpy_handoff"] = {"error": f"{type(exc).__name__}: {exc}"}
                    self._record["recovery"] = {"mode": RECOVERY_MODE, "error": "navpy handoff failed: no RTL sent"}
                    return
                if handoff.get("rtl") == "direct":
                    self._record["recovery"] = {"mode": RECOVERY_MODE, "sent_by": "navpy handoff"}
                    return
            try:
                self._record["recovery"] = recover_above(self._vehicle, self.floor_m)
            except Exception as exc:     # noqa: BLE001 - the breach stays the run's failure; the record keeps this
                self._record["recovery"] = {"mode": RECOVERY_MODE, "error": f"{type(exc).__name__}: {exc}"}

    def _check(self, conn: object, msg: object) -> None:
        if self.breach is not None or msg.get_type() != "GLOBAL_POSITION_INT":
            return
        height = msg.relative_alt / 1000.0
        self.lowest_m = height if self.lowest_m is None else min(self.lowest_m, height)
        if height < self.floor_m:
            self.breach = f"height {height:.1f} m above home, below the run's floor of {self.floor_m:g} m"
            raise FlightCheckFailed(self.breach)


def recover_above(vehicle: Vehicle, floor_m: float) -> dict:
    """RECOVERY_MODE until the aircraft is back at or above floor_m above home, or until the time a climb of that
    height is given (as climb_to). Returns the mode, whether it got there, the height and the simulated seconds."""
    vehicle.set_mode(RECOVERY_MODE)
    start = vehicle.state()["alt"]
    limit_s = max(0.0, floor_m - (start or 0.0)) / CLIMB_RATE_MIN_MPS + CLIMB_SETTLE_S
    flown = 0.0
    while True:
        height = vehicle.state()["alt"]
        recovered = height is not None and height >= floor_m
        if recovered or flown >= limit_s:
            print(f"[driver] {RECOVERY_MODE} after the floor breach: {height} m above home after {flown:g} s "
                  f"({'back above' if recovered else 'still below'} the floor of {floor_m:g} m)", flush=True)
            return {"mode": RECOVERY_MODE, "recovered": recovered, "height_m": height, "seconds": flown}
        vehicle.fly_for(CLIMB_CHECK_S)
        flown += CLIMB_CHECK_S
