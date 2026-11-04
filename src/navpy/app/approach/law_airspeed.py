"""The HITL law-input guard for airspeed (hitl_final_approach.md section 9, step 20).

With integration timing (a pc-sitl run; ``IntegrationTimingConfig`` refuses every other mode) the final-approach law
reads airspeed only from the eligible source (``plugins/vehicle/ardupilot/airspeed_eligibility.py``), never from
VFR_HUD. The guard has two ends:
- input: the frame association's airspeed reader is ``read``; before admission and after a revoke it gives None, so
  every frame is refused (``FinalApproachFrameProjector`` refuses a frame without positive airspeed);
- write: ``guard`` wraps the attitude actuator; a write while the source is not admitted, or stale at the write,
  raises ``AirspeedIneligible`` and sends nothing. The command executor and the held-command refresh both mark the
  episode failed on that error, so the approach ends with NAV_FAIL and no command after a revoke.
Each source event is printed as one flushed ``NAVPY_AIRSPEED_V1 {json}`` line; the approach runner reads them and
fails the trial on any revoke (tools/tuning/approach_airspeed.py).
"""
from __future__ import annotations

import json

from navpy.core.approach.vision_nav.command_transaction import FinalApproachAttitudeActuator
from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.airspeed_attach import AirspeedVehicle, attach
from navpy.plugins.vehicle.ardupilot.airspeed_eligibility import AirspeedEligibility
from navpy.plugins.vehicle.ardupilot.airspeed_profile import profile_for_mode

NAVPY_AIRSPEED_MARKER = "NAVPY_AIRSPEED_V1"


class AirspeedIneligible(RuntimeError):
    """A final-approach write without an admitted, fresh airspeed source."""


def print_airspeed_record(record: dict[str, object]) -> None:
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"), default=str)
    print(f"{NAVPY_AIRSPEED_MARKER} {encoded}", flush=True)


def parse_airspeed_record(line: str) -> dict[str, object] | None:
    prefix = f"{NAVPY_AIRSPEED_MARKER} "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        record = json.loads(line[len(prefix):])
    except ValueError:
        return None
    return record if isinstance(record, dict) else None


class AirspeedGuardedActuator:
    def __init__(self, actuator: FinalApproachAttitudeActuator, source: AirspeedEligibility) -> None:
        self._actuator = actuator
        self._source = source
        self.refused = 0

    def issue(self, roll_deg: float, pitch_deg: float, throttle: float | None) -> None:
        # The check and the send hold the source's lock together: no revoke can fall between them.
        if self._source.read(rt.perf_counter(), lambda: self._actuator.issue(roll_deg, pitch_deg, throttle)) is None:
            self.refused += 1
            raise AirspeedIneligible(f"airspeed source {self._source.state}: {self._source.revoked or 'unproven'}")


class LawAirspeed:
    """The one airspeed input of a HITL run's final-approach law, and its write guard."""

    def __init__(self, source: AirspeedEligibility) -> None:
        self._source = source

    @classmethod
    def attach(cls, vehicle: AirspeedVehicle, mode: str) -> "LawAirspeed":
        """Refuses any mode without a simulated profile (flight, Qualification) before touching the vehicle."""
        profile = profile_for_mode(mode)
        return cls(attach(vehicle, profile, print_airspeed_record))

    @property
    def source(self) -> AirspeedEligibility:
        return self._source

    def read(self) -> float | None:
        return self._source.read(rt.perf_counter())

    def guard(self, actuator: FinalApproachAttitudeActuator) -> AirspeedGuardedActuator:
        return AirspeedGuardedActuator(actuator, self._source)


__all__ = [
    "AirspeedGuardedActuator", "AirspeedIneligible", "LawAirspeed", "NAVPY_AIRSPEED_MARKER", "parse_airspeed_record",
    "print_airspeed_record",
]
