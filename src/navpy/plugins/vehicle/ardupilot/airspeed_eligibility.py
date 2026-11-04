"""Eligible airspeed: the only airspeed a HITL final-approach law may read (hitl_final_approach.md section 9, step 20).

The source is the own autopilot's selected pitot sensor, as AIRSPEED (295) reports it (decoded on the bus, step 19),
together with SYS_STATUS's differential-pressure bits. VFR_HUD's airspeed (flight-summary telemetry, which ArduPilot
fills with its synthetic estimate when no sensor is used) is never a fallback.

States: pending (nothing proven yet), admitted, revoked. Admission needs, all at once:
- an accepted COMMAND_ACK for the 20 Hz AIRSPEED claim (SET_MESSAGE_INTERVAL 295, 50,000 us; the PC router keeps it
  as a per-client stream claim, mav_router_policy.StreamClaims);
- the pinned parameters read back from the board with the profile's values (ARSPD_TYPE, ARSPD_USE, ARSPD_PRIMARY);
- 60 healthy seconds at 18 Hz or more: every AIRSPEED of the selected sensor fresh, finite, USING and not UNHEALTHY,
  and every SYS_STATUS fresh with the sensor present, enabled and healthy. A bad packet or a gap restarts the window.
Once admitted, any fault revokes for good: an unhealthy, not-used, non-finite or stale packet, a stale or bad
SYS_STATUS, another sensor reported as the one in use, a pinned parameter that changes, or a refused claim. A revoked
source is never readmitted in this process (one trial); a revoke before admission is equally final.

Only own-autopilot packets reach here: the inbound router admits AIRSPEED and SYS_STATUS only from the target
system's autopilot component (inbound_router.OWN_AUTOPILOT_ONLY_TYPES), and every callback checks again
(airspeed_attach.py). Pure vision constraint: airspeed is an allowed law input; nothing here reads ground speed,
altitude or heading.
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.airspeed_profile import (
    ADMIT_MIN_RATE_HZ,
    ADMIT_WINDOW_S,
    ADMITTED,
    AIRSPEED_INTERVAL_US,
    AIRSPEED_MAX_AGE_S,
    CLAIM_REFUSED,
    CLAIM_UNACKNOWLEDGED,
    DIFFERENTIAL_PRESSURE,
    FLAG_UNHEALTHY,
    FLAG_USING,
    MAV_RESULT_ACCEPTED,
    PENDING,
    PIN_CHANGED,
    PIN_MISMATCH,
    PIN_UNREADABLE,
    REVOKED,
    SENSOR_NOT_USED,
    SENSOR_STALE,
    SENSOR_UNHEALTHY,
    SENSOR_VALUE_INVALID,
    SOURCE_CHANGED,
    STATUS_BAD,
    STATUS_MAX_AGE_S,
    STATUS_STALE,
    AirspeedProfile,
    pin_matches,
)

AirspeedEvent = Callable[[dict[str, object]], None]


def _emit(events: AirspeedEvent, event: str, now_s: float, **detail: object) -> None:
    try:
        events({"event": event, "t_s": now_s, **detail})
    except Exception:  # noqa: BLE001 - the log is evidence, never authority
        return


@dataclass
class _Setup:
    """The claim and the pins: both must hold before the healthy window opens."""

    claim_sent_s: float | None = None
    ack_ok: bool = False
    pins_ok: bool = False

    @property
    def ready(self) -> bool:
        return self.ack_ok and self.pins_ok


@dataclass
class _Latest:
    airspeed: float | None = None
    airspeed_s: float | None = None
    status_s: float | None = None


@dataclass
class _Window:
    """The healthy window: its first good packet's time and the good packets after it."""

    start_s: float | None = None
    count: int = 0


class AirspeedEligibility:
    """The source's state; fed on the bus reader thread, read on the vision and command threads."""

    def __init__(self, profile: AirspeedProfile, events: AirspeedEvent = lambda record: None) -> None:
        self._profile = profile
        self._events = events
        self._lock = rt.Lock()
        self._state = PENDING
        self._reason: str | None = None
        self._setup = _Setup()
        self._latest = _Latest()
        self._window = _Window()
        self.counts = {"airspeed": 0, "other_sensor": 0, "status": 0, "reads": 0, "refused_reads": 0}

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def claim_acknowledged(self) -> bool:
        with self._lock:
            return self._setup.ack_ok

    @property
    def revoked(self) -> str | None:
        """The revoke reason, or None while not revoked."""
        with self._lock:
            return self._reason if self._state == REVOKED else None

    # -- setup -------------------------------------------------------------------------------------------------
    def claim_sent(self, now_s: float) -> None:
        with self._lock:
            self._setup.claim_sent_s = now_s
        _emit(self._events, "claim", now_s, interval_us=AIRSPEED_INTERVAL_US)

    def observe_ack(self, result: int, now_s: float) -> None:
        """A SET_MESSAGE_INTERVAL COMMAND_ACK from the own autopilot after the claim. MAVLink's ack names no
        message id, so any refusal in this window counts as the claim's (fail closed)."""
        with self._lock:
            if self._setup.claim_sent_s is None or self._state == REVOKED:
                return
            accepted = result == MAV_RESULT_ACCEPTED
            if accepted:
                self._setup.ack_ok = True
        _emit(self._events, "claim_ack", now_s, result=int(result))
        if not accepted:
            self._revoke(CLAIM_REFUSED, now_s, result=int(result))

    def ack_deadline(self, now_s: float) -> None:
        """The claim's ack wait ended: no accepted ack is a revoke."""
        if not self.claim_acknowledged:
            self._revoke(CLAIM_UNACKNOWLEDGED, now_s)

    def pins_read(self, values: dict[str, float | None], now_s: float) -> None:
        expected = self._profile.pins()
        _emit(self._events, "pins", now_s, read=values, expected=expected)
        if any(values.get(name) is None for name in expected):
            self._revoke(PIN_UNREADABLE, now_s)
            return
        if any(not pin_matches(values[name], value) for name, value in expected.items()):
            self._revoke(PIN_MISMATCH, now_s)
            return
        with self._lock:
            self._setup.pins_ok = True

    # -- bus ---------------------------------------------------------------------------------------------------
    def observe_param(self, name: str, value: float, now_s: float) -> None:
        expected = self._profile.pins().get(name)
        if expected is not None and not pin_matches(value, expected):
            self._revoke(PIN_CHANGED, now_s, name=name, value=float(value))

    def observe_airspeed(self, sensor: int, airspeed: float, flags: int, now_s: float) -> None:
        if sensor != self._profile.sensor:
            with self._lock:
                self.counts["other_sensor"] += 1
            if flags & FLAG_USING:
                # The AHRS uses another sensor: the selected one is not the source in use.
                self._fault(SOURCE_CHANGED, now_s, sensor=sensor)
            return
        with self._lock:
            self.counts["airspeed"] += 1
            previous_s = self._latest.airspeed_s
        fault = None
        if not (isinstance(airspeed, float) and math.isfinite(airspeed)):
            fault = SENSOR_VALUE_INVALID
        elif flags & FLAG_UNHEALTHY:
            fault = SENSOR_UNHEALTHY
        elif not flags & FLAG_USING:
            fault = SENSOR_NOT_USED
        elif previous_s is not None and now_s - previous_s > AIRSPEED_MAX_AGE_S:
            fault = SENSOR_STALE
        if fault is not None and not self._fault(fault, now_s, flags=int(flags)):
            return      # revoked, or a bad packet before admission
        with self._lock:
            # A good packet; after a gap before admission it starts the new window.
            self._latest.airspeed, self._latest.airspeed_s = float(airspeed), now_s
            # The window opens only once the claim is acknowledged and the pins hold, so it measures the claim.
            if self._state == PENDING and self._setup.ready:
                if self._window.start_s is None:
                    self._window = _Window(now_s, 0)
                else:
                    self._window.count += 1
        self._try_admit(now_s)

    def observe_status(self, present: int, enabled: int, health: int, now_s: float) -> None:
        with self._lock:
            self.counts["status"] += 1
            previous_s = self._latest.status_s
        if not (present & enabled & health) & DIFFERENTIAL_PRESSURE:
            self._fault(STATUS_BAD, now_s, present=bool(present & DIFFERENTIAL_PRESSURE),
                        enabled=bool(enabled & DIFFERENTIAL_PRESSURE), healthy=bool(health & DIFFERENTIAL_PRESSURE))
            return
        if (previous_s is not None and now_s - previous_s > STATUS_MAX_AGE_S
                and not self._fault(STATUS_STALE, now_s, age_s=now_s - previous_s)):
            return
        with self._lock:
            self._latest.status_s = now_s

    # -- law input ---------------------------------------------------------------------------------------------
    def read(self, now_s: float) -> float | None:
        """The airspeed (m/s) the law may use now, else None. A stale source found here revokes."""
        with self._lock:
            state, latest = self._state, _Latest(**vars(self._latest))
            if state != ADMITTED:
                self.counts["refused_reads"] += 1
                return None
        if latest.airspeed_s is None or now_s - latest.airspeed_s > AIRSPEED_MAX_AGE_S:
            self._revoke(SENSOR_STALE, now_s, age_s=None if latest.airspeed_s is None else now_s - latest.airspeed_s)
            return None
        if latest.status_s is None or now_s - latest.status_s > STATUS_MAX_AGE_S:
            self._revoke(STATUS_STALE, now_s, age_s=None if latest.status_s is None else now_s - latest.status_s)
            return None
        with self._lock:
            if self._state != ADMITTED:
                self.counts["refused_reads"] += 1
                return None
            self.counts["reads"] += 1
        return latest.airspeed

    # -- internals ---------------------------------------------------------------------------------------------
    def _try_admit(self, now_s: float) -> None:
        with self._lock:
            if self._state != PENDING or not self._setup.ready or self._window.start_s is None:
                return
            if self._latest.status_s is None or now_s - self._latest.status_s > STATUS_MAX_AGE_S:
                return
            elapsed = now_s - self._window.start_s
            if elapsed < ADMIT_WINDOW_S:
                return
            rate_hz = self._window.count / elapsed
            if rate_hz < ADMIT_MIN_RATE_HZ:
                return
            self._state = ADMITTED
        _emit(self._events, "admitted", now_s, window_s=elapsed, rate_hz=rate_hz)

    def _fault(self, reason: str, now_s: float, **detail: object) -> bool:
        """Before admission a fault restarts the healthy window; after it, it revokes. True when the packet that
        showed the fault is itself usable: only a gap before admission (the packet after it starts the new window)."""
        with self._lock:
            state = self._state
            restarted = state == PENDING and self._window.start_s is not None
            if state == PENDING:
                self._window = _Window()
        if state == ADMITTED:
            self._revoke(reason, now_s, **detail)
            return False
        if restarted:
            _emit(self._events, "window_restart", now_s, reason=reason, **detail)
        return state == PENDING and reason in (SENSOR_STALE, STATUS_STALE)

    def _revoke(self, reason: str, now_s: float, **detail: object) -> None:
        with self._lock:
            if self._state == REVOKED:
                return
            self._state, self._reason = REVOKED, reason
            self._latest.airspeed = None
        _emit(self._events, "revoked", now_s, reason=reason, **detail)


__all__ = ["AirspeedEligibility", "AirspeedEvent"]
