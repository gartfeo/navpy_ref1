"""FBWA trimmed to the aircraft's cruise pitch for one run (CruisePitchTrim); the cruise readings: flight_steps.py."""
from __future__ import annotations

from types import TracebackType

from tools.tuning.flight_steps import (CRUISE_SPEED_TOL_M_S, FBWA_SAMPLE_S, PITCH_TRIM, FlightCheckFailed,
                                       median_flight)
from tools.tuning.vehicle import Vehicle

# The FBWA speed after a trim change converges with a time constant of 6-7 s (trim1); five.
TRIM_SETTLE_S = 36.0
# FBWA flies the cruise pitch this closely once trimmed; FBWA's offset from its demand changes with the speed, so the
# trim is corrected after each settle, at most PITCH_TRIM_ROUNDS times. 0.2 deg of pitch moves a cruising
# aircraft's speed by a fraction of a m/s.
PITCH_MATCH_TOL_DEG = 0.2
PITCH_TRIM_ROUNDS = 3


class CruisePitchTrim:
    """Inside the with-block FBWA's stick centre flies the cruise attitude: PTCH_TRIM_DEG for this run only.

    The trim is moved by the cruise's AHRS pitch less FBWA's (pilot_throttle's readings), then, at most
    PITCH_TRIM_ROUNDS times, FBWA flies TRIM_SETTLE_S and the trim is corrected by what its pitch still misses; once
    within PITCH_MATCH_TOL_DEG, FBWA's median airspeed must be the cruise's within CRUISE_SPEED_TOL_M_S
    (FlightCheckFailed otherwise). On leaving, the old value is set back, also after a failure; a failed restore is
    recorded, not raised. record["pitch_trim"] holds the old and the run's trim, each round, and the restore."""

    def __init__(self, vehicle: Vehicle, pilot: dict, record: dict) -> None:
        self._vehicle = vehicle
        self._pilot = pilot
        self._record = record
        self.before: float | None = None

    def __enter__(self) -> CruisePitchTrim:
        v, pilot = self._vehicle, self._pilot
        self.before = v.get_param(PITCH_TRIM)
        want = pilot["cruise_pitch_deg"]
        trim = round(self.before + want - pilot["fbwa_pitch_deg"], 2)
        entry = self._record["pitch_trim"] = {"name": PITCH_TRIM, "before_deg": self.before, "rounds": []}
        try:
            for _ in range(PITCH_TRIM_ROUNDS):
                v.set_param(PITCH_TRIM, trim)
                v.fly_for(TRIM_SETTLE_S)
                fbwa = median_flight(v, FBWA_SAMPLE_S)
                entry["rounds"].append({"trim_deg": trim, "pitch_deg": fbwa["pitch_deg"],
                                        "airspeed_m_s": fbwa["airspeed_m_s"]})
                miss = want - fbwa["pitch_deg"]
                print(f"[driver] {PITCH_TRIM} {trim:g} deg: FBWA flies {fbwa['pitch_deg']:.2f} deg at "
                      f"{fbwa['airspeed_m_s']:.1f} m/s, the cruise {want:.2f} deg at "
                      f"{pilot['cruise_airspeed_m_s']:.1f} m/s", flush=True)
                if abs(miss) <= PITCH_MATCH_TOL_DEG:
                    break
                trim = round(trim + miss, 2)
            else:
                last = entry["rounds"][-1]
                raise FlightCheckFailed(f"FBWA did not fly the cruise pitch {want:.2f} deg in {PITCH_TRIM_ROUNDS} "
                                        f"trims (last {last['trim_deg']:g} deg: {last['pitch_deg']:.2f} deg)")
            entry["run_deg"] = trim
            if abs(fbwa["airspeed_m_s"] - pilot["cruise_airspeed_m_s"]) > CRUISE_SPEED_TOL_M_S:
                raise FlightCheckFailed(f"FBWA trimmed to {trim:g} deg flies {fbwa['airspeed_m_s']:.1f} m/s, not the "
                                        f"cruise {pilot['cruise_airspeed_m_s']:.1f} m/s")
        except BaseException:
            self._restore()
            raise
        return self

    def __exit__(self, kind: type[BaseException] | None, exc: BaseException | None,
                 trace: TracebackType | None) -> None:
        self._restore()

    def _restore(self) -> None:
        entry = self._record["pitch_trim"]
        if "restored" in entry:
            return
        try:
            self._vehicle.set_param(PITCH_TRIM, self.before)
            entry["restored"] = True
        except Exception as exc:     # noqa: BLE001 - the run's own failure, if any, stays the one raised
            entry["restored"] = False
            entry["restore_error"] = f"{type(exc).__name__}: {exc}"
