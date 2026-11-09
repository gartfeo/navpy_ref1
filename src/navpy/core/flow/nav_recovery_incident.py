"""One NAV recovery incident's hold budget (docs/design/navpy_recovery.md, step 9).

The RTL budget (NAV_HOLD_RECOVERY_S) is time spent holding (HOLD and RESUMING), summed across the incident's holds,
in recovery's clock (aircraft time). An incident starts at its first hold and ends when NavPy has run healthy in
NORMAL for NAV_INCIDENT_CLEAR_S; then the sum starts again from zero.

- A single hold reaches RTL at the budget, as one long hold always did.
- Two short, unrelated stalls (a host stall of 1.7 s, seen on all three NavPys at once in step 9) cost only their own
  hold time, however close together: no early RTL for the fleet.
- A fault that returns after every resume (a fresh approach whose entry always fails; Codex review of the step 9
  fixes, round 2) still reaches RTL: each hold lasts at least NAV_HOLD_TRIGGER_S of healthy cycles before it can
  resume, so it adds at least that much, and RTL comes after at most budget / trigger = 30 / 1.5 = 20 holds. Between
  them NavPy is in NORMAL for at most a hold trigger of failing cycles, so RTL comes at most
  budget + (budget / trigger) x trigger = 2 x budget, 60 s of aircraft time, after the first hold.
"""

from __future__ import annotations

#: Healthy time in NORMAL that ends an incident: as long as the hold's own budget (NAV_HOLD_RECOVERY_S), so an incident
#: is over when NavPy has run healthy for as long as it was allowed to hold.
NAV_INCIDENT_CLEAR_S = 30.0


class RecoveryIncident:
    def __init__(self, hold_budget_s: float) -> None:
        self._hold_budget_s = hold_budget_s
        self._active = False
        self._spent_s = 0.0           # time spent holding in this incident
        self._last_s: float | None = None

    def advance(self, holding: bool, now: float) -> None:
        """First in each recovery step: ``holding`` is whether recovery held since its previous step."""
        if holding and self._active and self._last_s is not None:
            self._spent_s += max(now - self._last_s, 0.0)
        self._last_s = now

    def hold(self) -> None:
        """A hold begins: the incident's first, or another within it."""
        self._active = True

    def expired(self) -> bool:
        """The incident's hold time has reached the budget."""
        return self._active and self._spent_s >= self._hold_budget_s

    def healthy(self, good_since: float | None, now: float) -> None:
        """In NORMAL: healthy since ``good_since``; long enough ends the incident."""
        if self._active and good_since is not None and now - good_since >= NAV_INCIDENT_CLEAR_S:
            self._active = False
            self._spent_s = 0.0


__all__ = ["NAV_INCIDENT_CLEAR_S", "RecoveryIncident"]
