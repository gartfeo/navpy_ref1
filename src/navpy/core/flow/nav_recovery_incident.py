"""One NAV recovery incident's RTL deadline (docs/design/navpy_recovery.md, step 9).

An incident starts at its first hold and keeps that hold's deadline across re-holds, until NavPy has run healthy in
NORMAL for NAV_INCIDENT_CLEAR_S. So a fault that returns after every resume (a fresh approach whose entry always
fails) reaches RTL at the hold budget from the first hold, plus at most one resume and one hold trigger, instead of
starting a fresh deadline at each hold (Codex review of the step 9 fixes, round 2).
"""

from __future__ import annotations

#: Healthy time in NORMAL that ends an incident: as long as the hold's own budget (NAV_HOLD_RECOVERY_S), so an incident
#: is over when NavPy has run healthy for as long as it was allowed to hold.
NAV_INCIDENT_CLEAR_S = 30.0


class RecoveryIncident:
    def __init__(self, hold_budget_s: float) -> None:
        self._hold_budget_s = hold_budget_s
        self._since: float | None = None        # the incident's first hold
        self._holding = False

    def hold(self, now: float) -> None:
        if self._since is None:
            self._since = now
        self._holding = True

    def reopened(self) -> None:
        self._holding = False

    def expired(self, now: float) -> bool:
        """The hold budget, counted from the incident's first hold, is spent."""
        return self._holding and self._since is not None and now - self._since >= self._hold_budget_s

    def healthy(self, good_since: float | None, now: float) -> None:
        """In NORMAL: healthy since ``good_since``; long enough ends the incident."""
        if self._since is not None and good_since is not None and now - good_since >= NAV_INCIDENT_CLEAR_S:
            self._since = None


__all__ = ["NAV_INCIDENT_CLEAR_S", "RecoveryIncident"]
