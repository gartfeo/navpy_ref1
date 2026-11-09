"""A standing review: a POI only the operator's confirmation ends (docs/design/navpy_recovery.md, step 10, item 3).

Owner rulings 2026-11-09: parking at the mission's return dock waits for the operator, with no timeout and no cancel.
For such a POI, while a network is connected:
- auto-confirm is off (the coordinator asks; the final-approach path does not confirm locally);
- the round has no deadline: the request is resent every interval until the operator answers or the review is
  withdrawn (the dock out of view: the acquisition gates withdraw it, and it is asked again once they hold);
- a negative answer (deny, cancel, a card's timeout) is ignored;
- CONFIRM has no maximum dwell and no re-acquire abort, so it never tears down to search and never registers a
  cooldown (core/skills/confirm/poi_rejection_exit.py).
With no network the POI is confirmed as before (``network is None``): a UAV with no link must not circle until its
battery fails. The mission module says which POI it is (the home dock it parks at).
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.ports.detect_data import DetectedObject


class StandingReview:
    """The mission module's check, set once at composition; none set: no POI stands."""

    def __init__(self) -> None:
        self._check: Callable[[DetectedObject], bool] = lambda _poi: False

    def set_check(self, check: Callable[[DetectedObject], bool]) -> None:
        self._check = check

    def __call__(self, poi: DetectedObject | None) -> bool:
        return poi is not None and bool(self._check(poi))


__all__ = ["StandingReview"]
