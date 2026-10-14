"""The fleet's two latches: the boot barrier and the first-SNAP terminal tick.

Boot barrier (``lockstep_fleet.md``, "Boot barrier"): ``helpers_quiet`` is
latched per SITL, the first frame that reports it. A late latch only holds
admission: the barrier completes at the tick of the last latch, which is
still a boot tick, and the initial participants are admitted at the tick
after it. If the barrier is still open once frame k is ``boot_budget_us``
of sim time past frame 1, the run ends with ``CoordinatorFault("boot_barrier")``.

Terminal tick (``lockstep_fleet.md``, "Terminal tick"): the expected fleet
comes from the scenario, never from the live set. Each expected companion's
first terminal marker is latched as ``(ordinal, generation, tick)``; a later
one, or another generation's, changes nothing. The mission is complete at
the tick the last expected companion latches: k_T is the maximum of the
first-SNAP ticks. An expected companion never spawned, or retired without a
SNAP, never latches, so the mission cannot complete ``ok``.
"""

from __future__ import annotations

from collections.abc import Iterable

from navpy.sim.lockstep.coordinator_types import CoordinatorFault


class BootLatch:
    def __init__(self, sitls: int, budget_us: int | None) -> None:
        self._latched: list[int | None] = [None] * sitls
        self._budget_us = budget_us
        self._first_us: int | None = None

    @property
    def complete(self) -> bool:
        return all(tick is not None for tick in self._latched)

    @property
    def ticks(self) -> list[int | None]:
        """Each SITL's latch tick, in SITL order (None: not yet)."""
        return list(self._latched)

    def observe(self, k: int, t_us: int, quiet: Iterable[bool]) -> list[int]:
        """Frame k's ``helpers_quiet`` per SITL; returns the SITLs (0-based) latched now."""
        if self._first_us is None:
            self._first_us = t_us
        latched = []
        for index, flag in enumerate(quiet):
            if flag and self._latched[index] is None:
                self._latched[index] = k
                latched.append(index)
        if (not self.complete and self._budget_us is not None
                and t_us - self._first_us >= self._budget_us):
            waiting = [index + 1 for index, tick in enumerate(self._latched) if tick is None]
            raise CoordinatorFault("boot_barrier", f"tick {k}: SITLs {waiting} not quiet within "
                                   f"{self._budget_us} us of boot", tick=k, context={"waiting": waiting})
        return latched


class SnapLatch:
    def __init__(self, expected: tuple[int, ...]) -> None:
        self._expected = tuple(expected)
        #: ordinal -> (generation, tick) of its first terminal marker.
        self.first: dict[int, tuple[int, int]] = {}

    def observe(self, k: int, snaps: Iterable[tuple[int, int]]) -> None:
        """This tick's terminal markers, as ``(ordinal, generation)`` in ``DONE`` order."""
        for ordinal, generation in snaps:
            if ordinal in self._expected and ordinal not in self.first:
                self.first[ordinal] = (generation, k)

    @property
    def complete(self) -> bool:
        return bool(self._expected) and all(ordinal in self.first for ordinal in self._expected)

    @property
    def terminal_tick(self) -> int:
        """k_T: the latest first-SNAP tick of the expected fleet (only once complete)."""
        assert self.complete
        return max(tick for _generation, tick in self.first.values())

    def records(self) -> list[list[int]]:
        """``[ordinal, generation, tick]`` per latched companion, in ordinal order."""
        return [[ordinal, *self.first[ordinal]] for ordinal in sorted(self.first)]


__all__ = ["BootLatch", "SnapLatch"]
