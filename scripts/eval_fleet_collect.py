"""Drain the fleet's one shared telemetry stream, routing to every aircraft.

`drain_position_messages` cannot serve a fleet: it filters the single
consumptive stream to ONE sysid and DISCARDS every other aircraft's messages,
so calling it per aircraft would starve all but one consumer. This drain
routes by srcSystem instead -- one typed drain for both message types,
because a second type-filtered drain would starve one consumer of the
other's messages (and the type filter must stay a list; pymavlink wraps a
tuple as a single element, which matches nothing).

Per aircraft the single-case semantics are kept exactly: scoring interval is
sticky from the first `scoring_active.marker` sighting; the EKF scorer and ground
track FREEZE at that aircraft's `result.json` (its episode is over --
post-pass trajectory could invalidate the run on post-episode maneuvers or
distort the wind classification); the truth recorder keeps receiving until
its post-CPA closure evidence is in, bounded by
`TRUTH_CLOSURE_DRAIN_TIMEOUT_S` from that aircraft's result, and then closes
AND finalizes, so a later re-approach cannot enter the truth track and the
trailing-freshness stamp lands at this aircraft's own episode end rather
than after the whole fleet has collected.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from scripts import eval_direct_pixel_pn as one  # noqa: E402
from scripts.eval_ground_track import GroundTrackRecorder  # noqa: E402
from scripts.eval_navigation_cases import CoordinateScorer  # noqa: E402
from scripts.eval_navigation_telemetry import (  # noqa: E402
    _DRAIN_MESSAGE_TYPES,
    position_sample_from_message,
)
from scripts.eval_navigation_truth import TruthRecorder  # noqa: E402


def collect_fleet(
    master: object,
    children: dict[int, object],
    directories: dict[int, Path],
    scorers: dict[int, CoordinateScorer],
    tracks: dict[int, GroundTrackRecorder],
    truths: dict[int, TruthRecorder],
    timeout_s: float,
) -> tuple[dict[int, dict[str, object]], dict[int, str]]:
    """Fly the fleet to its results, every consumer fed from the one stream.

    A dead or timed-out aircraft becomes a NAMED failure in the second
    return value, and the drain keeps flying everyone else. Raising here
    would discard the rest of the fleet's evidence for one aircraft's crash
    -- the single path salvages exactly this situation into an unscored
    verdict, and a fleet must not do worse times thirty-six. The failed
    aircraft's truth door closes at detection, so salvage scores what was
    actually observed.

    Once every aircraft holds a result or a failure, the loop runs PAST the
    overall deadline for the still-open truth closure windows -- each is
    bounded by `TRUTH_CLOSURE_DRAIN_TIMEOUT_S` from its own result, exactly
    the window the single case grants, so a result landing near the deadline
    is not silently stripped of its closure evidence.
    """
    results: dict[int, dict[str, object]] = {}
    failures: dict[int, str] = {}
    scoring_active = {sys_id: False for sys_id in children}
    truth_open = {sys_id: sys_id in truths for sys_id in children}
    closure_deadlines: dict[int, float] = {}
    deadline_s = time.monotonic() + timeout_s

    def close_truth(sys_id: int) -> None:
        # The door close IS this aircraft's episode end, so the finalize
        # stamp lands here -- deferring it to the fleet-wide verdict pass
        # makes trailing-freshness certification measure the fleet's
        # finish-time spread (2 of 3 aircraft failed the 2 s gate by ~12 s
        # on the first acceptance flight, their streams perfectly live).
        if truth_open[sys_id]:
            truth_open[sys_id] = False
            truths[sys_id].finalize()

    def aircraft_done(sys_id: int) -> bool:
        if sys_id in failures:
            return True
        return sys_id in results and not truth_open[sys_id]

    while not all(aircraft_done(sys_id) for sys_id in children):
        for sys_id in children:
            scoring_active[sys_id] = scoring_active[sys_id] or (
                directories[sys_id] / "scoring_active.marker"
            ).exists()
        for _ in range(500):
            message = master.recv_match(type=_DRAIN_MESSAGE_TYPES, blocking=False)
            if message is None:
                break
            sys_id = int(message.get_srcSystem())
            if sys_id not in children:
                continue
            if message.get_type() == "SIM_STATE":
                if truth_open[sys_id]:
                    truths[sys_id].add_message(
                        message, time.time(), scoring_active=scoring_active[sys_id]
                    )
                continue
            if sys_id in results or sys_id in failures or not scoring_active[sys_id]:
                continue
            scorers[sys_id].add(position_sample_from_message(message, time.time()))
            # Same message, velocity fields the scorer's PositionSample drops.
            tracks[sys_id].add(message)
        for sys_id, child in children.items():
            if sys_id in results or sys_id in failures:
                continue
            result_path = directories[sys_id] / "result.json"
            if result_path.exists():
                try:
                    results[sys_id] = json.loads(
                        result_path.read_text(encoding="utf-8")
                    )
                except (OSError, ValueError):
                    # The child publishes write-then-replace, but an
                    # unreadable file must still not lose the fleet: while
                    # the child lives it may complete on the next pass; once
                    # the child has exited it never will, so it becomes that
                    # aircraft's named failure.
                    if child.poll() is not None:
                        failures[sys_id] = (
                            f"unreadable result.json from exited child "
                            f"sysid={sys_id} (code={child.returncode})"
                        )
                        close_truth(sys_id)
                else:
                    closure_deadlines[sys_id] = (
                        time.monotonic() + one.TRUTH_CLOSURE_DRAIN_TIMEOUT_S
                    )
            elif child.poll() is not None:
                failures[sys_id] = (
                    f"direct pixel child sysid={sys_id} exited "
                    f"code={child.returncode}"
                )
                close_truth(sys_id)
        if time.monotonic() >= deadline_s:
            for sys_id in children:
                if sys_id not in results and sys_id not in failures:
                    failures[sys_id] = (
                        f"no result.json within {timeout_s:g}s"
                    )
                    close_truth(sys_id)
        for sys_id in children:
            if sys_id in results and truth_open[sys_id] and (
                truths[sys_id].closure_ready()
                or time.monotonic() >= closure_deadlines[sys_id]
            ):
                close_truth(sys_id)
        time.sleep(0.01)
    return results, failures


__all__ = ["collect_fleet"]
