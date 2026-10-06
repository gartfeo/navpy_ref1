#!/usr/bin/env python
"""SCRATCH: launch SITL, fly ONE guided scoring interval, tear it down, report.

The parent to `scratch_navigation_uav.py`, built the same way as the plant
harness parent (`scratch_sitl_scale.py`) and reusing its claim, cleanup and
wait helpers rather than copying them -- those helpers carry the reasons a
naive version is unsafe on a shared box, and a second copy would drift.

WHAT IT DOES AND DOES NOT DO
----------------------------
It launches, waits, tabulates and tears down. It never touches a vehicle
itself: everything the aircraft is told comes from the child process, which
owns the connection. Teardown kills exactly the sysids this run launched,
plus its own router -- never broad-kills, because this box runs other
sessions' SITL.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from scripts import scratch_navigation_arms as arms_matrix  # noqa: E402
# Cells and arms are one allocation concern and live together.
from scripts.scratch_navigation_cells import (  # noqa: E402,F401
    Cell,
    parse_cells,
)
import scratch_navigation_uav as navigation  # noqa: E402
# Re-exported: the child-invocation helpers are one concern, owned by
# `scratch_navigation_launch`, and reached through it rather than copied.
from scripts.scratch_navigation_launch import (  # noqa: E402,F401
    TEARDOWN_MARGIN_S,
    _child_args,
    _child_script,
    _child_source_path,
    _child_timeout_s,
    _row,
    _spawn,
)
import swarm_run_wsl as wsl  # noqa: E402
from gcs.backend import instance_ports as ip  # noqa: E402
from scratch_sitl_scale import (  # noqa: E402
    _await_children,
    _claim_span,
    _cleanup_sysids,
    _release_span,
)
from scratch_sitl_uav import (  # noqa: E402
    CLIMB_LIMIT_S,
    WALL_GUARD_FACTOR,
    WALL_GUARD_FLOOR_S,
)

# The child's own argparse defaults, read from its parser rather than retyped.
_CHILD_DEFAULTS = {
    action.dest: action.default
    for action in navigation._parser()._actions
}
CONNECT_TIMEOUT_S = float(_CHILD_DEFAULTS["connect_timeout"])
READY_TIMEOUT_S = float(_CHILD_DEFAULTS["ready_timeout"])

# The launcher's own default start point, from run_swarm.sh:103. Stated rather
# than inherited: the default has been seen to shift with the slot index, and
# this harness places the target relative to where the aircraft actually is, so
# a start point that moved between runs would silently change the scoring interval.
DEFAULT_HOME = "40.3117414,44.455211099999985,1294.86,0.0"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chat", type=int, default=None,
                        help="chat slot to borrow; defaults to the eval band floor")
    parser.add_argument("--instances", type=int, default=1)
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument("--distance", type=int, default=100)
    parser.add_argument("--home", default=DEFAULT_HOME)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--child-timeout", type=float, default=None,
        help="wall seconds before the child is killed. Default: derived from "
             "the child's own limits, the only value that cannot be wrong",
    )
    # Passed straight through to the child.
    parser.add_argument("--target-range-m", type=float, default=3000.0)
    parser.add_argument("--target-below-m", type=float, default=350.0)
    parser.add_argument("--target-off-boresight-deg", type=float, default=0.0)
    parser.add_argument("--settle-s", type=float, default=20.0)
    parser.add_argument("--climb-to-m", type=float, default=400.0)
    parser.add_argument("--level-settle-s", type=float, default=0.0)
    parser.add_argument("--engage-s", type=float, default=180.0, dest='scoring_duration_s')
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--wind-speed", type=float, default=None)
    parser.add_argument("--wind-dir", type=float, default=0.0)
    parser.add_argument(
        "--cells", default=None,
        help="PER-AIRCRAFT cells, so one launch flies a whole matrix instead "
             "of one condition. Format: 'name:wind_speed:wind_dir[,...]', "
             "repeated across the aircraft in order and cycled to fill "
             "--instances (e.g. 'calm:0:0,cross:8:90,tail:8:180' with "
             "--instances 12 gives 4 replicates of each). Every aircraft in "
             "a launch then SHARES its batch conditions, which is what makes "
             "the comparison paired instead of sequential -- single-batch "
             "sequential comparison was measured at up to 5.5x spread on "
             "identical code, so it cannot resolve anything smaller.",
    )
    parser.add_argument(
        "--arms", default=None,
        help="TWO NAVIGATION LAWS IN ONE LAUNCH. Format: "
             "'name=source_root,...', where each root is a full tree holding "
             "both src/ and scripts/. Arms are crossed with --cells, so every "
             "arm flies in every condition, and each arm's children run from "
             "their own tree with --expect-law-source pointing at it. The "
             "trees are verified identical except the navigation law before "
             "anything launches; see scratch_navigation_arms.",
    )
    parser.add_argument(
        "--cell-rotate", type=int, default=0,
        help="Rotate the cell list before assignment, so a repeat launch flies "
             "every treatment from different aircraft slots. Without it a "
             "treatment stays pinned to the same sysids and its effect cannot "
             "be told apart from a position effect, however many replicates.",
    )
    parser.add_argument(
        "--arm-rotate", type=int, default=0,
        help="The same, for arms: a repeat launch at a different rotation "
             "re-tests the comparison against a different arm-to-aircraft "
             "mapping, which is what separates a real effect from a "
             "launch-position one.",
    )
    parser.add_argument(
        "--expect-law-source", type=Path, default=None,
        help="ARM PROOF: source root every child MUST import the navigation law "
             "from, refused at startup rather than recorded at scoring time. "
             "Needed because a child rebuilds sys.path from its own location, "
             "so an arm selected by an alternate source tree is discarded "
             "silently and both arms fly identical code -- reporting a true "
             "difference of zero that reads as 'no effect'.",
    )
    parser.add_argument(
        "--estimate-source", choices=navigation.ESTIMATE_SOURCES,
        default="attitude",
        help="which pitch/roll de-rotates the camera ray; see the child",
    )
    parser.add_argument("--estimate-delay-poses", type=int, default=0)
    parser.add_argument("--estimate-lag-s", type=float, default=0.0)
    parser.add_argument("--estimate-jitter-deg", type=float, default=0.0)
    parser.add_argument("--estimate-jitter-seed", type=int, default=0)
    return parser


def assign_cells(
    cells: tuple[Cell, ...], sysids: list[int]
) -> dict[int, Cell | None]:
    """Cycle the cells across the aircraft, so replicates interleave.

    Interleaving matters: consecutive sysids share launch order and start
    within the same seconds, so cycling spreads each cell evenly through
    whatever ordering effects the launch has instead of grouping one cell at
    the front.

    Arms cross with the cells rather than cycling beside them -- see
    `scratch_navigation_arms.assign_cases`, which owns the allocation whenever
    arms are in play and reduces to this when they are not.
    """
    return {
        sysid: case.cell
        for sysid, case in arms_matrix.assign_cases((), cells, sysids).items()
    }


def main() -> int:
    options = _parser().parse_args()
    # Cells FIRST: they change what each child is launched with, so the
    # timeout cannot be derived before they are known.
    cells = parse_cells(options.cells)
    if cells and options.cell_rotate:
        shift = options.cell_rotate % len(cells)
        cells = cells[shift:] + cells[:shift]
    # Arms are verified BEFORE the launch claims ports or starts aircraft:
    # every way two trees can lie yields a full set of plausible results, so
    # ahead of the flying is the only place the check is worth anything.
    arms = arms_matrix.parse_arms(options.arms)
    arms_matrix.check_balanced(arms, cells, options.instances)
    arm_digests = arms_matrix.verify_arm_trees(arms) if arms else {}
    if options.child_timeout is None:
        options.child_timeout = _child_timeout_s(options, cells, arms)
    chat = ip.eval_band()[0] if options.chat is None else options.chat
    offset = ip.VEHICLES_PER_CHAT * chat
    sysids = [offset + index for index in range(1, options.instances + 1)]
    cases = arms_matrix.assign_cases(arms, cells, sysids, options.arm_rotate)
    cells_by_sysid = {sysid: case.cell for sysid, case in cases.items()}
    arms_by_sysid = {sysid: case.arm for sysid, case in cases.items()}
    root = WORKTREE / ".sitl-runs" / f"navigation-{time.strftime('%Y%m%d-%H%M%S')}"

    win_ports = ",".join(str(port) for port in ip.router_win_ports(chat))
    command = wsl.launch_command(
        options.speedup, offset, win_ports, options.instances,
        options.distance, home_coords=options.home,
    )
    print(f"=== navigation: sysids {sysids[0]}..{sysids[-1]} chat {chat} "
          f"speedup {options.speedup:g} ===", flush=True)
    print(f"  home {options.home}", flush=True)
    print(f"  artifacts {root}", flush=True)
    print(f"  child timeout {options.child_timeout:.0f}s", flush=True)

    # Ownership BEFORE any pkill, and held for the whole run. Teardown kills by
    # sysid and does not ask who started the process, so running without the
    # claim would risk taking down another session's aircraft.
    held = _claim_span(sysids)
    swarm: subprocess.Popen[bytes] | None = None
    results: dict[int, dict] = {}
    try:
        _cleanup_sysids(chat, sysids)
        swarm = subprocess.Popen(
            wsl.wsl_argv(command),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        children = {
            sysid: _spawn(options, root / f"uav-{sysid}", sysid,
                          cells_by_sysid[sysid], arms_by_sysid[sysid])
            for sysid in sysids
        }
        late = _await_children(children, options.child_timeout)
        for sysid in sysids:
            path = root / f"uav-{sysid}" / "result.json"
            try:
                results[sysid] = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as error:
                # One aircraft's failure, not the run's. Letting the decode
                # error propagate would discard every other aircraft's result.
                results[sysid] = {
                    "sysid": sysid,
                    "errors": [f"no readable result: {type(error).__name__}: "
                               f"{error} (exit {children[sysid].returncode})"],
                }
        if late:
            for sysid in late:
                results[sysid].setdefault("errors", []).append(
                    f"child had to be killed after {options.child_timeout}s")
    finally:
        # Teardown runs whatever happened, including a crash above, and only
        # ever touches the aircraft this run launched.
        _cleanup_sysids(chat, sysids)
        if swarm is not None and swarm.poll() is None:
            swarm.terminate()
        _release_span(held)

    print("=== results ===", flush=True)
    for sysid in sysids:
        print(_row(sysid, results[sysid], cells_by_sysid[sysid],
                   arms_by_sysid[sysid]), flush=True)
        for error in results[sysid].get("errors") or []:
            print(f"      ! {error}", flush=True)

    summary = {
        "cells": {
            str(sysid): None if cell is None else cell.name
            for sysid, cell in cells_by_sysid.items()
        },
        # The allocation and the treatment identity, so the comparison can be
        # checked rather than trusted: which aircraft flew which arm, and the
        # hash of each arm's law. A summary that names arms without naming
        # their contents cannot rule out two arms having run the same code.
        "arms": {
            str(sysid): None if arm is None else arm.name
            for sysid, arm in arms_by_sysid.items()
        },
        "arm_sources": {arm.name: str(arm.source_root) for arm in arms},
        "arm_law_sha256": arm_digests,
        "arm_rotate": options.arm_rotate,
        "chat": chat, "sysids": sysids, "home": options.home,
        "speedup": options.speedup, "root": str(root),
        "per_uav": {str(sysid): results[sysid] for sysid in sysids},
    }
    root.mkdir(parents=True, exist_ok=True)
    (root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    if options.out is not None:
        options.out.parent.mkdir(parents=True, exist_ok=True)
        options.out.write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    failed = [s for s in sysids if results[s].get("errors")]
    print(f"\n{len(sysids) - len(failed)}/{len(sysids)} clean", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
