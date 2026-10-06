"""How one aircraft is invoked, kept apart from how a launch is orchestrated.

The parent decides WHAT to fly -- slots, conditions, arms, teardown. This module
decides how a single child is called: its argv, the tree it imports from, its
timeout, and how its result reads back. The split is not cosmetic. Three of the
four have already produced a run with no result and no error:

  * an option the parent accepted and never forwarded, so the child silently
    used a default and the experiment looked like it ran;
  * a timeout derived from arguments the child was not actually given, so the
    parent killed it before its own deadline;
  * a source tree selected by environment alone, which the child discards when
    it rebuilds its import roots from its own location.

Each is a mistake about the CALL, not about the launch, and each is guarded
here.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import subprocess
import sys
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from scripts import scratch_navigation_arms as arms_matrix  # noqa: E402
import scratch_navigation_uav as navigation  # noqa: E402
from gcs.backend import instance_ports as ip  # noqa: E402
from scripts.scratch_navigation_cells import Cell  # noqa: E402

# Enough for the child to finish writing result.json after its own guard fires.
TEARDOWN_MARGIN_S = 60.0


def _child_args(options: argparse.Namespace, sysid: int,
                directory: Path, cell: Cell | None = None,
                arm: "arms_matrix.Arm | None" = None) -> list[str]:
    """The child's own arguments, without the interpreter or script path.

    Separated so the timeout can be computed by PARSING THE EXACT INVOCATION
    about to be launched, rather than by re-deriving it from the parent's
    options. Two hand-written budgets in a row were short.
    """
    args = [
        "--connection", ip.companion_device(sysid),
        "--sysid", str(sysid),
        "--out", str(directory / "result.json"),
        "--target-range-m", repr(options.target_range_m),
        "--target-below-m", repr(options.target_below_m),
        "--settle-s", repr(options.settle_s),
        "--climb-to-m", repr(options.climb_to_m),
        "--level-settle-s", repr(options.level_settle_s),
        "--engage-s", repr(options.scoring_duration_s),
        "--speedup", repr(options.speedup),
    ]
    # GEOMETRY and ENERGY, cell first. Both were launch-wide only, so an angle
    # sweep cost one launch per angle and every comparison across them carried
    # the batch spread this bench exists to avoid.
    args += ["--target-off-boresight-deg", repr(
        options.target_off_boresight_deg
        if (cell is None or cell.off_boresight_deg is None)
        else cell.off_boresight_deg)]
    args += ["--throttle", repr(
        options.throttle
        if (cell is None or cell.throttle is None) else cell.throttle)]
    # TRUTH-FED gain, cell only -- deliberately no launch-wide flag. A whole
    # launch of it would measure the ceiling against nothing, and the
    # comparison it exists for only means anything within one batch.
    if cell is not None and cell.oracle is not None:
        args += ["--oracle-gain", cell.oracle]
    # An arm's own tree wins over the launch-wide declaration: the whole point
    # of the arm is that its law lives somewhere else, and a launch-wide root
    # would refuse the very tree it was asked to fly.
    expected_source = (
        options.expect_law_source if arm is None else arm.source_path
    )
    if expected_source is not None:
        args += ["--expect-law-source", str(expected_source)]
    # Fidelity, timing and noise all come from the cell when it states them, so the sweep
    # flies per aircraft instead of one arm per launch. `None` means the cell
    # is silent and the launch-wide value stands; 0 is an explicit arm.
    args += ["--estimate-source",
             options.estimate_source if (cell is None or cell.source is None)
             else cell.source]
    delay = options.estimate_delay_poses if (
        cell is None or cell.delay_poses is None) else cell.delay_poses
    jitter = options.estimate_jitter_deg if (
        cell is None or cell.jitter_deg is None) else cell.jitter_deg
    lag_s = options.estimate_lag_s if (
        cell is None or cell.lag_s is None) else cell.lag_s
    if delay:
        args += ["--estimate-delay-poses", str(delay)]
    if lag_s:
        args += ["--estimate-lag-s", repr(lag_s)]
    scope = (None if cell is None else cell.lag_scope)
    if scope is not None:
        args += ["--estimate-lag-scope", scope]
    if jitter:
        # The requested seed OFFSET BY THE AIRCRAFT. Both halves are needed and
        # an earlier version dropped one: forwarding the requested seed alone
        # makes every replicate of a noise cell draw the SAME realisation, so
        # their agreement is zero by construction; forwarding the sysid alone
        # silently discards a requested seed, which is the "accepted but never
        # forwarded" failure this harness has a guard test for.
        args += ["--estimate-jitter-deg", repr(jitter),
                 "--estimate-jitter-seed",
                 str(options.estimate_jitter_seed + sysid)]
    # A per-aircraft cell overrides the launch-wide wind, so one launch can
    # fly every condition at once under shared batch conditions.
    wind_speed = options.wind_speed if cell is None else cell.wind_speed_mps
    wind_dir = options.wind_dir if cell is None else cell.wind_dir_deg
    if wind_speed is not None:
        args += ["--wind-speed", repr(wind_speed),
                 "--wind-dir", repr(wind_dir)]
    return args


def _child_script(arm: "arms_matrix.Arm | None") -> Path:
    """The arm's OWN child script, not this tree's.

    Both halves are needed and neither is enough. The child rebuilds its import
    roots from its own `__file__`, so this tree's script with the arm's
    PYTHONPATH imports THIS tree's law; and the arm's script with this tree's
    PYTHONPATH is the same mistake mirrored. Either way both arms fly identical
    code and the comparison reports a difference of zero.
    """
    return SCRIPTS / "scratch_navigation_uav.py" if arm is None else arm.child_script


def _child_source_path(arm: "arms_matrix.Arm | None") -> Path:
    """The source root the child imports from; the arm's own when it has one."""
    return WORKTREE / "src" if arm is None else arm.source_path


def _spawn(options: argparse.Namespace, directory: Path,
           sysid: int, cell: Cell | None = None,
           arm: "arms_matrix.Arm | None" = None) -> subprocess.Popen[bytes]:
    """One child for one aircraft, on its own connection."""
    directory.mkdir(parents=True, exist_ok=True)
    command = [
        str(options.python.resolve()), str(_child_script(arm)),
        *_child_args(options, sysid, directory, cell, arm),
    ]
    if cell is not None:
        # The WHOLE cell, not a summary of it. The name is arbitrary, and a
        # treatment axis missing from the artifact is an experiment that
        # cannot be told apart from its control afterwards.
        (directory / "cell.json").write_text(
            json.dumps(dataclasses.asdict(cell), indent=2, default=str),
            encoding="utf-8",
        )
    # Written before the process starts, so a run that dies early still says
    # exactly what it was asked to do.
    (directory / "child.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8")
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(_child_source_path(arm))
    environment["PYTHONUTF8"] = "1"
    return subprocess.Popen(
        command, cwd=str(WORKTREE), env=environment,
        stdout=(directory / "child.out.log").open("wb"),
        stderr=(directory / "child.err.log").open("wb"),
    )


def _child_timeout_s(
    options: argparse.Namespace,
    cells: tuple[Cell, ...] | None = None,
    arms: "tuple[arms_matrix.Arm, ...] | None" = None,
) -> float:
    """The child's own worst case, plus room to write its result.

    ASKED OF THE CHILD, on the exact arguments it is about to be launched with.
    Three hand-written or under-scoped budgets have each cost a full run:

      * the first hardcoded 900 s, below the child's own hung-run guard, so the
        parent killed a child that had not yet reached its own deadline;
      * the second derived it but budgeted the climb in AIRCRAFT-seconds and
        omitted the parameter, stream, mode and arming waits -- 2480 s against
        a legitimate 2525 s;
      * the third asked the child about a CELL-LESS invocation after
        `--cells` made the arguments differ per aircraft: a windy cell adds
        two parameter waits the budget never saw, 2705 s against a legitimate
        2725 s.

    Each produced a run with no result and no reason. So the number comes from
    the child, and it is asked once per DISTINCT invocation this launch will
    actually make -- the largest wins, because one timeout governs them all.
    """
    # Crossed, not two separate loops, because that is what the launch does.
    # Following the rule literally is the point: the moment an arm gains a
    # timed option, a per-cell-only budget becomes the fourth short budget.
    invocations = [
        (cell, arm)
        for cell in (list(cells or ()) or [None])
        for arm in (list(arms or ()) or [None])
    ]
    return max(
        navigation.worst_case_wall_s(
            navigation._parser().parse_args(
                # Any sysid and directory will do: neither is a timeout.
                _child_args(options, 1, Path("."), cell, arm)
            )
        )
        for cell, arm in invocations
    ) + TEARDOWN_MARGIN_S


def _row(sysid: int, result: dict, cell: "Cell | None" = None,
         arm: "arms_matrix.Arm | None" = None) -> str:
    miss = result.get("miss_m")
    parts = [part.name for part in (arm, cell) if part is not None]
    label = f"[{'/'.join(parts)}] " if parts else ""
    return (
        f"  uav-{sysid}: {label}end={result.get('engage_end')!s:<8}"
        f" miss={'--' if miss is None else f'{miss:.2f} m':>10}"
        f" cmds={result.get('commands')}"
        f" rejected={result.get('rejected_frames')}"
        f" saturated={result.get('saturated_commands')}"
        f" reversals={result.get('roll_reversals')}"
        f" engage={result.get('engage_s')}s"
        f" clock={result.get('measured_speedup')}x"
        f" errors={len(result.get('errors') or [])}"
    )


