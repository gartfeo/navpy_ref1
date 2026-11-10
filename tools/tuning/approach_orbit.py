"""The approach runner's orbit gate (hitl_final_approach.md steps 23 and 23b): refuse a run whose HITL orbit cannot be
flown or its dock seen, before the record or the run folder exists (tools/hitl/scenario_checks.py)."""
from __future__ import annotations

import argparse
from pathlib import Path

from tools.hitl import orbit_plan, scenario_checks
from tools.hitl.json_checks import read_json
from tools.tuning.approach_chain import VISION_PROFILE
from tools.tuning.hitl import board_param_files
from tools.tuning.hitl_params import merge_param_files


def orbit_check(args: argparse.Namespace, scenario: dict) -> dict:
    """The check's report on the scenario's public half, the board's merged parameter files, the model input's
    envelope (the dock's size is in the public half); SystemExit on a failed check or an unreadable input."""
    try:
        params, _ = merge_param_files(board_param_files([], [Path(p) for p in args.params_file or []], True, False,
                                                        args.hitl_data))
        plan = orbit_plan.plan_for(scenario, params, read_json(Path(args.model_input)),
                                   profile_name=VISION_PROFILE)
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"error: the orbit check cannot run: {error}") from error
    result = scenario_checks.report(scenario_checks.run_checks(plan))
    if not result["feasible"]:
        failed = ", ".join(check["name"] for check in result["checks"] if not check["passed"])
        raise SystemExit(f"error: the HITL orbit is infeasible ({failed}): see tools/hitl/scenario_checks.py")
    return result
