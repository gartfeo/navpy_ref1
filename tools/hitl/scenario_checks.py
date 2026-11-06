"""The pre-series orbit check (hitl_final_approach.md section 9, stage 1 step 23; section 4 "Feasibility contract").

Before a ten-run series, pure arithmetic on the HITL profile, the series' public half and the aircraft's inputs
decides whether the explicit HITL orbit (``final_approach_orbit.radius_m``, core/skills/acquire/
orbit_radius_setting.py) can be flown and read. Any failed check refuses the series (exit 1); so does any input it
cannot read. The approach runner (tools/tuning/approach.py) runs it before each run's folder exists and refuses
the run on any failure. Nothing here reaches navpy: navpy gets only the profile radius, and this check reads no dock
pose and no seed (only the scenario's public lane and limits, scenario_public.py).

Geometry: the aircraft circles at radius r, h = lane height - board height above home, round a centre that may be
off by e (orbit_centre.py); the board is then between r - e and r + e away horizontally. The inputs are in
orbit_plan.py.

Checks, each using the model speed (the run's commanded airspeed, inside the model's declared airspeed band) and
the profile's measured optics at the planned zoom step:

- model_speed: the commanded airspeed lies in the model input's ``envelope.airspeed_m_s`` band;
- turn_radius: r >= kappa v^2 / (g tan phi), the planner's turn term (orbit_geometry.py, kappa 2), phi the NAV
  law's bank cap (command_anchor.ROLL_LIMIT_CAP_DEG, 35 deg) or the aircraft's ROLL_LIMIT_DEG when smaller;
- descent: r >= H / tan(eta |PTCH_LIM_MIN_DEG|), the planner's descent term (eta 0.65), H the lane over the
  orbit centre (the board for an accepted pose; the ground ray's height zero otherwise, so H is the lane height);
- reader: at the farthest slant R the board's squares along the line of sight are fy * square * sin(d) / R pixels,
  sin(d) = h / R, at least the reader limit;
- containment: the board's box (its ChArUco grid at any yaw: the diagonal across, the diagonal times sin(d) along),
  centred, keeps the fixed-zoom policy's zoom-in margin on both axes at the nearest slant, so the policy reaches the
  planned step and holds it round the circle; and the next narrower step, if any, fails that margin at the farthest
  slant, so the policy does not zoom past it;
- recovery_floor: the lane flies above the run's floor, and the board sits at least the pull-out allowance above it,
  so the floor's recovery cannot trip before contact.

    python -s -m tools.hitl.scenario_checks --scenario DIR --params-file F.parm --model-input MI.json
        --board-height-m H [--profile siyi_zr10_hitl_encoded] [--zoom 10.0] [--reader-min-square-px 11.5]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from navpy.core.perception.zoom.fixed_zoom_steps import ZoomStep
from navpy.core.skills.acquire.orbit_geometry import OrbitNavigationLimits
from navpy.plugins.laws.vision_pn.command_anchor import ROLL_LIMIT_CAP_DEG
from tools.hitl.orbit_centre import centre_error_m, descent_height_m
from tools.hitl.orbit_plan import (
    HITL_PROFILE,
    READER_MIN_SQUARE_PX,
    Board,
    OrbitPlan,
    finite,
    height_above_board,
    load_plan,
    planned_step,
)

GRAVITY = 9.81
# Plan value (section 6, from the pickup plan): a 20-30 deg dive loses 4-9 m in its pull-out; unmeasured (H2).
PULL_OUT_ALLOWANCE_M = 9.0


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: dict[str, Any]


def _slant(horizontal_m: float, h: float) -> float:
    return math.hypot(horizontal_m, h)


def _gaps(step: ZoomStep, board: Board, slant_m: float, h: float, width: int, height: int) -> tuple[float, float]:
    """Centred box gaps to the frame edges, as fractions of each axis (negative: the box overflows)."""
    sin_d = h / slant_m
    box_w = board.diagonal_m * step.optics.fx / slant_m
    box_h = board.diagonal_m * sin_d * step.optics.fy / slant_m
    return (1.0 - box_w / width) / 2.0, (1.0 - box_h / height) / 2.0


def check_model_speed(plan: OrbitPlan) -> CheckResult:
    band = plan.airspeed_band
    passed = band is not None and band[0] <= plan.airspeed_m_s <= band[1]
    return CheckResult("model_speed", passed, {"airspeed_m_s": plan.airspeed_m_s,
                                               "model_band_m_s": None if band is None else list(band)})


def turn_bank_deg(plan: OrbitPlan) -> float:
    """The bank the orbit can use: the NAV law's cap, or the aircraft's limit (the planner's) when smaller."""
    return min(ROLL_LIMIT_CAP_DEG, plan.roll_limit_deg)


def check_turn_radius(plan: OrbitPlan) -> CheckResult:
    bank = turn_bank_deg(plan)
    kappa = OrbitNavigationLimits(plan.airspeed_m_s, bank, plan.min_pitch_deg).kappa
    usable = 0.0 < bank < 90.0
    raw = plan.airspeed_m_s ** 2 / (GRAVITY * math.tan(math.radians(bank))) if usable else None
    need = None if raw is None else kappa * raw
    return CheckResult("turn_radius", need is not None and plan.radius_m >= need,
                       {"radius_m": plan.radius_m, "turn_radius_m": None if raw is None else round(raw, 2),
                        "kappa": kappa, "required_m": None if need is None else round(need, 2), "bank_deg": bank,
                        "bank_cap_deg": ROLL_LIMIT_CAP_DEG, "roll_limit_deg": plan.roll_limit_deg})


def check_descent(plan: OrbitPlan) -> CheckResult:
    eta = OrbitNavigationLimits(plan.airspeed_m_s, ROLL_LIMIT_CAP_DEG, plan.min_pitch_deg).eta
    gamma = eta * abs(plan.min_pitch_deg)
    h = descent_height_m(plan)
    passed = 0.0 < gamma < 90.0 and plan.radius_m >= h / math.tan(math.radians(gamma))
    need = h / math.tan(math.radians(gamma)) if 0.0 < gamma < 90.0 else None
    return CheckResult("descent", passed, {"descent_height_m": h, "min_pitch_deg": plan.min_pitch_deg,
                                           "eta": eta, "required_m": None if need is None else round(need, 2)})


def check_reader(plan: OrbitPlan) -> CheckResult:
    step, h, error = planned_step(plan), height_above_board(plan), centre_error_m(plan)
    far = _slant(plan.radius_m + error, h)
    if step is None:
        return CheckResult("reader", False, {"zoom_step": "not a measured step"})
    px = step.optics.fy * plan.board.square_m * (h / far) / far
    return CheckResult("reader", px >= plan.reader_min_square_px,
                       {"zoom_step": step.command, "fy_px": step.optics.fy, "centre_error_m": round(error, 2),
                        "farthest_slant_m": round(far, 2), "square_px_along_los": round(px, 2),
                        "reader_min_square_px": plan.reader_min_square_px})


def check_containment(plan: OrbitPlan) -> CheckResult:
    step, h, error = planned_step(plan), height_above_board(plan), centre_error_m(plan)
    if step is None:
        return CheckResult("containment", False, {"zoom_step": "not a measured step"})
    width, height = plan.steps.stream.image_width, plan.steps.stream.image_height
    near = _slant(max(plan.radius_m - error, 0.0), h)
    far = _slant(plan.radius_m + error, h)
    gaps = _gaps(step, plan.board, near, h, width, height)
    reaches = min(gaps) >= plan.margins.zoom_in
    narrower = plan.steps.narrower(step)
    past = None if narrower is None else _gaps(narrower, plan.board, far, h, width, height)
    stays = past is None or min(past) < plan.margins.zoom_in
    return CheckResult("containment", reaches and stays,
                       {"zoom_step": step.command, "nearest_slant_m": round(near, 2),
                        "gaps_at_nearest": [round(g, 4) for g in gaps], "zoom_in_margin": plan.margins.zoom_in,
                        "narrower_step": None if narrower is None else narrower.command,
                        "narrower_gaps_at_farthest": None if past is None else [round(g, 4) for g in past]})


def check_recovery_floor(plan: OrbitPlan) -> CheckResult:
    board_clearance = plan.board_height_m - plan.floor_m
    passed = (height_above_board(plan) > 0.0 and plan.lane_height_m > plan.floor_m
              and board_clearance >= PULL_OUT_ALLOWANCE_M)
    return CheckResult("recovery_floor", passed,
                       {"floor_m": plan.floor_m, "lane_height_m": plan.lane_height_m,
                        "board_height_m": plan.board_height_m, "board_above_floor_m": board_clearance,
                        "pull_out_allowance_m": PULL_OUT_ALLOWANCE_M})


CHECKS = (check_model_speed, check_turn_radius, check_descent, check_reader, check_containment,
          check_recovery_floor)


def run_checks(plan: OrbitPlan) -> list[CheckResult]:
    """Every check's result; a plan with a non-finite or non-positive value fails them all."""
    numbers = (plan.radius_m, plan.lane_height_m, plan.board_height_m, plan.floor_m, plan.airspeed_m_s,
               plan.min_pitch_deg, plan.roll_limit_deg, plan.reader_min_square_px)
    if not finite(*numbers) or plan.radius_m <= 0.0 or plan.airspeed_m_s <= 0.0 or height_above_board(plan) <= 0.0:
        reason = {"input": "non-finite value, non-positive radius or airspeed, or the lane not above the board"}
        return [CheckResult(check.__name__.removeprefix("check_"), False, reason) for check in CHECKS]
    return [check(plan) for check in CHECKS]


def feasible(results: Sequence[CheckResult]) -> bool:
    return bool(results) and all(result.passed for result in results)


def report(results: Sequence[CheckResult]) -> dict[str, Any]:
    return {"feasible": feasible(results), "checks": [{"name": r.name, "passed": r.passed, **r.detail}
                                                      for r in results]}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.scenario_checks",
                                     description="Refuse a series whose HITL orbit cannot be flown or read.")
    parser.add_argument("--scenario", required=True, help="a prepared scenario folder (its public half only)")
    parser.add_argument("--params-file", required=True, help="the run's aircraft parameter file")
    parser.add_argument("--model-input", required=True, help="the model input declaration (its envelope)")
    parser.add_argument("--board-height-m", required=True, type=float,
                        help="the board's design height above home (m)")
    parser.add_argument("--profile", default=HITL_PROFILE)
    parser.add_argument("--zoom", default=10.0, type=float, help="the planned zoom step (absolute zoom command)")
    parser.add_argument("--reader-min-square-px", default=READER_MIN_SQUARE_PX, type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        plan = load_plan(args)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({"feasible": False, "refused": str(error)}, indent=2))
        return 1
    out = report(run_checks(plan))
    print(json.dumps(out, indent=2))
    return 0 if out["feasible"] else 1


if __name__ == "__main__":
    sys.exit(main())
