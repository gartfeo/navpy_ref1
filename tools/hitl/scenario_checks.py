"""The pre-series orbit check (hitl_final_approach.md section 9, stage 1 steps 23 and 23b; section 4 "Feasibility
contract"; hitl_dock_detector.md section 5).

Before a ten-run series, pure arithmetic on the HITL profile, the series' public half and the aircraft's inputs
decides whether the explicit HITL orbit (``final_approach_orbit.radius_m``, core/skills/acquire/
orbit_radius_setting.py) can be flown and the dock seen by the whole-dock detector. Any failed check refuses the
series (exit 1); so does any input it cannot read. The approach runner (tools/tuning/approach.py) runs it before each
run's folder exists and refuses the run on any failure. Nothing here reaches navpy: navpy gets only the profile
radius, and this check reads no dock pose and no seed (only the scenario's public lane, limits and dock size,
scenario_public.py).

Geometry: the aircraft circles at radius r, h = lane height - the POI's height (the dock's mid-height; the dock stands
on the ground), round a centre that may be off by e (the ground ray's bound, orbit_centre.py); the dock is then
between r - e and r + e away horizontally. The inputs are in orbit_plan.py.

Checks, each using the model speed (the run's commanded airspeed, inside the model's declared airspeed band) and
the profile's measured optics at the planned zoom step:

- model_speed: the commanded airspeed lies in the model input's ``envelope.airspeed_m_s`` band;
- turn_radius: r >= kappa v^2 / (g tan phi), the planner's turn term (orbit_geometry.py, kappa 2), phi the NAV
  law's bank cap (command_anchor.ROLL_LIMIT_CAP_DEG, 35 deg) or the aircraft's ROLL_LIMIT_DEG when smaller;
- descent: r >= H / tan(eta |PTCH_LIM_MIN_DEG|), the planner's descent term (eta 0.65), H the lane over the
  ground ray's height zero;
- detector: the dock's smaller box side in the detector's input pixels (full-frame pixels times its input scale),
  over dock yaws, at the farthest slant on the orbit (r + e) is at least the measured ``recall_min_box_px``; and the
  acquisition gate's floor ``min_box_px`` is not below it, so the first detection (at D_max, which sets e) is one
  the detector finds at its measured recall; and that floor is met at the search step at the radius or beyond, so
  the gate can acquire at all;
- containment: the dock's box at its largest yaw, centred, keeps the fixed-zoom policy's zoom-in margin on both axes
  at the nearest slant, so the policy reaches the planned step and holds it round the circle; and the next narrower
  step, if any, fails that margin at the farthest slant, so the policy does not zoom past it;
- recovery_floor: the lane (and so the search and the orbit, flown at it) is above the run's floor. The floor is
  released at the operator's confirm (option A, owner 2026-11-08: the dock stands on the ground), so the dock is no
  longer required above it; in the sim a miss after the confirm ends on the ground. Stage 1 only.

    python -s -m tools.hitl.scenario_checks --scenario DIR --params-file F.parm --model-input MI.json
        [--profile siyi_zr10_hitl_encoded] [--zoom 10.0]
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
from tools.hitl.orbit_centre import (
    DOCK_YAWS_DEG,
    acquirable_beyond_radius,
    centre_error_m,
    centre_error_terms,
    descent_height_m,
    dock_box_px,
    first_detection_distance_m,
    smaller_side_input_px,
)
from tools.hitl.orbit_plan import HITL_PROFILE, OrbitPlan, finite, height_above_poi, load_plan, planned_step

GRAVITY = 9.81


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: dict[str, Any]


def _slant(horizontal_m: float, h: float) -> float:
    return math.hypot(horizontal_m, h)


def _gaps(plan: OrbitPlan, step: ZoomStep, slant_m: float) -> tuple[float, float]:
    """Centred box gaps to the frame edges at the dock's worst yaw, as fractions of each axis (negative: overflow)."""
    width, height = plan.steps.stream.image_width, plan.steps.stream.image_height
    boxes = [dock_box_px(step, plan.dock, slant_m, height_above_poi(plan), yaw) for yaw in DOCK_YAWS_DEG]
    box_w, box_h = max(box[0] for box in boxes), max(box[1] for box in boxes)
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


def check_detector(plan: OrbitPlan) -> CheckResult:
    step, h, error = planned_step(plan), height_above_poi(plan), centre_error_m(plan)
    if step is None:
        return CheckResult("detector", False, {"zoom_step": "not a measured step"})
    far = _slant(plan.radius_m + error, h)
    side = min(smaller_side_input_px(plan, step, far, yaw) for yaw in DOCK_YAWS_DEG)
    limits = plan.detector
    gate_ok = limits.acq_min_box_px >= limits.recall_min_box_px
    acquirable = acquirable_beyond_radius(plan)
    return CheckResult("detector", side >= limits.recall_min_box_px and gate_ok and acquirable,
                       {"zoom_step": step.command, "centre_error_m": round(error, 2),
                        "centre_error_terms_m": {k: round(v, 2) for k, v in centre_error_terms(plan).items()},
                        "first_detection_m": round(first_detection_distance_m(plan), 1),
                        "acquirable_beyond_radius": acquirable,
                        "farthest_slant_m": round(far, 2), "smaller_side_input_px": round(side, 2),
                        "recall_min_box_px": limits.recall_min_box_px, "acq_min_box_px": limits.acq_min_box_px,
                        "evidence": limits.evidence})


def check_containment(plan: OrbitPlan) -> CheckResult:
    step, h, error = planned_step(plan), height_above_poi(plan), centre_error_m(plan)
    if step is None:
        return CheckResult("containment", False, {"zoom_step": "not a measured step"})
    near = _slant(max(plan.radius_m - error, 0.0), h)
    far = _slant(plan.radius_m + error, h)
    gaps = _gaps(plan, step, near)
    reaches = min(gaps) >= plan.margins.zoom_in
    narrower = plan.steps.narrower(step)
    past = None if narrower is None else _gaps(plan, narrower, far)
    stays = past is None or min(past) < plan.margins.zoom_in
    return CheckResult("containment", reaches and stays,
                       {"zoom_step": step.command, "nearest_slant_m": round(near, 2),
                        "gaps_at_nearest": [round(g, 4) for g in gaps], "zoom_in_margin": plan.margins.zoom_in,
                        "narrower_step": None if narrower is None else narrower.command,
                        "narrower_gaps_at_farthest": None if past is None else [round(g, 4) for g in past]})


def check_recovery_floor(plan: OrbitPlan) -> CheckResult:
    passed = height_above_poi(plan) > 0.0 and plan.lane_height_m > plan.floor_m
    return CheckResult("recovery_floor", passed,
                       {"floor_m": plan.floor_m, "lane_height_m": plan.lane_height_m,
                        "lane_above_floor_m": round(plan.lane_height_m - plan.floor_m, 2),
                        "dock_placement": "ground", "floor_released_at": "operator confirm"})


CHECKS = (check_model_speed, check_turn_radius, check_descent, check_detector, check_containment,
          check_recovery_floor)


def _usable(plan: OrbitPlan) -> bool:
    dock, limits = plan.dock, plan.detector
    numbers = (plan.radius_m, plan.lane_height_m, dock.length_m, dock.width_m, dock.height_m, plan.floor_m,
               plan.airspeed_m_s, plan.min_pitch_deg, plan.roll_limit_deg, limits.recall_min_box_px,
               limits.acq_min_box_px)
    if not finite(*numbers):
        return False
    positive = (plan.radius_m, plan.airspeed_m_s, dock.length_m, dock.width_m, dock.height_m,
                limits.recall_min_box_px, limits.acq_min_box_px, height_above_poi(plan))
    return min(positive) > 0.0


def run_checks(plan: OrbitPlan) -> list[CheckResult]:
    """Every check's result; a plan with a non-finite or non-positive value fails them all."""
    if not _usable(plan):
        reason = {"input": "non-finite value, non-positive radius, airspeed, dock size or detector limit, "
                           "or the lane not above the dock"}
        return [CheckResult(check.__name__.removeprefix("check_"), False, reason) for check in CHECKS]
    return [check(plan) for check in CHECKS]


def feasible(results: Sequence[CheckResult]) -> bool:
    return bool(results) and all(result.passed for result in results)


def report(results: Sequence[CheckResult]) -> dict[str, Any]:
    return {"feasible": feasible(results), "checks": [{"name": r.name, "passed": r.passed, **r.detail}
                                                      for r in results]}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.scenario_checks",
                                     description="Refuse a series whose HITL orbit cannot be flown or its dock seen.")
    parser.add_argument("--scenario", required=True, help="a prepared scenario folder (its public half only)")
    parser.add_argument("--params-file", required=True, help="the run's aircraft parameter file")
    parser.add_argument("--model-input", required=True, help="the model input declaration (its envelope)")
    parser.add_argument("--profile", default=HITL_PROFILE)
    parser.add_argument("--zoom", default=10.0, type=float, help="the planned zoom step (absolute zoom command)")
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
