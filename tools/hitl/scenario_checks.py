"""The pre-series orbit check (hitl_final_approach.md section 9, stage 1 step 23; section 4 "Feasibility contract").

Before a ten-run series, pure arithmetic on the HITL profile, the series' public half and the aircraft's inputs
decides whether the explicit HITL orbit (``final_approach_orbit.radius_m``, core/skills/acquire/
orbit_radius_setting.py) can be flown and read. Any failed check refuses the series (exit 1); so does any input it
cannot read. Nothing here reaches navpy: navpy gets only the profile radius, and this check reads no dock pose and
no seed (only the scenario's public lane and limits, scenario_public.py).

Geometry: the aircraft circles at radius r, h = lane height - board height above home. The circle's centre is
navpy's own geolocation of the board; at this range step 22's pose fit may be refused, so the centre can be the
height-zero ground ray, which lies r * board_height / h beyond the board. The board is then between r - e and r + e
away horizontally, e = max(that error, step 22's 0.5 m pose limit).

Checks, each using the model speed (the run's commanded airspeed, inside the model's declared airspeed band) and
the profile's measured optics at the planned zoom step:

- model_speed: the commanded airspeed lies in the model input's ``envelope.airspeed_m_s`` band;
- turn_radius: r >= kappa v^2 / (g tan 35 deg), the planner's turn term (orbit_geometry.py, kappa 2) at the NAV
  law's bank cap (command_anchor.ROLL_LIMIT_CAP_DEG);
- descent: r >= h / tan(eta |PTCH_LIM_MIN_DEG|), the planner's descent term (eta 0.65);
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
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from navpy.app.perception.fixed_zoom_profile import build_fixed_zoom
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.perception.zoom.fixed_step_zoom_policy import ContainmentMargins
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.skills.acquire.orbit_geometry import OrbitNavigationLimits
from navpy.core.skills.acquire.orbit_radius_setting import final_approach_orbit_setting
from navpy.plugins.detection.charuco.charuco_board import charuco_board_spec_from_settings
from navpy.plugins.laws.vision_pn.command_anchor import ROLL_LIMIT_CAP_DEG
from tools.hitl import scenario_public as sp
from tools.hitl.json_checks import read_json
from tools.tuning.dock_pass import PASS_AIRSPEED_M_S
from tools.tuning.hitl_params import read_param_file

GRAVITY = 9.81
HITL_PROFILE = "siyi_zr10_hitl_encoded"
# Plan value (section 4 "Range per zoom", dock4 in hitl_unreal_camera.md): the reader needs about 11.5 px per 40 cm
# square along the line of sight.
READER_MIN_SQUARE_PX = 11.5
# Plan value (section 6, from the pickup plan): a 20-30 deg dive loses 4-9 m in its pull-out; unmeasured (H2).
PULL_OUT_ALLOWANCE_M = 9.0
# Step 22's done limit on the own-detected centre when its pose fit is accepted.
POSE_CENTRE_LIMIT_M = 0.5
MODEL_AIRSPEED_BAND = "airspeed_m_s"
MIN_PITCH_PARAM = "PTCH_LIM_MIN_DEG"
CRUISE_PARAM = "AIRSPEED_CRUISE"


@dataclass(frozen=True)
class Board:
    """The ChArUco grid the detector boxes, in metres."""
    width_m: float
    length_m: float
    square_m: float

    @property
    def diagonal_m(self) -> float:
        return math.hypot(self.width_m, self.length_m)


@dataclass(frozen=True)
class OrbitPlan:
    """Everything the checks use; built from files by ``load_plan``."""
    radius_m: float
    lane_height_m: float
    board_height_m: float           # the board above home: the scene's design height, not a seed's pose
    floor_m: float
    airspeed_m_s: float
    airspeed_band: Optional[tuple[float, float]]
    min_pitch_deg: float
    steps: FixedZoomSteps
    zoom_command_tenths: int
    board: Board
    reader_min_square_px: float = READER_MIN_SQUARE_PX
    margins: ContainmentMargins = ContainmentMargins()


@dataclass(frozen=True)
class CheckResult:
    name: str
    passed: bool
    detail: dict[str, Any]


def _finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def height_above_board(plan: OrbitPlan) -> float:
    return plan.lane_height_m - plan.board_height_m


def centre_error_m(plan: OrbitPlan) -> float:
    """The worst horizontal error of the orbit centre: the height-zero ground ray, or the pose limit."""
    h = height_above_board(plan)
    return max(abs(plan.radius_m * plan.board_height_m / h), POSE_CENTRE_LIMIT_M)


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


def check_turn_radius(plan: OrbitPlan) -> CheckResult:
    kappa = OrbitNavigationLimits(plan.airspeed_m_s, ROLL_LIMIT_CAP_DEG, plan.min_pitch_deg).kappa
    raw = plan.airspeed_m_s ** 2 / (GRAVITY * math.tan(math.radians(ROLL_LIMIT_CAP_DEG)))
    need = kappa * raw
    return CheckResult("turn_radius", plan.radius_m >= need,
                       {"radius_m": plan.radius_m, "turn_radius_m": round(raw, 2), "kappa": kappa,
                        "required_m": round(need, 2), "bank_cap_deg": ROLL_LIMIT_CAP_DEG})


def check_descent(plan: OrbitPlan) -> CheckResult:
    eta = OrbitNavigationLimits(plan.airspeed_m_s, ROLL_LIMIT_CAP_DEG, plan.min_pitch_deg).eta
    gamma = eta * abs(plan.min_pitch_deg)
    h = height_above_board(plan)
    passed = 0.0 < gamma < 90.0 and plan.radius_m >= h / math.tan(math.radians(gamma))
    need = h / math.tan(math.radians(gamma)) if 0.0 < gamma < 90.0 else None
    return CheckResult("descent", passed, {"height_above_board_m": h, "min_pitch_deg": plan.min_pitch_deg,
                                           "eta": eta, "required_m": None if need is None else round(need, 2)})


def _planned_step(plan: OrbitPlan) -> Optional[ZoomStep]:
    return plan.steps.by_command(plan.zoom_command_tenths)


def check_reader(plan: OrbitPlan) -> CheckResult:
    step, h, error = _planned_step(plan), height_above_board(plan), centre_error_m(plan)
    far = _slant(plan.radius_m + error, h)
    if step is None:
        return CheckResult("reader", False, {"zoom_step": "not a measured step"})
    px = step.optics.fy * plan.board.square_m * (h / far) / far
    return CheckResult("reader", px >= plan.reader_min_square_px,
                       {"zoom_step": step.command, "fy_px": step.optics.fy, "centre_error_m": round(error, 2),
                        "farthest_slant_m": round(far, 2), "square_px_along_los": round(px, 2),
                        "reader_min_square_px": plan.reader_min_square_px})


def check_containment(plan: OrbitPlan) -> CheckResult:
    step, h, error = _planned_step(plan), height_above_board(plan), centre_error_m(plan)
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
               plan.min_pitch_deg, plan.reader_min_square_px)
    if not _finite(*numbers) or plan.radius_m <= 0.0 or plan.airspeed_m_s <= 0.0 or height_above_board(plan) <= 0.0:
        reason = {"input": "non-finite value, non-positive radius or airspeed, or the lane not above the board"}
        return [CheckResult(check.__name__.removeprefix("check_"), False, reason) for check in CHECKS]
    return [check(plan) for check in CHECKS]


def feasible(results: Sequence[CheckResult]) -> bool:
    return bool(results) and all(result.passed for result in results)


# --- inputs -----------------------------------------------------------------------------------------------------

def _zoom_steps(profile: Mapping[str, Any]) -> FixedZoomSteps:
    built = [zoom for zoom in (build_fixed_zoom(device) for device in profile.get("devices", ())) if zoom]
    if len(built) != 1:
        raise ValueError(f"the profile needs exactly one mount with measured zoom steps, found {len(built)}")
    return built[0].steps


def _board(profile: Mapping[str, Any]) -> Board:
    spec = charuco_board_spec_from_settings(profile.get("detector", {}))
    return Board(spec.squares_x * spec.square_mm / 1000.0, spec.squares_y * spec.square_mm / 1000.0,
                 spec.square_mm / 1000.0)


def _airspeed_band(model_input: Mapping[str, Any]) -> Optional[tuple[float, float]]:
    band = model_input.get("envelope", {}).get(MODEL_AIRSPEED_BAND)
    if not isinstance(band, Mapping) or not _finite(band.get("min"), band.get("max")):
        return None
    return float(band["min"]), float(band["max"])


def load_plan(args: argparse.Namespace) -> OrbitPlan:
    profiles, _, _ = load_profiles()
    if args.profile not in profiles:
        raise ValueError(f"vision profile {args.profile!r} not found")
    profile = profiles[args.profile]
    radius = final_approach_orbit_setting(profile)
    if radius is None:
        raise ValueError(f"vision profile {args.profile!r} sets no final_approach_orbit.radius_m")
    public = sp.read_public(Path(args.scenario))
    params = read_param_file(Path(args.params_file))
    if MIN_PITCH_PARAM not in params:
        raise ValueError(f"{args.params_file} sets no {MIN_PITCH_PARAM}")
    # The lane commands PASS_AIRSPEED_M_S (DO_CHANGE_SPEED); the faster of it and the cruise is the worst case.
    airspeed = max(PASS_AIRSPEED_M_S, params.get(CRUISE_PARAM, 0.0))
    return OrbitPlan(
        radius_m=radius, lane_height_m=float(public["lane"]["height_m"]), board_height_m=float(args.board_height_m),
        floor_m=float(public["limits"]["min_height_m"]), airspeed_m_s=airspeed,
        airspeed_band=_airspeed_band(read_json(Path(args.model_input))), min_pitch_deg=params[MIN_PITCH_PARAM],
        steps=_zoom_steps(profile), zoom_command_tenths=round(float(args.zoom) * 10.0), board=_board(profile),
        reader_min_square_px=float(args.reader_min_square_px))


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
    results = run_checks(plan)
    ok = feasible(results)
    print(json.dumps({"feasible": ok, "checks": [{"name": r.name, "passed": r.passed, **r.detail}
                                                 for r in results]}, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
