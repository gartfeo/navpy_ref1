"""The inputs of the pre-series orbit check (scenario_checks.py, hitl_guidance.md step 23).

An OrbitPlan holds everything the checks use: the HITL profile's explicit radius, measured zoom optics and board,
the scenario's public lane height and floor (scenario_public.py, never a seed or dock pose), the run's merged
aircraft parameters, the model input's airspeed band and the board's design height.
"""
from __future__ import annotations

import argparse
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from navpy.app.perception.fixed_zoom_profile import build_fixed_zoom
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.perception.zoom.fixed_step_zoom_policy import ContainmentMargins
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.skills.acquire.orbit_radius_setting import guidance_orbit_setting
from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec, charuco_board_spec_from_settings
from tools.hitl import scenario_public as sp
from tools.hitl.json_checks import read_json
from tools.tuning.dock_pass import PASS_AIRSPEED_M_S
from tools.tuning.hitl_params import read_param_file

HITL_PROFILE = "siyi_zr10_hitl_encoded"
# Plan value (section 4 "Range per zoom", dock4 in hitl_unreal_camera.md): the reader needs about 11.5 px per 40 cm
# square along the line of sight.
READER_MIN_SQUARE_PX = 11.5
MODEL_AIRSPEED_BAND = "airspeed_m_s"
MIN_PITCH_PARAM = "PTCH_LIM_MIN_DEG"
ROLL_LIMIT_PARAM = "ROLL_LIMIT_DEG"
CRUISE_PARAM = "AIRSPEED_CRUISE"


@dataclass(frozen=True)
class Board:
    """The ChArUco board the detector boxes and the pose fit uses (the profile's detector.charuco)."""
    spec: CharucoBoardSpec

    @property
    def width_m(self) -> float:
        return self.spec.squares_x * self.spec.square_mm / 1000.0

    @property
    def length_m(self) -> float:
        return self.spec.squares_y * self.spec.square_mm / 1000.0

    @property
    def square_m(self) -> float:
        return self.spec.square_mm / 1000.0

    @property
    def diagonal_m(self) -> float:
        return math.hypot(self.width_m, self.length_m)


@dataclass(frozen=True)
class OrbitPlan:
    """Everything the checks use; built from files by ``plan_for``."""
    radius_m: float
    lane_height_m: float
    board_height_m: float           # the board above home: the scene's design height, not a seed's pose
    floor_m: float
    airspeed_m_s: float
    airspeed_band: Optional[tuple[float, float]]
    min_pitch_deg: float
    roll_limit_deg: float           # the aircraft's ROLL_LIMIT_DEG, which the planner's turn term reads
    steps: FixedZoomSteps
    zoom_command_tenths: int
    board: Board
    reader_min_square_px: float = READER_MIN_SQUARE_PX
    margins: ContainmentMargins = ContainmentMargins()


def finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def height_above_board(plan: OrbitPlan) -> float:
    return plan.lane_height_m - plan.board_height_m


def planned_step(plan: OrbitPlan) -> Optional[ZoomStep]:
    return plan.steps.by_command(plan.zoom_command_tenths)


def reading_slant_m(plan: OrbitPlan, step: ZoomStep) -> float:
    """The farthest slant the board's squares are read from at this step and height (px = fy s h / R^2)."""
    return math.sqrt(step.optics.fy * plan.board.square_m * height_above_board(plan) / plan.reader_min_square_px)


def _zoom_steps(profile: Mapping[str, Any]) -> FixedZoomSteps:
    built = [zoom for zoom in (build_fixed_zoom(device) for device in profile.get("devices", ())) if zoom]
    if len(built) != 1:
        raise ValueError(f"the profile needs exactly one mount with measured zoom steps, found {len(built)}")
    return built[0].steps


def _board(profile: Mapping[str, Any]) -> Board:
    return Board(charuco_board_spec_from_settings(profile.get("detector", {})))


def _airspeed_band(model_input: Mapping[str, Any]) -> Optional[tuple[float, float]]:
    band = model_input.get("envelope", {}).get(MODEL_AIRSPEED_BAND)
    if not isinstance(band, Mapping) or not finite(band.get("min"), band.get("max")):
        return None
    return float(band["min"]), float(band["max"])


def plan_for(public: Mapping[str, Any], params: Mapping[str, float], model_input: Mapping[str, Any],
             board_height_m: float, *, profile_name: str = HITL_PROFILE, zoom: float = 10.0,
             reader_min_square_px: float = READER_MIN_SQUARE_PX) -> OrbitPlan:
    """The plan from the scenario's public half, the run's merged parameters and the model input's envelope."""
    profiles, _, _ = load_profiles()
    if profile_name not in profiles:
        raise ValueError(f"vision profile {profile_name!r} not found")
    profile = profiles[profile_name]
    radius = guidance_orbit_setting(profile)
    if radius is None:
        raise ValueError(f"vision profile {profile_name!r} sets no guidance_orbit.radius_m")
    for name in (MIN_PITCH_PARAM, ROLL_LIMIT_PARAM):
        if name not in params:
            raise ValueError(f"the run's parameters set no {name}")
    # The lane commands PASS_AIRSPEED_M_S (DO_CHANGE_SPEED); the faster of it and the cruise is the worst case.
    airspeed = max(PASS_AIRSPEED_M_S, params.get(CRUISE_PARAM, 0.0))
    return OrbitPlan(
        radius_m=radius, lane_height_m=float(public["lane"]["height_m"]), board_height_m=float(board_height_m),
        floor_m=float(public["limits"]["min_height_m"]), airspeed_m_s=airspeed,
        airspeed_band=_airspeed_band(model_input), min_pitch_deg=params[MIN_PITCH_PARAM],
        roll_limit_deg=params[ROLL_LIMIT_PARAM],
        steps=_zoom_steps(profile), zoom_command_tenths=round(float(zoom) * 10.0), board=_board(profile),
        reader_min_square_px=float(reader_min_square_px))


def load_plan(args: argparse.Namespace) -> OrbitPlan:
    """The plan from the CLI's files: a public scenario folder, one parameter file and the model input."""
    return plan_for(sp.read_public(Path(args.scenario)), read_param_file(Path(args.params_file)),
                    read_json(Path(args.model_input)), args.board_height_m, profile_name=args.profile,
                    zoom=args.zoom, reader_min_square_px=args.reader_min_square_px)
