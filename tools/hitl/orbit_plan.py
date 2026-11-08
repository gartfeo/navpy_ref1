"""The inputs of the pre-series orbit check (scenario_checks.py, hitl_final_approach.md steps 23 and 23b).

An OrbitPlan holds everything the checks use: the HITL profile's explicit radius, measured zoom optics and whole-dock
detector limits (its rfdetr input and acquisition blocks), the scenario's public lane height, floor and dock size
(scenario_public.py, never a seed or dock pose), the run's merged aircraft parameters, the model input's airspeed band
and the declared ground-ray error bounds (orbit_centre.py).
"""
from __future__ import annotations

import argparse
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from navpy.app.perception.fixed_zoom_profile import build_fixed_zoom
from navpy.app.perception.rfdetr_profile import whole_dock_settings
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.perception.tracking.dock_acquisition_gate import acquisition_block
from navpy.core.perception.zoom.fixed_step_zoom_policy import ContainmentMargins
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.skills.acquire.orbit_radius_setting import final_approach_orbit_setting
from tools.hitl import scenario_public as sp
from tools.hitl.json_checks import read_json
from tools.tuning.dock_pass import PASS_AIRSPEED_M_S
from tools.tuning.hitl_params import read_param_file

HITL_PROFILE = "siyi_zr10_hitl_encoded"
MODEL_AIRSPEED_BAND = "airspeed_m_s"
MIN_PITCH_PARAM = "PTCH_LIM_MIN_DEG"
ROLL_LIMIT_PARAM = "ROLL_LIMIT_DEG"
CRUISE_PARAM = "AIRSPEED_CRUISE"


@dataclass(frozen=True)
class Dock:
    """The scene's dock (phase A: a car on the ground), from the scenario's public half."""
    length_m: float
    width_m: float
    height_m: float

    @property
    def poi_height_m(self) -> float:
        """The POI, the dock's centre at mid-height, above home: it stands on the ground at home's height."""
        return self.height_m / 2.0


@dataclass(frozen=True)
class DetectorLimits:
    """The whole-dock detector's measured and enforced box sizes, in its own input pixels."""
    input_scale: Callable[[int, int], tuple[float, float]]
    recall_min_box_px: float        # measured: recall meets the target from here up (hitl_dock_detector.md 6.1)
    acq_min_box_px: float           # enforced by the acquisition gate: bounds the first-detection distance
    evidence: str


@dataclass(frozen=True)
class GroundRayBounds:
    """Declared stage 1 bounds of the height-zero ground ray's error terms (hitl_dock_detector.md section 4.1).

    My values for stage 1 (SITL attitude, the emulated gimbal): placeholders until section 6 items 3 and 6 measure
    them; stage 3 and later use measured readback and compass bounds."""
    lane_height_error_m: float = 1.0        # aircraft relative altitude and home reference
    depression_bias_deg: float = 0.2        # gimbal readback, mount, aircraft pitch
    azimuth_bias_deg: float = 1.0           # gimbal yaw readback, mount, aircraft compass yaw
    box_centre_bias_px: float = 3.0         # box centre against the POI (section 6 item 3)
    box_size_error_px: float = 2.0          # the box's measured size against the dock's (sets D_max)
    capture_skew_s: float = 0.1             # frame capture time against the pose it is paired with


@dataclass(frozen=True)
class OrbitPlan:
    """Everything the checks use; built from files by ``plan_for``."""
    radius_m: float
    lane_height_m: float
    dock: Dock
    floor_m: float
    airspeed_m_s: float
    airspeed_band: Optional[tuple[float, float]]
    min_pitch_deg: float
    roll_limit_deg: float           # the aircraft's ROLL_LIMIT_DEG, which the planner's turn term reads
    steps: FixedZoomSteps
    zoom_command_tenths: int
    detector: DetectorLimits
    bounds: GroundRayBounds = field(default_factory=GroundRayBounds)
    margins: ContainmentMargins = ContainmentMargins()


def finite(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def height_above_poi(plan: OrbitPlan) -> float:
    return plan.lane_height_m - plan.dock.poi_height_m


def planned_step(plan: OrbitPlan) -> Optional[ZoomStep]:
    return plan.steps.by_command(plan.zoom_command_tenths)


def search_step(plan: OrbitPlan) -> ZoomStep:
    """The search's step: the widest (each search start commands it, step 21)."""
    return plan.steps.steps[0]


def _zoom_steps(profile: Mapping[str, Any]) -> FixedZoomSteps:
    built = [zoom for zoom in (build_fixed_zoom(device) for device in profile.get("devices", ())) if zoom]
    if len(built) != 1:
        raise ValueError(f"the profile needs exactly one mount with measured zoom steps, found {len(built)}")
    return built[0].steps


def detector_limits(profile: Mapping[str, Any]) -> DetectorLimits:
    detector = profile.get("detector", {})
    block = acquisition_block(detector)
    if block is None:
        raise ValueError("the profile sets no detector.acquisition block (the whole-dock detector)")
    return DetectorLimits(whole_dock_settings(detector).input_scale, float(block["recall_min_box_px"]),
                          float(block["min_box_px"]), str(block["recall_evidence"]))


def dock_of(public: Mapping[str, Any]) -> Dock:
    dock = public["dock_size"]
    if dock["placement"] != "ground":
        raise ValueError(f"dock placement {dock['placement']!r} is not checked; only a dock on the ground")
    return Dock(float(dock["length_m"]), float(dock["width_m"]), float(dock["height_m"]))


def _airspeed_band(model_input: Mapping[str, Any]) -> Optional[tuple[float, float]]:
    band = model_input.get("envelope", {}).get(MODEL_AIRSPEED_BAND)
    if not isinstance(band, Mapping) or not finite(band.get("min"), band.get("max")):
        return None
    return float(band["min"]), float(band["max"])


def plan_for(public: Mapping[str, Any], params: Mapping[str, float], model_input: Mapping[str, Any], *,
             profile_name: str = HITL_PROFILE, zoom: float = 10.0) -> OrbitPlan:
    """The plan from the scenario's public half, the run's merged parameters and the model input's envelope."""
    profiles, _, _ = load_profiles()
    if profile_name not in profiles:
        raise ValueError(f"vision profile {profile_name!r} not found")
    profile = profiles[profile_name]
    radius = final_approach_orbit_setting(profile)
    if radius is None:
        raise ValueError(f"vision profile {profile_name!r} sets no final_approach_orbit.radius_m")
    for name in (MIN_PITCH_PARAM, ROLL_LIMIT_PARAM):
        if name not in params:
            raise ValueError(f"the run's parameters set no {name}")
    # The lane commands PASS_AIRSPEED_M_S (DO_CHANGE_SPEED); the faster of it and the cruise is the worst case.
    airspeed = max(PASS_AIRSPEED_M_S, params.get(CRUISE_PARAM, 0.0))
    return OrbitPlan(
        radius_m=radius, lane_height_m=float(public["lane"]["height_m"]), dock=dock_of(public),
        floor_m=float(public["limits"]["min_height_m"]), airspeed_m_s=airspeed,
        airspeed_band=_airspeed_band(model_input), min_pitch_deg=params[MIN_PITCH_PARAM],
        roll_limit_deg=params[ROLL_LIMIT_PARAM], steps=_zoom_steps(profile),
        zoom_command_tenths=round(float(zoom) * 10.0), detector=detector_limits(profile))


def load_plan(args: argparse.Namespace) -> OrbitPlan:
    """The plan from the CLI's files: a public scenario folder, one parameter file and the model input."""
    return plan_for(sp.read_public(Path(args.scenario)), read_param_file(Path(args.params_file)),
                    read_json(Path(args.model_input)), profile_name=args.profile, zoom=args.zoom)
