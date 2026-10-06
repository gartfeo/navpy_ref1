"""Finite-SIYI pixel/camera loop used by final-approach certification tests."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.modules.navigation.nav.vision_nav.law import (
    FixedFinalApproachLawConfigProvider,
    FinalApproachLawConfig,
    VisionNavLaw,
)
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTracker
from navpy.modules.vision.gimbal_rate_types import GimbalRateTrackerConfig
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample
from navpy.modules.vision.models.pixel_observation import (
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    VisualDetection,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.visual_ray_projection import (
    camera_to_body_from_gimbal,
)
from navpy.utils.math_utils import GRAVITY_MSS
from scripts.vision_static_point_mass_plant import (
    PointMassState,
    closest_point_to_poi,
    component_miss,
    has_passed_poi,
    initial_state,
    integrate_plant,
    is_approaching_poi,
)
from scripts.vision_static_point_mass_sensor import (
    render_poi_pixel,
    undelayed_truth_gyro,
)
from scripts.vision_static_point_mass_types import (
    StaticPointMassCase,
    StaticPointMassMiss,
)


FRAME_WIDTH_PX = 2560.0
FRAME_HEIGHT_PX = 1440.0
CALIBRATION = PixelCalibration(2066.49, 2082.84, 1199.61, 810.24)
PROJECTOR = FinalApproachFrameProjector(FinalApproachProjectionConfig("ZYX", True))
TEST_AIRSPEED_MPS = 27.0


@dataclass(frozen=True)
class SiyiCameraLoopResult:
    miss: StaticPointMassMiss
    first_fov_loss_m: float | None
    observation_count: int


@dataclass(frozen=True)
class _RenderedMeasurement:
    frame: FinalApproachVisionFrame
    gimbal_sample: GimbalAngularSample


class _NoopLog:
    def debug(self, _message: str) -> None:
        return None

    def info(self, _message: str) -> None:
        return None


class _SiyiRatePlant:
    MAX_SLEW_DPS = 90.0

    def __init__(self, *, pitch_deg: float, yaw_deg: float) -> None:
        self._pitch_deg = float(pitch_deg)
        self._yaw_deg = float(yaw_deg)
        self._pitch_command = 0.0
        self._yaw_command = 0.0

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        self._yaw_command = float(np.clip(yaw_rate, -100.0, 100.0))
        self._pitch_command = float(np.clip(pitch_rate, -100.0, 100.0))

    def step(self, dt_s: float) -> None:
        scale = self.MAX_SLEW_DPS / 100.0
        self._yaw_deg += self._yaw_command * scale * dt_s
        self._pitch_deg += self._pitch_command * scale * dt_s

    def data(self, timestamp_s: float) -> GimbalData:
        return GimbalData(
            att=Attitude(self._pitch_deg, self._yaw_deg, 0.0),
            timestamp_s=timestamp_s,
        )


def run_siyi_camera_loop() -> SiyiCameraLoopResult:
    case = StaticPointMassCase(
        name="siyi-log-153922-camera-loop",
        wind_dir_from_deg=0.0,
        wind_speed_mps=0.0,
        start_ned_m=(-301.6, -2.6, -135.1),
        pitch_deg=-3.4,
        airspeed_mps=TEST_AIRSPEED_MPS,
        attitude_time_constant_s=0.45,
        dt_s=0.01,
        max_t_s=20.0,
    )
    state = initial_state(case)
    gimbal = _SiyiRatePlant(pitch_deg=-20.72905, yaw_deg=0.481962)
    tracker = GimbalRateTracker(
        gimbal,
        _NoopLog(),
        GimbalRateTrackerConfig(command_lead_time=0.0),
    )
    # The measurement below publishes the instantaneous coordinated-turn
    # rate, so the law must model a zero gyro delay here (see the sensor
    # module's undelayed_truth_gyro docstring).
    with undelayed_truth_gyro():
        law = VisionNavLaw(
            FixedFinalApproachLawConfigProvider(
                FinalApproachLawConfig(-55.0, 30.0, 60.0, 0.5, None)
            )
        )
    law.seed(
        FinalApproachVisionFrame(
            "siyi-finite-camera",
            0,
            1,
            1,
            state.t_s,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            state.roll_deg,
            case.airspeed_mps,
            0.0,
            state.pitch_deg,
        )
    )
    target_pitch_deg = state.pitch_deg
    target_roll_deg = state.roll_deg
    next_detector_t_s = 0.0
    detector_interval_s = 1.0 / 30.0
    first_fov_loss_m: float | None = None
    observation_count = 0
    approach_observed = False
    best_miss = StaticPointMassMiss(
        math.inf,
        math.inf,
        math.inf,
        math.inf,
        0.0,
    )

    for _ in range(int(case.max_t_s / case.dt_s)):
        if state.t_s + 1e-12 >= next_detector_t_s:
            rendered = _render_measurement(state, gimbal)
            if rendered is None:
                gimbal.set_rate(0.0, 0.0)
                if first_fov_loss_m is None:
                    first_fov_loss_m = float(
                        np.linalg.norm(state.position_ned_m)
                    )
            else:
                tracker.update(rendered.gimbal_sample)
                plan = law.plan(rendered.frame)
                assert plan is not None
                target_roll_deg = plan.command.cmd_roll_deg
                target_pitch_deg = plan.command.cmd_pitch_deg
                law.commit(plan)
                observation_count += 1
            next_detector_t_s += detector_interval_s

        gimbal.step(case.dt_s)
        step = integrate_plant(
            state,
            case,
            target_pitch_deg,
            target_roll_deg,
            np.zeros(3, dtype=float),
        )
        state = step.state
        closest = closest_point_to_poi(
            step.segment_start_ned_m,
            step.segment_ned_m,
        )
        miss = component_miss(closest, step.segment_ned_m, state.t_s)
        if miss.slant_m < best_miss.slant_m:
            best_miss = miss
        approach_observed = approach_observed or is_approaching_poi(step)
        if approach_observed and has_passed_poi(step, case.dt_s):
            return SiyiCameraLoopResult(
                replace(best_miss, passed_poi=True),
                first_fov_loss_m,
                observation_count,
            )

    return SiyiCameraLoopResult(
        replace(best_miss, timed_out=True),
        first_fov_loss_m,
        observation_count,
    )


def _render_measurement(
    state: PointMassState,
    gimbal: _SiyiRatePlant,
) -> _RenderedMeasurement | None:
    gimbal_data = gimbal.data(state.t_s)
    camera_to_body = camera_to_body_from_gimbal(gimbal_data)
    try:
        u_px, v_px = render_poi_pixel(
            position_ned_m=state.position_ned_m,
            pitch_deg=state.pitch_deg,
            roll_deg=state.roll_deg,
            yaw_deg=state.yaw_deg,
            calibration=CALIBRATION,
            camera_to_body=camera_to_body,
            projection=PixelProjectionKind.PINHOLE,
        )
    except ValueError:
        return None
    if not (0.0 <= u_px <= FRAME_WIDTH_PX and 0.0 <= v_px <= FRAME_HEIGHT_PX):
        return None
    detection = VisualDetection(
        task_id=1,
        obj_id=1,
        observation=PixelObservation(
            u_px=u_px,
            v_px=v_px,
            calibration=CALIBRATION,
            camera_to_body=camera_to_body,
            projection=PixelProjectionKind.PINHOLE,
            aircraft_pitch_deg=state.pitch_deg,
            aircraft_roll_deg=state.roll_deg,
            source_timestamp_s=state.t_s,
            pose_is_frame_atomic=True,
            source_name="siyi-finite-camera",
            aircraft_yaw_rate_rad_s=(
                GRAVITY_MSS
                * math.tan(math.radians(state.roll_deg))
                / max(
                    TEST_AIRSPEED_MPS
                    * math.cos(math.radians(state.pitch_deg)),
                    1e-6,
                )
            ),
        ),
    )
    frame = PROJECTOR.project(
        detection,
        source_generation=0,
        air_speed_mps=TEST_AIRSPEED_MPS,
    )
    assert frame is not None
    return _RenderedMeasurement(
        frame,
        GimbalAngularSample(
            yaw_error_rad=(u_px - CALIBRATION.cx_px) / CALIBRATION.fx_px,
            pitch_error_rad=(v_px - CALIBRATION.cy_px) / CALIBRATION.fy_px,
            source_timestamp_s=state.t_s,
        ),
    )


__all__ = ["SiyiCameraLoopResult", "run_siyi_camera_loop"]
