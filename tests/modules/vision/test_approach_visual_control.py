"""Camera-motion feedforward for gimbal pointing (HITL step 24, design N7)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np
import pytest

from navpy.core.perception.pointing.compensated_los_rate import (
    CameraFeedforwardConfig,
)
from navpy.core.perception.pointing.gimbal_rate_tracker import GimbalRateTracker
from navpy.core.perception.pointing.gimbal_rate_types import (
    GimbalRateTrackerConfig,
)
from tests.modules.vision.camera_motion_world import (
    FRAME_DT_S,
    WorldState,
    camera_world,
    los_world,
    sample,
    wrap_deg,
)

ZERO_MOTION_RESIDUAL_DPS = 0.1  # plan step 24 done check
SETTLE_S = 2.0


@dataclass
class _Actuator:
    commands: list[tuple[float, float]] = field(default_factory=list)
    fail_next: bool = False

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        if self.fail_next:
            self.fail_next = False
            raise OSError("actuator unavailable")
        self.commands.append((yaw_rate, pitch_rate))


class _Logger:
    def debug(self, message: str) -> None:
        pass

    def info(self, message: str) -> None:
        pass

    def warning(self, message: str) -> None:
        pass


def _tracker(
    feedforward: bool = True,
) -> tuple[GimbalRateTracker, _Actuator]:
    actuator = _Actuator()
    config = GimbalRateTrackerConfig(
        camera_feedforward=CameraFeedforwardConfig() if feedforward else None
    )
    return GimbalRateTracker(actuator, _Logger(), config), actuator


def _scripted(
    t: float,
    los_az_rate_dps: float = 0.0,
    los_el_rate_dps: float = 0.0,
    los_el0_deg: float = -17.0,
) -> WorldState:
    """Aircraft turning, pitching and rolling; camera wobbling around the LOS."""
    los_az = 120.0 + los_az_rate_dps * t
    los_el = los_el0_deg + los_el_rate_dps * t
    return WorldState(
        heading_deg=30.0 + 8.0 * t + 3.0 * math.sin(1.3 * t),
        pitch_deg=4.0 * math.sin(0.9 * t),
        roll_deg=20.0 + 6.0 * math.sin(0.7 * t),
        turn_rate_dps=8.0 + 3.0 * 1.3 * math.cos(1.3 * t),
        camera_yaw_deg=los_az + 2.0 * math.sin(1.1 * t),
        camera_elevation_deg=los_el + 1.5 * math.sin(0.8 * t),
        los_azimuth_deg=los_az,
        los_elevation_deg=los_el,
    )


def _open_loop(
    duration_s: float,
    world,
    **noise,
) -> list[tuple[float, float, float]]:
    """(time, yaw ff dps, pitch ff dps) for scripted camera motion."""
    tracker, _ = _tracker()
    out = []
    for index in range(int(duration_s / FRAME_DT_S)):
        t = index * FRAME_DT_S
        kwargs = {key: value(index) for key, value in noise.items()}
        result = tracker.update(sample(world(t), t, **kwargs)).result
        out.append(
            (
                t,
                math.degrees(result.feedforward_yaw_rate_rad_s),
                math.degrees(result.feedforward_pitch_rate_rad_s),
            )
        )
    return out


def _settled(rows):
    return [row for row in rows if row[0] >= SETTLE_S]


def test_fixed_ray_under_camera_and_aircraft_motion_has_no_feedforward():
    rows = _settled(_open_loop(10.0, _scripted))

    residual = max(max(abs(yaw), abs(pitch)) for _, yaw, pitch in rows)
    assert residual <= ZERO_MOTION_RESIDUAL_DPS


@pytest.mark.parametrize(
    ("az_rate", "el_rate"),
    [(5.0, 0.0), (-8.1, 0.0), (0.0, 3.0), (6.0, -2.5)],
)
def test_moving_ray_feedforward_is_the_world_los_rate(az_rate, el_rate):
    rows = _settled(
        _open_loop(8.0, lambda t: _scripted(t, az_rate, el_rate, -30.0))
    )

    for _, yaw, pitch in rows:
        assert yaw == pytest.approx(az_rate, abs=ZERO_MOTION_RESIDUAL_DPS)
        assert pitch == pytest.approx(el_rate, abs=ZERO_MOTION_RESIDUAL_DPS)


def test_constant_pixels_while_the_camera_turns_feed_forward_the_camera_rate():
    def world(t: float) -> WorldState:
        yaw = 40.0 + 6.0 * t
        return WorldState(10.0, 2.0, 15.0, 0.0, yaw, -20.0, yaw + 1.0, -20.0)

    rows = _settled(_open_loop(6.0, world))

    for _, yaw, pitch in rows:
        assert yaw == pytest.approx(6.0, abs=ZERO_MOTION_RESIDUAL_DPS)
        assert abs(pitch) <= ZERO_MOTION_RESIDUAL_DPS


@pytest.mark.parametrize("seed", range(10))
def test_noisy_replay_feedforward_has_the_los_rate_sign(seed):
    rng = np.random.default_rng(seed)
    az_rate = rng.choice([-1.0, 1.0]) * rng.uniform(2.0, 10.0)
    el_rate = rng.choice([-1.0, 1.0]) * rng.uniform(1.0, 4.0)
    gyro_bias = rng.normal(0.0, 0.05)

    def world(t: float) -> WorldState:
        return _scripted(t, az_rate, el_rate, -40.0)

    def readback(index: int) -> tuple[float, float]:
        state = world(index * FRAME_DT_S)  # ZR10 reports 0.1 deg steps
        return (
            round(state.camera_yaw_deg, 1) - state.camera_yaw_deg,
            round(state.camera_elevation_deg, 1) - state.camera_elevation_deg,
        )

    rows = _settled(
        _open_loop(
            8.0,
            world,
            readback_error_deg=readback,
            pixel_error_rad=lambda _: tuple(rng.normal(0.0, 0.3 / 1400.0, 2)),
            gyro_error_dps=lambda _: gyro_bias + rng.normal(0.0, 0.1),
        )
    )

    for _, yaw, pitch in rows:  # every settled frame, not only the mean
        assert math.copysign(1.0, yaw) == math.copysign(1.0, az_rate)
        assert math.copysign(1.0, pitch) == math.copysign(1.0, el_rate)
    yaw_mean = float(np.mean([row[1] for row in rows]))
    pitch_mean = float(np.mean([row[2] for row in rows]))
    assert yaw_mean == pytest.approx(az_rate, abs=0.5)
    assert pitch_mean == pytest.approx(el_rate, abs=0.5)


def _orbit_error_deg(feedforward: bool, duration_s: float = 12.0) -> float:
    """Closed loop on a 170 m, 24 m/s right orbit; gimbal in LOCK, 1-frame delay."""
    turn_dps = math.degrees(24.0 / 170.0)
    roll = math.degrees(math.atan(24.0 ** 2 / (9.81 * 170.0)))
    tracker, actuator = _tracker(feedforward)
    max_slew = GimbalRateTrackerConfig().max_slew_dps
    camera_yaw, camera_el = 92.0, -16.0
    pending = (0.0, 0.0)
    errors = []
    for index in range(int(duration_s / FRAME_DT_S)):
        t = index * FRAME_DT_S
        heading = turn_dps * t
        state = WorldState(
            heading, 0.0, roll, turn_dps, camera_yaw, camera_el, heading + 90.0, -17.0
        )
        tracker.update(sample(state, t))
        axis = camera_world(camera_yaw, camera_el)[:, 2]
        los = los_world(state.los_azimuth_deg, state.los_elevation_deg)
        if t >= duration_s / 2.0:
            errors.append(math.degrees(math.acos(min(1.0, float(axis @ los)))))
        camera_yaw += pending[0] / 100.0 * max_slew * FRAME_DT_S
        camera_el += pending[1] / 100.0 * max_slew * FRAME_DT_S
        pending = actuator.commands[-1]
    return max(errors)


def test_orbit_feedforward_removes_the_steady_tracking_lag():
    without = _orbit_error_deg(feedforward=False)
    with_ff = _orbit_error_deg(feedforward=True)

    assert without > 2.5  # (v/r)/bw = 3.2 deg, design N7
    assert with_ff < GimbalRateTrackerConfig().settle_tolerance_deg


def test_missing_camera_state_gives_no_feedforward_and_restarts():
    tracker, _ = _tracker()
    for index in range(30):
        t = index * FRAME_DT_S
        tracker.update(sample(_scripted(t, 5.0), t))
    t = 30 * FRAME_DT_S
    bare = replace(sample(_scripted(t, 5.0), t), camera=None)

    result = tracker.update(bare).result
    first_after = tracker.update(
        sample(_scripted(t + FRAME_DT_S, 5.0), t + FRAME_DT_S)
    ).result

    assert result.feedforward_yaw_rate_rad_s == 0.0
    assert first_after.feedforward_yaw_rate_rad_s == 0.0


def test_gap_longer_than_the_limit_restarts_the_estimate():
    tracker, _ = _tracker()
    for index in range(30):
        t = index * FRAME_DT_S
        tracker.update(sample(_scripted(t, 5.0), t))
    later = 30 * FRAME_DT_S + CameraFeedforwardConfig().max_gap_s + 0.01

    result = tracker.update(sample(_scripted(later, 5.0), later)).result

    assert result.feedforward_yaw_rate_rad_s == 0.0


def test_steep_elevation_withholds_feedforward():
    tracker, _ = _tracker()
    limit = CameraFeedforwardConfig().max_abs_elevation_deg
    for index in range(10):
        t = index * FRAME_DT_S
        state = WorldState(0.0, 0.0, 0.0, 5.0, 0.0, -limit - 2.0, 3.0, -limit - 1.0)
        result = tracker.update(sample(state, t)).result

    assert result.feedforward_yaw_rate_rad_s == 0.0
    assert result.feedforward_pitch_rate_rad_s == 0.0


def test_failed_actuation_commits_no_feedforward_state():
    tracker, actuator = _tracker()
    reference, reference_actuator = _tracker()
    for index in range(20):
        t = index * FRAME_DT_S
        tracker.update(sample(_scripted(t, 5.0), t))
        reference.update(sample(_scripted(t, 5.0), t))
    t = 20 * FRAME_DT_S
    actuator.fail_next = True
    with pytest.raises(OSError):
        tracker.update(sample(_scripted(t, 5.0), t))

    retried = tracker.update(sample(_scripted(t, 5.0), t)).result
    expected = reference.update(sample(_scripted(t, 5.0), t)).result

    assert retried == expected
    assert actuator.commands[-1] == reference_actuator.commands[-1]


def test_heading_never_enters_feedforward():
    """The same frames under any heading offset give the same feedforward."""
    def shifted(offset: float):
        def world(t: float) -> WorldState:
            state = _scripted(t, 4.0, 1.0)
            return replace(
                state,
                heading_deg=wrap_deg(state.heading_deg + offset),
                camera_yaw_deg=state.camera_yaw_deg + offset,
                los_azimuth_deg=state.los_azimuth_deg + offset,
            )
        return world

    base = _open_loop(4.0, shifted(0.0))
    for offset in (90.0, -137.0, 179.0):
        rows = _open_loop(4.0, shifted(offset))
        for (_, yaw, pitch), (_, yaw0, pitch0) in zip(rows, base):
            assert yaw == pytest.approx(yaw0, abs=1e-9)
            assert pitch == pytest.approx(pitch0, abs=1e-9)
