"""Step 1: verify the actuator-form NavLaw wrapper preserves RollL1PitchNav behavior.

Tests run the wrapper and a directly-constructed inner RollL1PitchNav through
identical contexts and assert bit-identical (roll, pitch, thr) outputs.

Determinism:
  - NAVL1_XTRACK_I is 0 so the L1 xtrack integrator short-circuits (no time
    dependency in the integral branch).
  - PID pitch uses ki=0 and kd=0 (configured via PIDArgs defaults).
  - PN pitch is time-independent.
"""
from __future__ import annotations

import argparse
import math
import unittest
from unittest.mock import Mock

import numpy as np

from navpy.args.navigation_args import NavigationArgs
from navpy.args.navigation_poi_args import NavigationPoiArgs
from navpy.args.mission_planner_args import MissionPlannerArgs
from navpy.args.pid_args import PIDArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.nav_law import (
    NavCommand,
    NavCommandMode,
    NavContext,
    NavInvalidReason,
)
from navpy.modules.navigation.nav.roll_l1_pitch_nav import (
    PitchPidController,
)
from navpy.modules.navigation.nav.roll_l1_composition import (
    compose_roll_l1_law,
    compose_roll_l1_nav,
)


def _build_args(pitch_controller: str):
    """Build an argparse namespace with deterministic test settings.

    Routes ``pitch_controller`` through the real CLI flag so
    ``StoreWithFlag`` records it in ``_cli_overrides``; otherwise
    ``NavigationArgs._get_param`` reads from the mocked MAVLink param
    (which always returns 1 = PN) and both parity sweeps end up
    exercising PN, leaving the PID wrapper untested.
    """
    parser = argparse.ArgumentParser()
    NavigationArgs.add_args(parser)
    NavigationPoiArgs.add_args(parser)
    PIDArgs.add_args(parser, "pitch")
    PIDArgs.add_args(parser, "roll")
    MissionPlannerArgs.add_args(parser)
    args, _ = parser.parse_known_args(["--pitch-controller", pitch_controller])
    # PIDArgs defaults are kp=1.2/0.0, ki=0.0, kd=0.0 — already deterministic.
    return args


def _build_vehicle() -> Mock:
    """Mock IVehicle with deterministic parameter lookups.

    NAVL1_XTRACK_I=0 disables the time-dependent L1 integral branch.
    """
    vehicle = Mock()
    vehicle.min_pitch = -60
    vehicle.max_pitch = 60
    vehicle.velocity = [20.0, 0.0, 0.0]
    vehicle.attitude = Attitude(pitch=-5.0, roll=2.0, yaw=45.0)
    vehicle.heading = 45.0
    vehicle.wind = None
    vehicle.mission_items_count = 0

    def param_or_default(name, default):
        values = {
            "NAVL1_PERIOD": 17,
            "NAVL1_DAMPING": 0.75,
            "NAVL1_XTRACK_I": 0.0,
            "TRIM_THROTTLE": 50,
            "AAS_DEL_PITCH": 0,
            "AAS_DEL_THR": -1,
            "AAS_DEL_DIR": False,
            "AAS_DEL_P_KP": 1.5,
            "AAS_DEL_CTRL": 1,
            "AAS_USE_TRN": True,
            "AAS_DEL_PLD": 100,
            "AAS_DEL_PLRD": 2.0,
        }
        return values.get(name, default)

    vehicle.get_param_or_default = Mock(side_effect=param_or_default)
    vehicle.get_parameter = Mock(side_effect=lambda name: param_or_default(name, None))
    return vehicle


def _build_ctx(seed: int) -> NavContext:
    rng = np.random.default_rng(seed)
    lat_offset = rng.uniform(-1e-3, 1e-3)
    lng_offset = rng.uniform(-1e-3, 1e-3)
    prev = Location(lat=40.0, lng=44.0, alt=500.0)
    curr = Location(lat=40.0 + lat_offset, lng=44.0 + lng_offset, alt=500.0)
    nxt = Location(lat=40.0 + 2 * lat_offset, lng=44.0 + 2 * lng_offset, alt=500.0)
    bearing_cd = float(rng.uniform(0, 36000))
    poi_ned = rng.uniform(-500, 500, size=3)
    pitch_error = float(rng.uniform(-30, 30))
    distance = float(rng.uniform(50, 1500))
    return NavContext(
        prev_loc=prev,
        current_loc=curr,
        next_loc=nxt,
        target_bearing_cd=bearing_cd,
        poi_ned=poi_ned,
        pitch_error=pitch_error,
        distance=distance,
    )


def _assert_same_output(inner_out, wrapper_out: NavCommand):
    inner_roll, inner_pitch, inner_thr = inner_out
    assert wrapper_out.mode == NavCommandMode.ACTUATOR
    # Direct float equality: the wrapper must not lose or re-round values.
    assert wrapper_out.cmd_roll_deg == inner_roll
    assert wrapper_out.cmd_pitch_deg == inner_pitch
    assert wrapper_out.cmd_thr == inner_thr


class AdapterMatchesInnerTests(unittest.TestCase):
    def _run_parity_sweep(self, pitch_controller: str):
        args = _build_args(pitch_controller)
        v_inner = _build_vehicle()
        v_wrapper = _build_vehicle()

        gn_inner = NavigationArgs(args, v_inner, Mock())
        gn_wrapper = NavigationArgs(args, v_wrapper, Mock())

        inner = compose_roll_l1_nav(v_inner, gn_inner)
        wrapper = compose_roll_l1_law(v_wrapper, gn_wrapper)

        for seed in range(10):
            ctx = _build_ctx(seed)
            inner_out = inner.calc(
                prev_loc=ctx.prev_loc,
                current_loc=ctx.current_loc,
                next_loc=ctx.next_loc,
                target_bearing_cd=ctx.target_bearing_cd,
                poi_ned=ctx.poi_ned,
                pitch_error=ctx.pitch_error,
                distance=ctx.distance,
            )
            wrapper_out = wrapper.calc(ctx)
            _assert_same_output(inner_out, wrapper_out)

    def test_adapter_matches_inner_pn(self):
        self._run_parity_sweep("pn")

    def test_adapter_matches_inner_pid(self):
        self._run_parity_sweep("pid")

    def test_command_mode_is_actuator(self):
        args = _build_args("pn")
        vehicle = _build_vehicle()
        gn = NavigationArgs(args, vehicle, Mock())
        wrapper = compose_roll_l1_law(vehicle, gn)
        cmd = wrapper.calc(_build_ctx(seed=0))
        self.assertEqual(cmd.mode, NavCommandMode.ACTUATOR)
        self.assertTrue(cmd.valid)
        self.assertIsNotNone(cmd.cmd_roll_deg)
        self.assertIsNotNone(cmd.cmd_pitch_deg)

    def test_reset_delegates(self):
        """After reset, wrapper must produce identical output to a freshly constructed one."""
        args = _build_args("pn")
        vehicle_a = _build_vehicle()
        vehicle_b = _build_vehicle()
        gn_a = NavigationArgs(args, vehicle_a, Mock())
        gn_b = NavigationArgs(args, vehicle_b, Mock())

        dirty = compose_roll_l1_law(vehicle_a, gn_a)
        # Run a few calls to build up internal state (L1 previous-turn memory, pitch lock).
        for seed in range(5):
            dirty.calc(_build_ctx(seed))

        dirty.reset()
        fresh = compose_roll_l1_law(vehicle_b, gn_b)

        # After reset, first outputs should match a fresh instance exactly.
        for seed in range(3):
            ctx = _build_ctx(seed + 100)
            out_dirty = dirty.calc(ctx)
            out_fresh = fresh.calc(ctx)
            self.assertEqual(out_dirty.cmd_roll_deg, out_fresh.cmd_roll_deg)
            self.assertEqual(out_dirty.cmd_pitch_deg, out_fresh.cmd_pitch_deg)
            self.assertEqual(out_dirty.cmd_thr, out_fresh.cmd_thr)


class NavCommandInvariantTests(unittest.TestCase):
    def test_actuator_requires_roll_and_pitch(self):
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.ACTUATOR, cmd_roll_deg=1.0)
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.ACTUATOR, cmd_pitch_deg=1.0)

    def test_actuator_rejects_accel_field(self):
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.ACTUATOR,
                cmd_roll_deg=1.0,
                cmd_pitch_deg=1.0,
                a_cmd_ned=np.zeros(3),
            )

    def test_actuator_rejects_reason(self):
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.ACTUATOR,
                cmd_roll_deg=1.0,
                cmd_pitch_deg=1.0,
                reason=NavInvalidReason.V_NAVIGATION_INVALID,
            )

    def test_acceleration_requires_vector(self):
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.ACCELERATION)

    def test_acceleration_requires_shape_3(self):
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.ACCELERATION, a_cmd_ned=np.zeros(2))
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.ACCELERATION, a_cmd_ned=np.zeros((3, 1)))

    def test_acceleration_rejects_actuator_fields(self):
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.ACCELERATION,
                a_cmd_ned=np.zeros(3),
                cmd_roll_deg=1.0,
            )
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.ACCELERATION,
                a_cmd_ned=np.zeros(3),
                cmd_thr=50.0,
            )

    def test_invalid_requires_reason(self):
        with self.assertRaises(ValueError):
            NavCommand(mode=NavCommandMode.INVALID)

    def test_invalid_rejects_command_fields(self):
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.INVALID,
                reason=NavInvalidReason.DETECTION_LOST,
                cmd_roll_deg=1.0,
                cmd_pitch_deg=1.0,
            )
        with self.assertRaises(ValueError):
            NavCommand(
                mode=NavCommandMode.INVALID,
                reason=NavInvalidReason.DETECTION_LOST,
                a_cmd_ned=np.zeros(3),
            )

    def test_valid_property_tracks_mode(self):
        ok = NavCommand(
            mode=NavCommandMode.ACTUATOR, cmd_roll_deg=0.0, cmd_pitch_deg=0.0
        )
        self.assertTrue(ok.valid)
        bad = NavCommand(
            mode=NavCommandMode.INVALID, reason=NavInvalidReason.EKF_DIVERGED
        )
        self.assertFalse(bad.valid)


class PitchPidPursuitTests(unittest.TestCase):
    """PID pitch = LOS feedforward + PID on the residual.

    Without the feedforward a proportional tracker settles at a fraction
    of the LOS depression and can never close on the POI — the vehicle passes
    the POI and decays into a stable orbit (2026-06-12 run 181207).
    ``pitch_error`` is the residual off the LOS (zero when the nose is
    on it).
    """

    def _make_controller(self):
        args = _build_args("pid")
        vehicle = _build_vehicle()
        gn = NavigationArgs(args, vehicle, Mock())
        controller = PitchPidController(
            gn.pitch_args,
            vehicle.min_pitch,
            vehicle.max_pitch,
        )
        return controller, controller._pid._Kp

    @staticmethod
    def _ned(los_deg: float, slant: float = 500.0) -> np.ndarray:
        los = math.radians(los_deg)
        return np.array([
            slant * math.cos(los), 0.0, slant * math.sin(los),
        ])

    def test_on_los_commands_pure_pursuit(self):
        controller, _ = self._make_controller()
        cmd = controller.calc(self._ned(9.3), pitch_error=0.0, current_pitch=-9.3)
        self.assertAlmostEqual(cmd, -9.3, places=5)

    def test_too_shallow_commands_more_nose_down(self):
        # The final-approach-orbit sample: los ~9 deg, nose 6.8 deg above the
        # LOS. The old tracker commanded ~-3 deg and orbited; pursuit
        # commands below the LOS.
        controller, kp = self._make_controller()
        cmd = controller.calc(self._ned(9.0), pitch_error=6.8, current_pitch=-2.4)
        self.assertAlmostEqual(cmd, -9.0 - kp * 6.8, places=5)
        self.assertLess(cmd, -9.0)

    def test_too_steep_relaxes_toward_los(self):
        # NAV-entry geometry: nose 27.5 deg below the LOS — pursuit
        # relaxes the dive instead of deepening it.
        controller, kp = self._make_controller()
        cmd = controller.calc(self._ned(9.3), pitch_error=-27.5, current_pitch=-36.8)
        self.assertAlmostEqual(cmd, -9.3 + kp * 27.5, places=5)
        self.assertGreater(cmd, -9.3)

    def test_steep_final_approach_los_tracks_full_dive(self):
        # Close-in over the POI the LOS steepens toward vertical: the
        # commanded pitch follows it (the nav law clips to airframe
        # limits downstream).
        controller, _ = self._make_controller()
        cmd = controller.calc(self._ned(80.0), pitch_error=0.0, current_pitch=-80.0)
        self.assertAlmostEqual(cmd, -80.0, places=5)


if __name__ == "__main__":
    unittest.main()
