"""Vehicle compatibility boundary for legacy Roll-L1 inputs."""

from __future__ import annotations

from navpy.args.navigation_args import NavigationArgs
from navpy.modules.navigation.nav.roll_l1 import RollL1Control
from navpy.modules.navigation.nav.roll_l1_inputs import (
    PitchLockSettings,
    RollL1AirframeLimits,
    RollL1AttitudeSnapshot,
    RollL1MotionSnapshot,
)
from navpy.modules.navigation.nav.roll_l1_core import (
    PitchLockPolicy,
    RollL1PitchNav,
    create_pitch_controller,
)
from navpy.modules.navigation.nav.roll_l1_pitch_nav_law import RollL1PitchNavLaw
from navpy.modules.navigation.nav.roll_l1_state import RollL1Parameters
from navpy.modules.vehicle.vehicle_interface import IVehicle


def compose_roll_l1_nav(
    vehicle: IVehicle,
    args: NavigationArgs,
    *,
    controller_name: str | None = None,
) -> RollL1PitchNav:
    def parameters() -> RollL1Parameters:
        return RollL1Parameters(
            period=vehicle.get_param_or_default("NAVL1_PERIOD", 17),
            damping=vehicle.get_param_or_default("NAVL1_DAMPING", 0.75),
            xtrack_i_gain=vehicle.get_param_or_default(
                "NAVL1_XTRACK_I",
                0.02,
            ),
            pitch_min=vehicle.min_pitch,
            pitch_max=vehicle.max_pitch,
        )

    def motion() -> RollL1MotionSnapshot:
        attitude = vehicle.attitude
        velocity = vehicle.velocity
        return RollL1MotionSnapshot(
            yaw_deg=float(attitude.yaw),
            heading_deg=float(vehicle.heading),
            velocity_ne_mps=(float(velocity[0]), float(velocity[1])),
        )

    def attitude() -> RollL1AttitudeSnapshot:
        value = vehicle.attitude
        return RollL1AttitudeSnapshot(
            pitch_deg=float(value.pitch),
            roll_deg=float(value.roll),
        )

    def limits() -> RollL1AirframeLimits:
        return RollL1AirframeLimits(
            pitch_min_deg=float(vehicle.min_pitch),
            pitch_max_deg=float(vehicle.max_pitch),
            trim_throttle_percent=float(
                vehicle.get_param_or_default("TRIM_THROTTLE", 50)
            ),
        )

    def lock_settings() -> PitchLockSettings:
        return PitchLockSettings(
            termination_angle_deg=float(args.termination_angle),
            lock_distance_m=float(args.pitch_lock_dist),
            roll_difference_deg=float(args.pitch_lock_roll_diff),
        )

    initial_limits = limits()
    pitch = create_pitch_controller(
        controller_name or args.pitch_controller,
        args.pitch_args,
        lambda: float(args.pitch_args.kp),
        initial_limits.pitch_min_deg,
        initial_limits.pitch_max_deg,
    )
    return RollL1PitchNav(
        RollL1Control(parameters, motion),
        pitch,
        PitchLockPolicy(lock_settings),
        attitude,
        limits,
        lambda: args.termination_throttle,
    )


def compose_roll_l1_law(
    vehicle: IVehicle,
    args: NavigationArgs,
) -> RollL1PitchNavLaw:
    return RollL1PitchNavLaw(compose_roll_l1_nav(vehicle, args))


__all__ = ["compose_roll_l1_law", "compose_roll_l1_nav"]
