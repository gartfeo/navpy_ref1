"""NAV recovery's view of one vehicle (core/flow/nav_recovery.py): its bus's command fence and its latest heartbeat."""

from __future__ import annotations

from navpy.core.flow.nav_ownership import ModeObservation
from navpy.core.flow.nav_recovery import NavRecoveryLink
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence


def nav_recovery_link(fence: MavCommandFence, vehicle: IVehicle) -> NavRecoveryLink:
    """The heartbeat count is read first, so a newer count means a mode at least that new."""

    def observe_mode() -> ModeObservation:
        count = vehicle.heartbeat_count
        return ModeObservation(
            heartbeat_count=count,
            mode=vehicle.get_mode,
            armed=vehicle.is_armed,
            fresh=vehicle.link_ok,
        )

    return NavRecoveryLink(close_fence=fence.close, reopen_fence=fence.reopen, observe_mode=observe_mode)


__all__ = ["NavRecoveryLink", "nav_recovery_link"]
