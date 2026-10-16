"""Wall mode's scheduler cadence: SIM_SPEEDUP plus the bus reader budget."""

from __future__ import annotations

from navpy.core.kernel.reader_budget_feed import ReaderBudgetFeed
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from navpy.plugins.vehicle.ardupilot.pose_streams import (
    resolve_ardupilot_scheduler_rate_hz,
)


def speedup_scheduler_cadence(vehicle: IVehicle, bus: MavBus) -> SchedulerCadence:
    """NavPy's cadence follows the SITL's ``SIM_SPEEDUP``.

    It also bounds the vehicle bus's MAVLink reader batch to one scheduler
    wall slot (docs/design/reader_batch.md); the scheduler period is
    resolved before the cadence's first speedup sync.
    """
    return SchedulerCadence(
        vehicle.sim_speedup,
        reader_budget=ReaderBudgetFeed(
            sink=bus.reader_budgets.request("scheduler cadence"),
            scheduler_period_s=1.0 / resolve_ardupilot_scheduler_rate_hz(vehicle),
        ),
    )


__all__ = ["speedup_scheduler_cadence"]
