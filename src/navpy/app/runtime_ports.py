"""What a host binds in place of NavPy's wall defaults (a lockstep participant)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.app.runtime_stop import LifecycleStopRequest
from navpy.app.wall_scheduler_cadence import speedup_scheduler_cadence
from navpy.core.kernel.logging.logger_api import ILogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus, MavBusOpener
from navpy.sim.detection.camera_frame_ports import CameraFrameFactoryPort


@dataclass(frozen=True)
class RuntimePorts:
    """What a host binds in place of the wall defaults (a lockstep participant).

    ``observe_logger`` sees the navigation logger once it exists;
    ``lifecycle_stop`` is the stop that ``--stop-on-lifecycle`` requires;
    ``camera_frames`` is the sim camera's latency and drops (dev sim step 5).
    """

    open_bus: MavBusOpener = MavBus.get_or_create
    scheduler_cadence: Callable[[IVehicle, MavBus], SchedulerCadence] = speedup_scheduler_cadence
    observe_logger: Callable[[ILogger], None] = field(default=lambda _logger: None)
    lifecycle_stop: LifecycleStopRequest | None = None
    camera_frames: CameraFrameFactoryPort | None = None


__all__ = ["RuntimePorts"]
