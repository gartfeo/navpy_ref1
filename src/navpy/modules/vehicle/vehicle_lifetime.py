"""Vehicle lifetime and runtime-wide logger replacement."""
from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle


class VehicleLifetime:
    def __init__(
        self,
        lifecycle: VehicleLifecycle,
        logger_ref: LoggerRef,
    ) -> None:
        self._lifecycle = lifecycle
        self._logger_ref = logger_ref

    def wait_heartbeat_from(
        self,
        target_system: int,
        timeout: float = 30.0,
    ) -> bool:
        return self._lifecycle.wait_heartbeat_from(target_system, timeout)

    def set_logger(self, logger: ILogger) -> None:
        self._logger_ref.replace(logger)

    def close(self) -> None:
        self._lifecycle.close()
