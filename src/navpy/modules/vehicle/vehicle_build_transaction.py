"""Transactional ownership for staged vehicle construction."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

from navpy.modules.vehicle.mav_bus import MavBus, MavBusLease
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleBuildRequest,
    VehicleParts,
    VehicleProtocolParts,
)
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle
from navpy.modules.vehicle.vehicle_capability_composition import (
    build_capability_parts,
)
from navpy.modules.vehicle.vehicle_foundation_composition import (
    build_vehicle_foundation,
)
from navpy.modules.vehicle.vehicle_protocol_composition import (
    build_protocol_parts,
)
from navpy.modules.vehicle.vehicle_runtime_composition import (
    assemble_vehicle_parts,
)


def _raise_build_rollback(
    primary: BaseException,
    cleanup: BaseException,
) -> None:
    errors = [primary, cleanup]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("vehicle build and rollback failed", errors)
    raise BaseExceptionGroup("vehicle build and rollback failed", errors)


class VehicleBuildTransaction:
    """Prepare, execute, and transfer one vehicle's resource ownership."""

    def __init__(self, request: VehicleBuildRequest) -> None:
        self._request = request
        self._active_bus: MavBus | None = None
        self._bus_lease: MavBusLease | None = None
        self._lifecycle: VehicleLifecycle | None = None
        self._phase = "new"

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def has_pending_ownership(self) -> bool:
        return self._phase in {"prepared", "executing", "rollback_failed"}

    def prepare(self) -> None:
        if self._phase != "new":
            raise RuntimeError(f"cannot prepare vehicle build in {self._phase}")
        link = self._request.link
        self._active_bus = (
            link.bus
            if link.bus is not None
            else MavBus.get_or_create(
                link.device,
                link.baud,
                link.target_system,
                link.mav_comp_id,
            )
        )
        self._bus_lease = self._active_bus.reserve(link.target_system)
        self._phase = "prepared"

    def execute(self) -> VehicleParts:
        if (
            self._phase != "prepared"
            or self._active_bus is None
            or self._bus_lease is None
        ):
            raise RuntimeError("vehicle build transaction was not prepared")
        self._phase = "executing"
        foundation = build_vehicle_foundation(
            self._request,
            self._active_bus,
            self._bus_lease,
        )
        protocols = build_protocol_parts(self._request, foundation)
        self._lifecycle = protocols.lifecycle
        capabilities = build_capability_parts(foundation, protocols)
        parts = assemble_vehicle_parts(foundation, protocols, capabilities)
        self._activate(protocols, parts)
        return parts

    def commit(self, parts: VehicleParts) -> VehicleParts:
        if self._phase != "executing":
            raise RuntimeError(f"cannot commit vehicle build in {self._phase}")
        self._phase = "committed"
        return parts

    def rollback(self, primary: BaseException) -> None:
        if self._phase in {"committed", "rolled_back"}:
            return
        try:
            self._release_owned_resources()
        except BaseException as cleanup:
            _raise_build_rollback(primary, cleanup)

    def retry_rollback(self) -> None:
        if self._phase != "rollback_failed":
            raise RuntimeError(f"no failed rollback to retry in {self._phase}")
        self._release_owned_resources()

    def _release_owned_resources(self) -> None:
        action = None
        if self._lifecycle is not None:
            action = self._lifecycle.close
        elif self._bus_lease is not None:
            action = self._bus_lease.release
        if action is None:
            self._phase = "rolled_back"
            return
        try:
            action()
        except BaseException:
            self._phase = "rollback_failed"
            raise
        self._phase = "rolled_back"
        self._lifecycle = None
        self._bus_lease = None
        self._active_bus = None

    def build(self) -> VehicleParts:
        try:
            self.prepare()
            return self.commit(self.execute())
        except BaseException as primary:
            self.rollback(primary)
            raise

    def _activate(
        self,
        protocols: VehicleProtocolParts,
        parts: VehicleParts,
    ) -> None:
        startup = self._request.startup
        protocols.lifecycle.attach(protocols.router)
        if (
            startup.wait_heartbeat
            and not parts.runtime.lifetime.wait_heartbeat_from(
                self._request.link.target_system,
                timeout=30.0,
            )
        ):
            raise RuntimeError(
                f"Vehicle {self._request.link.target_system} "
                "is not broadcasting heartbeats."
            )
        if startup.send_heartbeat:
            protocols.lifecycle.start_heartbeat()
        if not startup.skip_mission_download:
            protocols.mission.download()


def build_requested_vehicle(request: VehicleBuildRequest) -> VehicleParts:
    return VehicleBuildTransaction(request).build()


__all__ = ["VehicleBuildTransaction", "build_requested_vehicle"]
