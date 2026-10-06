"""Vehicle connection-pool ownership and operator telemetry coordination."""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional
from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_GCS, MAV_COMP_ID_MISSIONPLANNER
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.mav_bus import MavBus, MavBusClosedError
from gcs.backend.vehicle_entry import VehicleEntry
from gcs.backend import vehicle_connections, vehicle_mission_probe
from gcs.backend.vehicle_status import (
    GIMBAL_TELEMETRY_STALE_S, CAMERA_OPTICS_STALE_S,
    COMPANION_OK_WINDOW_S, COMPANION_DOWN_WINDOW_S, RC_CHANNELS_MAX,
    ACCEL_CAL_POS_LEVEL, ACCEL_CAL_POS_LEFT, ACCEL_CAL_POS_RIGHT,
    ACCEL_CAL_POS_NOSEDOWN, ACCEL_CAL_POS_NOSEUP, ACCEL_CAL_POS_BACK,
    ACCEL_CAL_ACTIVE_WINDOW_S, CONFIRM_BLOCKED_PREFIX, CONFIRM_BLOCKED_STALE_S,
    _classify_accel_cal_statustext,
)
from gcs.backend.vehicle_camera import (
    _coerce_gimbal_quaternion, _gimbal_device_id_from_camera_component,
    _coerce_fov_degrees_to_rad, _coerce_positive_float,
)

log = logging.getLogger(__name__)


class VehicleManager:
    """Own the vehicle pool, its buses, listeners and telemetry loop."""

    def __init__(self):
        self._vehicles: dict[int, VehicleEntry] = {}
        self._lock = threading.RLock()
        self._buses: dict[str, MavBus] = {}  # device -> MavBus
        self._telemetry_loop = None  # lazy: set by start_telemetry_loop
        self._task_confirm_listener = None
        self._task_assign_listener = None
        self._compass_cal_listener = None

    def set_task_confirm_listener(self, listener) -> None:
        """Set the TaskConfirmListener for new vehicle registrations."""
        self._task_confirm_listener = listener

    def get_task_confirm_listener(self):
        """Return the active TaskConfirmListener, or None if not yet set.

        Used by the /task_confirm route to clear the listener's (sys_id,
        task_id) popup-dedup entry (D-10) once a response has been sent for
        a confirm round.
        """
        return self._task_confirm_listener

    def set_task_assign_listener(self, listener) -> None:
        """Set the TaskAssignListener for new vehicle registrations."""
        self._task_assign_listener = listener

    def set_compass_cal_listener(self, listener) -> None:
        """Set the CompassCalListener for new vehicle registrations."""
        self._compass_cal_listener = listener

    @property
    def vehicles(self) -> dict[int, VehicleEntry]:
        return dict(self._vehicles)

    def _get_or_create_bus(self, device: str) -> MavBus:
        """Get or create a MavBus for *device*, tracking it locally."""
        with self._lock:
            bus = self._buses.get(device)
            if bus is not None and (
                getattr(bus, "is_closed", False) or getattr(bus, "is_closing", False)
            ):
                self._buses.pop(device, None)
                bus = None
            if bus is None:
                bus = MavBus.get_or_create(device, source_system=255, source_component=0)
                self._buses[device] = bus
            return bus

    def _is_current_bus(self, device: str, bus: MavBus) -> bool:
        with self._lock:
            return self._buses.get(device) is bus

    def _pop_bus_if_current(self, device: str, bus: MavBus) -> None:
        with self._lock:
            if self._buses.get(device) is bus:
                self._buses.pop(device, None)

    def _drop_stale_bus(self, device: str) -> None:
        with self._lock:
            bus = self._buses.get(device)
            if bus is not None and (
                getattr(bus, "is_closed", False) or getattr(bus, "is_closing", False)
            ):
                self._buses.pop(device, None)

    def add_vehicle(self, device: str, sys_id: int, name: Optional[str] = None) -> VehicleEntry:
        """Connect to a vehicle and add to the pool.

        MavBus handles connection sharing automatically.
        """
        with self._lock:
            if sys_id in self._vehicles:
                log.warning("Vehicle %d already connected", sys_id)
                return self._vehicles[sys_id]

            vname = name or f"s1-u{sys_id}"
            log.info("Adding vehicle %d on %s...", sys_id, device)
            for attempt in range(2):
                bus = self._get_or_create_bus(device)
                try:
                    vehicle = VehicleMav(
                        device=device,
                        target_system=sys_id,
                        skip_mission_download=True,
                        wait_heartbeat=False,
                        send_heartbeat=True,
                        heartbeat_hz=1.0,
                        heartbeat_timeout=5.0,
                        mav_type=MAV_TYPE_GCS,
                        mav_comp_id=MAV_COMP_ID_MISSIONPLANNER,
                        bus=bus,
                    )
                    break
                except MavBusClosedError:
                    self._drop_stale_bus(device)
                    if attempt >= 1:
                        raise
                    log.info("Vehicle %d connect hit closing bus; retrying with fresh bus", sys_id)
            entry = VehicleEntry(sys_id, vehicle, vname, device=device)
            self._vehicles[sys_id] = entry
        if self._task_confirm_listener:
            self._task_confirm_listener.register_vehicle(sys_id, vehicle)
        if self._task_assign_listener:
            self._task_assign_listener.register_vehicle(sys_id, vehicle)
        if self._compass_cal_listener:
            self._compass_cal_listener.register_vehicle(sys_id, vehicle)
        log.info("Vehicle %d (%s) connected", sys_id, vname)
        # Probe the existing mission OFF the connect path. The probe is a
        # blocking MAVLink mission download (seconds); running it inline made a
        # vehicle — and every later vehicle in a serial discover — invisible in
        # the UI until its full mission finished downloading. Deferring it lets
        # the entry register (and its card render) immediately; mission_uploaded
        # / cached_mission / probed_search_pattern fill in when the probe completes.
        self._probe_mission_async(entry)
        return entry

    def scan(self, device: str, timeout: float = 5.0) -> list[dict]:
        return vehicle_connections.scan(self, device, timeout)

    def discover_and_connect(
        self, device: str, timeout: float = 5.0,
    ) -> list[VehicleEntry]:
        return vehicle_connections.discover_and_connect(self, device, timeout)

    def reconnect_vehicle(self, sys_id: int, timeout: float = 10.0) -> Optional[VehicleEntry]:
        return vehicle_connections.reconnect_vehicle(self, sys_id, timeout)

    def remove_vehicle(self, sys_id: int) -> list[int]:
        return vehicle_connections.remove_vehicle(self, sys_id)

    def _probe_mission_async(self, entry: "VehicleEntry") -> None:
        return vehicle_mission_probe._probe_mission_async(self, entry)

    def probe_existing_mission(self, sys_id: int, expected: "VehicleEntry | None" = None) -> MissionProbe:
        return vehicle_mission_probe.probe_existing_mission(self, sys_id, expected)

    def get_vehicle(self, sys_id: int) -> Optional[VehicleEntry]:
        return self._vehicles.get(sys_id)

    def is_companion_active(self, sys_id: int, timeout: float = 5.0) -> bool:
        """Whether a companion heartbeat for *sys_id* arrived within *timeout*.

        NOTE: no production caller. Liveness now uses the tri-state
        :meth:`companion_status` (``get_all_snapshots`` derives ``companion_ok``
        from it). This binary check is retained only as a regression reference
        for the pre-tri-state 5 s behavior — the tests use it to witness the old
        flap (a ~10 s gap it reports inactive is now "checking", not faulty).
        """
        now = time.time()
        for bus in self._buses.values():
            ts = bus.companion_heartbeats.get(sys_id)
            if ts is not None and now - ts <= timeout:
                return True
        return False

    def _newest_companion_hb(self, sys_id: int) -> Optional[float]:
        """Most recent companion-heartbeat timestamp for *sys_id* across all buses.

        The companion heartbeat now carries the same system id as the
        aircraft's own; MavBus keeps the two apart by MAV_TYPE
        (MAV_TYPE_ONBOARD_CONTROLLER lands in ``companion_heartbeats``,
        everything else in ``heartbeats``), so the sysid is a safe key here.
        """
        newest: Optional[float] = None
        for bus in self._buses.values():
            ts = bus.companion_heartbeats.get(sys_id)
            if ts is not None and (newest is None or ts > newest):
                newest = ts
        return newest

    def companion_status(self, sys_id: int) -> str:
        """Tri-state companion liveness: ``"ok"`` | ``"checking"`` | ``"down"``.

        See COMPANION_OK_WINDOW_S / COMPANION_DOWN_WINDOW_S for the rationale:
        a brief gap in the 1 Hz companion heartbeat on the shared lossy link is
        reported as ``"checking"`` (transient, not faulty) rather than flapping
        straight to ``"down"``.
        """
        newest = self._newest_companion_hb(sys_id)
        if newest is None:
            return "down"
        age = time.time() - newest
        if age <= COMPANION_OK_WINDOW_S:
            return "ok"
        if age <= COMPANION_DOWN_WINDOW_S:
            return "checking"
        return "down"

    def get_all_snapshots(self) -> list[dict]:
        """Get telemetry snapshots for all connected vehicles."""
        snapshots = []
        for entry in list(self._vehicles.values()):
            try:
                snap = entry.snapshot()
                status = self.companion_status(entry.sys_id)
                snap["companion_status"] = status
                # "checking" is transient loss, not a fault: keep companion_ok
                # truthy so launch-readiness (prearmChecks) only trips on a
                # genuine, sustained "down".
                snap["companion_ok"] = status != "down"
                snapshots.append(snap)
            except Exception as e:
                log.error("Snapshot error for vehicle %d: %s", entry.sys_id, e)
        return snapshots

    def get_seen_ids(self, timeout: float = 5.0) -> list[int]:
        return vehicle_connections.get_seen_ids(self, timeout)

    async def start_telemetry_loop(self):
        """Start the async telemetry broadcast loop (delegates to TelemetryLoop)."""
        from gcs.backend.telemetry_loop import TelemetryLoop
        self._telemetry_loop = TelemetryLoop(self)
        await self._telemetry_loop.start()

    async def stop_telemetry_loop(self):
        """Stop the telemetry loop."""
        if self._telemetry_loop:
            await self._telemetry_loop.stop()

    def shutdown(self):
        """Close all vehicle connections."""
        for sys_id in list(self._vehicles.keys()):
            self.remove_vehicle(sys_id)


vehicle_mgr = VehicleManager()
