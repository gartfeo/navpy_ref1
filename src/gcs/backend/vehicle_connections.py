"""Connection discovery, replacement and removal for a vehicle pool."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from gcs.backend.vehicle_entry import VehicleEntry
    from gcs.backend.vehicle_manager import VehicleManager

log = logging.getLogger("gcs.backend.vehicle_manager")


def scan(manager: VehicleManager, device: str, timeout: float = 5.0) -> list[dict]:
    """Scan a MAVLink connection for heartbeats without connecting.

    Returns list of {sys_id, name} dicts for each found vehicle.
    Uses MavBus heartbeat tracking (no race condition).
    """
    log.info("Scanning for vehicles on %s (%.1fs)...", device, timeout)
    ignore_sids = {0, 254, 255}
    ignore_sids.update(manager._vehicles.keys())

    bus = manager._get_or_create_bus(device)

    # Poll bus heartbeats for the timeout duration
    deadline = time.time() + timeout
    found: set[int] = set()
    scan_start = time.time()
    while time.time() < deadline:
        for sid, ts in list(bus.heartbeats.items()):
            if sid not in ignore_sids and ts >= scan_start:
                found.add(sid)
        time.sleep(0.2)

    if (
        getattr(bus, "is_closed", False)
        or getattr(bus, "is_closing", False)
        or not manager._is_current_bus(device, bus)
    ):
        log.info("Scan discarded: bus for %s was closed or replaced", device)
        return []

    with manager._lock:
        found.difference_update(manager._vehicles.keys())

    for sid in sorted(found):
        log.info("Discovered vehicle sys_id=%d on %s", sid, device)
    log.info("Scan complete: found %d vehicle(s) on %s", len(found), device)

    if not found:
        # Close bus if we created it and nobody else is using it
        if manager._is_current_bus(device, bus) and not bus.has_targets:
            bus.close()
            manager._pop_bus_if_current(device, bus)
        return []

    return [
        {"sys_id": sid, "name": f"s1-u{sid}"}
        for sid in sorted(found)
    ]

def discover_and_connect(
    manager: VehicleManager, device: str, timeout: float = 5.0,
) -> list[VehicleEntry]:
    """Scan for heartbeats and connect to each vehicle found."""
    results = manager.scan(device, timeout)
    entries: list[VehicleEntry] = []
    for item in results:
        try:
            entry = manager.add_vehicle(device, item["sys_id"])
            entries.append(entry)
        except Exception as e:
            log.error("Failed to connect vehicle %d: %s", item["sys_id"], e)
    return entries

def reconnect_vehicle(manager: VehicleManager, sys_id: int, timeout: float = 10.0) -> Optional[VehicleEntry]:
    """Disconnect and reconnect a vehicle.

    MavBus stays alive so other vehicles are unaffected.
    Returns the new VehicleEntry or None on failure.
    """
    with manager._lock:
        old_entry = manager._vehicles.pop(sys_id, None)
    if not old_entry:
        log.warning("reconnect_vehicle: sys_id %d not found", sys_id)
        return None

    device = old_entry.device
    name = old_entry.name

    try:
        old_entry.vehicle.close()
    except Exception as e:
        log.error("Error closing vehicle %d for reconnect: %s", sys_id, e)

    # Drop full-param subscription for the old VehicleMav so the new one
    # gets a fresh `_ensure_subscribed` registration. Lazy import.
    try:
        from gcs.backend.full_params import full_param_cache
        full_param_cache.forget_vehicle(sys_id)
    except Exception:
        pass

    try:
        new_entry = manager.add_vehicle(device, sys_id, name)
    except Exception as e:
        log.error("Failed to reconnect vehicle %d: %s", sys_id, e)
        return None

    # Poll link_ok up to timeout
    deadline = time.time() + timeout
    while time.time() < deadline:
        if new_entry.vehicle.link_ok:
            log.info("Vehicle %d reconnected", sys_id)
            return new_entry
        time.sleep(0.2)

    log.warning("Vehicle %d reconnect: link_ok timeout after %.1fs", sys_id, timeout)
    return new_entry

def remove_vehicle(manager: VehicleManager, sys_id: int) -> list[int]:
    """Disconnect a single vehicle.

    VehicleMav.close() detaches from MavBus. MavBus auto-closes
    when no targets remain.
    Returns list of removed sys_ids (always one or zero).
    """
    removed = []
    with manager._lock:
        entry = manager._vehicles.pop(sys_id, None)
    if not entry:
        return removed

    device = entry.device
    try:
        entry.vehicle.close()
    except Exception as e:
        log.error("Error closing vehicle %d: %s", sys_id, e)
    removed.append(sys_id)
    log.info("Vehicle %d removed", sys_id)

    # Drop full-param cache + token (D7). Lazy import to avoid cycle.
    try:
        from gcs.backend.full_params import full_param_cache
        full_param_cache.forget_vehicle(sys_id)
    except Exception:
        pass

    # Clean up our bus reference if MavBus closed itself
    bus = manager._buses.get(device)
    if bus and not bus.has_targets:
        manager._pop_bus_if_current(device, bus)

    return removed

def get_seen_ids(manager: VehicleManager, timeout: float = 5.0) -> list[int]:
    """Get sys_ids with a recent heartbeat on any bus.

    Includes both connected and unconnected vehicles that are alive
    on the MAVLink bus.
    """
    now = time.time()
    seen: set[int] = set()
    for bus in manager._buses.values():
        for sid, ts in list(bus.heartbeats.items()):
            if now - ts <= timeout:
                seen.add(sid)
    return sorted(seen)
