"""Entry-bound mission validation and background probe lifecycle."""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gcs.backend.vehicle_manager import VehicleManager
    from gcs.backend.mission_validator import MissionProbe

log = logging.getLogger("gcs.backend.vehicle_manager")

import threading
import uuid

def _probe_mission_async(manager: VehicleManager, entry: "VehicleEntry") -> None:
    """Run probe_existing_mission on a daemon thread (off the connect path).

    Concurrent per-vehicle probes are safe: different vehicles use disjoint
    MavMission loaders, and same-vehicle mission transactions serialize on
    VehicleMav._mission_lock (so this probe can't corrupt a concurrent
    frontend /mission download). Nothing is gated on the result — the launch
    path re-probes any vehicle still unverified (routes/control.py), so a
    slow/failed probe cannot let an unvalidated mission through.

    The probe is bound to *this* entry: if a disconnect+reconnect replaces it
    (same sys_id, new VehicleEntry) before/while we run, we abort rather than
    probe or write onto the wrong entry.
    """
    # Flag "downloading" synchronously (before the daemon thread starts) so
    # there is no gap where the card would briefly read "ready" between
    # connect and the probe actually running. The thread's finally always
    # clears it; a reconnect discards the whole entry anyway.
    entry.is_probing = True
    from gcs.backend.diagnostics import emit
    operation_id = uuid.uuid4().hex
    emit("mission_probe_started", source="backend", sys_id=entry.sys_id,
         operation_id=operation_id)

    def _run() -> None:
        try:
            if manager.get_vehicle(entry.sys_id) is not entry:
                return  # already replaced by a reconnect
            manager.probe_existing_mission(entry.sys_id, expected=entry)
        except Exception as e:  # daemon thread — never let it escape
            log.warning("Async mission probe for vehicle %d ended: %s", entry.sys_id, e)
        finally:
            entry.is_probing = False
            emit("mission_probe_finished", source="backend", sys_id=entry.sys_id,
                 operation_id=operation_id,
                 outcome="success" if entry.cached_mission else "failure",
                 total=len((entry.cached_mission or {}).get("waypoints") or []))

    try:
        threading.Thread(
            target=_run, name=f"mission-probe-{entry.sys_id}", daemon=True,
        ).start()
    except Exception as e:
        # The finally that clears is_probing lives inside _run, so if the OS
        # can't even start the thread (thread exhaustion under load), the flag
        # would stick True forever and pin the card on "Downloading…". Clear it
        # here. Fail-open: the vehicle stays connected and the launch path
        # re-probes any still-unverified mission before it matters.
        entry.is_probing = False
        log.warning("Could not start mission-probe thread for vehicle %d: %s", entry.sys_id, e)

def probe_existing_mission(
    manager: VehicleManager,
    sys_id: int,
    expected: 'VehicleEntry | None' = None,
) -> MissionProbe:
    """Download and validate a vehicle's existing mission.

    If valid, marks mission_uploaded=True and caches the parsed mission
    so the download endpoint can serve it without re-downloading.
    Returns a MissionProbe dataclass.

    Runs synchronously; connect defers this via _probe_mission_async, while
    the launch path (routes/control.py) calls it directly to re-validate.

    *expected*: when given (async connect path), abort if the current entry
    for *sys_id* is not that exact object — a disconnect+reconnect may have
    replaced it. The result is also only written back if the entry is still
    current after the (seconds-long) download, so a reconnect mid-probe never
    lands stale mission state on a live entry.
    """
    from gcs.backend.mission_validator import validate_vehicle_mission, MissionProbe
    entry = manager.get_vehicle(sys_id)
    if not entry or (expected is not None and entry is not expected):
        return MissionProbe(valid=False)

    # Track the exact list we last published so our clear only nulls OUR
    # progress. A concurrent download of the same vehicle (e.g. the /mission
    # GET, if it runs its own download) may take over the field after our
    # last item, and our finally must not wipe its live count to None.
    _last = [None]

    def _progress(current: int, total: int) -> None:
        p = [current, total]
        _last[0] = p
        entry.mission_download_progress = p

    try:
        probe, cached = validate_vehicle_mission(entry.vehicle, sys_id, on_progress=_progress)
    finally:
        if entry.mission_download_progress is _last[0]:
            entry.mission_download_progress = None
    if manager.get_vehicle(sys_id) is not entry:
        # Replaced during the download (disconnect+reconnect). The probe
        # validated the OLD entry, so it says nothing about the new one —
        # return invalid (fail-closed) rather than a stale "valid" that a
        # caller (e.g. launch readiness) might apply to the reconnected
        # vehicle, and don't write onto the replaced entry.
        return MissionProbe(valid=False)
    if probe.valid:
        entry.mission_uploaded = True
        entry.probed_search_pattern = probe.search_pattern
        entry.cached_mission = cached
        log.info(
            "Vehicle %d: existing mission valid (%d track wps, search_pattern=%s)",
            sys_id, probe.track_count, probe.search_pattern,
        )
    else:
        log.info("Vehicle %d: no valid existing mission", sys_id)
    return probe
