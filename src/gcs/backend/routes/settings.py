"""Settings REST API — GET / PUT / POST reset."""
from __future__ import annotations

import asyncio
import csv
import io
import logging

from fastapi import APIRouter, UploadFile, File, HTTPException
from pydantic import ValidationError

from gcs.backend import navpy_sim_runtime as runtime
from gcs.backend.settings_model import FallbackDeliveryLocation, DELIVERY_LOCATION_TYPES, GcsSettings
from gcs.backend.settings_store import settings_store

log = logging.getLogger(__name__)
router = APIRouter()


@router.get("/api/settings")
async def get_settings():
    return _with_instance_device(settings_store.get().model_dump())


@router.put("/api/settings")
async def update_settings(body: dict):
    unknown = set(body) - set(GcsSettings.model_fields)
    if unknown:
        raise HTTPException(422, detail=f"Unknown settings fields: {', '.join(sorted(unknown))}")
    _drop_injected_instance_device(body)
    before = settings_store.get()
    try:
        updated = settings_store.update(body)
    except ValidationError as e:
        # A malformed/bad-typed field must not crash the endpoint. settings_store
        # leaves the stored settings unchanged when validation fails, so return a
        # graceful 422 with the field errors (sanitized to JSON-safe fields only —
        # pydantic's ctx can hold non-serializable exception objects).
        raise HTTPException(
            status_code=422,
            detail=[
                {"loc": list(err.get("loc", ())), "msg": err.get("msg", ""),
                 "type": err.get("type", "")}
                for err in e.errors()
            ],
        )
    await _stop_navpy_if_sim_mode_disabled(before, updated)
    await _restart_navpy_if_profile_changed(before, updated)
    return _with_instance_device(updated.model_dump())


@router.post("/api/settings/reset")
async def reset_settings():
    restored = settings_store.reset()
    return _with_instance_device(restored.model_dump())


def _with_instance_device(data: dict) -> dict:
    """In launcher-managed sim mode, overlay this chat's real ports onto the API
    response: ``connection.default_device`` -> the chat's GCS monitor (so
    auto-connect targets the right multiplexed UDP port), and
    ``simulation.sitl_presets`` -> the chat's per-vehicle companion UDP
    connections (so the Connection tab shows the true instances). Applied only to
    API responses — never persisted, so the shared settings file stays clean.
    """
    from gcs.backend import instance_ports
    n = instance_ports.chat_index()
    if n is None:
        return data
    if not (data.get("simulation") or {}).get("sim_mode"):
        return data
    data.setdefault("connection", {})["default_device"] = instance_ports.monitor_device(n)
    data.setdefault("simulation", {})["sitl_presets"] = [
        instance_ports.companion_device(s) for s in instance_ports.sysids_for_chat(n)
    ]
    return data


def _drop_injected_instance_device(body: dict) -> None:
    """Don't persist the per-chat values that ``_with_instance_device`` injects
    into GET responses. The frontend seeds its editable draft from the full GET
    body and PUTs the whole draft back on Save, so the injected
    ``connection.default_device`` and ``simulation.sitl_presets`` would otherwise
    be round-tripped into the shared (gitignored, cross-clone) settings file —
    making a later non-launcher run read stale per-chat ports. Drop each ONLY
    when it still equals the injected value, so genuine user edits are preserved.
    """
    from gcs.backend import instance_ports
    n = instance_ports.chat_index()
    if n is None:
        return
    conn = body.get("connection")
    if isinstance(conn, dict) and conn.get("default_device") == instance_ports.monitor_device(n):
        conn.pop("default_device", None)
    sim = body.get("simulation")
    injected_presets = [
        instance_ports.companion_device(s) for s in instance_ports.sysids_for_chat(n)
    ]
    if isinstance(sim, dict) and sim.get("sitl_presets") == injected_presets:
        sim.pop("sitl_presets", None)


@router.post("/api/settings/fallback-delivery-locations/import")
async def import_fallback_location_csv(file: UploadFile = File(...)) -> dict:
    """Import fallback delivery locations from a CSV file (columns: name, type, lat, lon)."""
    content = await file.read()
    text = content.decode("utf-8-sig")
    reader = csv.DictReader(io.StringIO(text))
    new_locations = []
    for row in reader:
        location_type = row.get("type", "other").strip().lower()
        if location_type not in DELIVERY_LOCATION_TYPES:
            location_type = "other"
        new_locations.append(FallbackDeliveryLocation(
            name=row["name"].strip(),
            type=location_type,
            lat=float(row["lat"]),
            lon=float(row["lon"]),
        ))
    current = settings_store.get()
    merged = list(current.fallback_delivery_locations) + new_locations
    updated = settings_store.update({"fallback_delivery_locations": [o.model_dump() for o in merged]})
    return updated.model_dump()


async def _stop_navpy_if_sim_mode_disabled(before, after) -> None:
    """Stop all NavPy companions and clear managed intent on a sim_mode True->False.

    Leaving sim mode must not orphan GCS-managed companions. With sim_mode off the
    ``/navpy-sim/*`` stop routes 400 and the watchdog (``ensure_auto_navpy_sim_running``)
    goes inert, so the operator can neither stop the companions nor rely on self-heal,
    and re-enabling sim_mode would silently resume managed restarts from the stale
    intent set. Reuse ``stop_all_instances`` (same effect as the operator Stop-All:
    terminate every companion and clear intent, keeping the manager) so a later
    re-enable starts from a clean slate — companions return only via fresh discovery
    auto-start or a manual Start.

    Then push one status broadcast: once sim_mode is off the telemetry loop stops
    broadcasting NavPy status, so without this an already-open GCS keeps showing the
    companions as "running" with a now-dead Stop button. ``broadcast_sim_status``
    reports the manager's true (now empty) state, flipping open clients to Start.

    Runs after settings are already persisted, so failures are logged, never raised,
    and cannot turn a good PUT into a 500.
    """
    before_on = bool(getattr(getattr(before, "simulation", None), "sim_mode", False))
    after_on = bool(getattr(getattr(after, "simulation", None), "sim_mode", False))
    if not (before_on and not after_on):
        return
    try:
        await asyncio.to_thread(runtime.stop_all_instances)
        log.info("Stopped all NavPy companions after sim_mode was disabled")
    except Exception as exc:
        log.warning("Failed to stop NavPy companions after sim_mode disabled: %s", exc)
    # Broadcast the true current status even if the stop above partially failed —
    # showing what actually remains beats leaving clients on a stale snapshot.
    try:
        from gcs.backend.routes.navpy_sim import broadcast_sim_status
        await broadcast_sim_status()
    except Exception as exc:
        log.warning("Failed to broadcast NavPy sim status after sim_mode disabled: %s", exc)


async def _restart_navpy_if_profile_changed(before, after) -> None:
    """Restart running NavPy sim processes when the selected profile changes."""
    if runtime.selected_vision_profile(before) == runtime.selected_vision_profile(after):
        return
    simulation = getattr(after, "simulation", None)
    if not bool(getattr(simulation, "sim_mode", False)):
        return
    try:
        restarted = await asyncio.to_thread(
            runtime.restart_running_navpy_sim,
            after,
        )
        if restarted:
            log.info("Restarted NavPy sim after vision profile change: %s", restarted)
    except Exception as exc:
        log.warning("Failed to restart NavPy sim after vision profile change: %s", exc)
