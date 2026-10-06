"""Source-filtered companion camera telemetry and per-device snapshots."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

if TYPE_CHECKING:
    from gcs.backend.vehicle_entry import VehicleEntry

log = logging.getLogger("gcs.backend.vehicle_manager")

import math
from typing import Any
from navpy.modules.vision.vision_profiles import MAX_GIMBAL_DEVICE_ID, MIN_GIMBAL_DEVICE_ID
from navpy.modules.vision.mavlink_camera_components import GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT
from gcs.backend.vehicle_status import (
    GIMBAL_TELEMETRY_STALE_S, CAMERA_OPTICS_STALE_S, _LOG_GIMBAL_TELEM,
)

def _on_gimbal_device_attitude_status(entry: VehicleEntry, msg: MAVLink_message) -> None:
    if not entry._from_own_companion(msg):
        return
    raw_device_id = getattr(msg, "gimbal_device_id", 0)
    if isinstance(raw_device_id, bool) or not isinstance(raw_device_id, int):
        return
    if raw_device_id < MIN_GIMBAL_DEVICE_ID or raw_device_id > MAX_GIMBAL_DEVICE_ID:
        return

    q = _coerce_gimbal_quaternion(getattr(msg, "q", None))
    if q is None:
        return

    now_wall = time.time()
    now_monotonic = time.monotonic()
    with entry._gimbal_lock:
        record = dict(entry._gimbals.get(raw_device_id, {}))
        record.update({
            "device_id": raw_device_id,
            "q": q,
            "flags": int(getattr(msg, "flags", 0)),
            "failure_flags": int(getattr(msg, "failure_flags", 0)),
            "time_boot_ms": int(getattr(msg, "time_boot_ms", 0)),
            "updated_at": now_wall,
            "_updated_monotonic": now_monotonic,
        })
        entry._gimbals[raw_device_id] = record

    if _LOG_GIMBAL_TELEM:
        # sys_id + device_id + MAVLink SOURCE (system/component) + companion
        # clock (time_boot_ms) + wall + raw quaternion (wxyz). The source
        # names WHO emitted the sample: two distinct src components under one
        # gimbal device => a double-publish (e.g. the NavPy camera publisher
        # vs a SITL/ArduPilot mount) that would flicker the map footprint.
        # Offline: diff q per (sys, device, src) for the footprint-driver
        # angular rate and jitter. Logged outside the lock.
        src_sys = msg.get_srcSystem() if hasattr(msg, "get_srcSystem") else -1
        src_comp = msg.get_srcComponent() if hasattr(msg, "get_srcComponent") else -1
        log.info(
            "GIMBAL_TELEM sys=%d dev=%d srcsys=%d srccomp=%d tb=%d wall=%.4f "
            "q=[%.6f,%.6f,%.6f,%.6f]",
            entry.sys_id, raw_device_id, src_sys, src_comp,
            int(getattr(msg, "time_boot_ms", 0)), now_wall,
            q[0], q[1], q[2], q[3],
        )

def _on_camera_fov_status(entry: VehicleEntry, msg: MAVLink_message) -> None:
    if not entry._from_own_companion(msg):
        return
    device_id = _gimbal_device_id_from_camera_component(msg)
    if device_id is None:
        return

    fov_h_rad = _coerce_fov_degrees_to_rad(getattr(msg, "hfov", None))
    fov_v_rad = _coerce_fov_degrees_to_rad(getattr(msg, "vfov", None))
    if fov_h_rad is None or fov_v_rad is None:
        return

    now_wall = time.time()
    now_monotonic = time.monotonic()
    with entry._gimbal_lock:
        record = dict(entry._gimbals.get(device_id, {}))
        record.update({
            "device_id": device_id,
            "fov_h_rad": fov_h_rad,
            "fov_v_rad": fov_v_rad,
            "optics_time_boot_ms": int(getattr(msg, "time_boot_ms", 0)),
            "optics_updated_at": now_wall,
            "_optics_updated_monotonic": now_monotonic,
        })
        entry._gimbals[device_id] = record

def _on_camera_settings(entry: VehicleEntry, msg: MAVLink_message) -> None:
    if not entry._from_own_companion(msg):
        return
    device_id = _gimbal_device_id_from_camera_component(msg)
    if device_id is None:
        return

    zoom_level = _coerce_positive_float(getattr(msg, "zoomLevel", None))
    if zoom_level is None:
        return

    with entry._gimbal_lock:
        record = dict(entry._gimbals.get(device_id, {}))
        record.update({
            "device_id": device_id,
            "zoom_level": zoom_level,
            "settings_time_boot_ms": int(getattr(msg, "time_boot_ms", 0)),
        })
        entry._gimbals[device_id] = record

def _gimbal_snapshot(entry: VehicleEntry) -> dict:
    gimbals = getattr(entry, "_gimbals", None)
    if not gimbals:
        return {}

    lock = getattr(entry, "_gimbal_lock", None)
    if lock is None:
        records = list(gimbals.items())
    else:
        with lock:
            records = list(gimbals.items())

    now_monotonic = time.monotonic()
    snapshot = {}
    for device_id, record in records:
        updated_monotonic = record.get("_updated_monotonic")
        stale = (
            updated_monotonic is None
            or now_monotonic - updated_monotonic > GIMBAL_TELEMETRY_STALE_S
        )
        item = {
            "device_id": record["device_id"],
            "stale": stale,
        }
        if "q" in record:
            item.update({
                "q": list(record["q"]),
                "flags": record["flags"],
                "failure_flags": record["failure_flags"],
                "time_boot_ms": record["time_boot_ms"],
                "updated_at": record["updated_at"],
            })

        optics_updated_monotonic = record.get("_optics_updated_monotonic")
        if "fov_h_rad" in record and "fov_v_rad" in record:
            optics_stale = (
                optics_updated_monotonic is None
                or now_monotonic - optics_updated_monotonic > CAMERA_OPTICS_STALE_S
            )
            item.update({
                "fov_h_rad": record["fov_h_rad"],
                "fov_v_rad": record["fov_v_rad"],
                "optics_time_boot_ms": record["optics_time_boot_ms"],
                "optics_updated_at": record["optics_updated_at"],
                "optics_stale": optics_stale,
            })
        if "zoom_level" in record:
            item["zoom_level"] = record["zoom_level"]
            item["settings_time_boot_ms"] = record["settings_time_boot_ms"]

        snapshot[str(device_id)] = item
    return snapshot

def _coerce_gimbal_quaternion(q: Any) -> list[float] | None:
    if q is None:
        return None
    try:
        values = [float(value) for value in q]
    except (TypeError, ValueError):
        return None
    if len(values) != 4:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values

def _gimbal_device_id_from_camera_component(msg: MAVLink_message) -> int | None:
    get_src_component = getattr(msg, "get_srcComponent", None)
    if not callable(get_src_component):
        return None
    try:
        component = int(get_src_component())
    except (TypeError, ValueError):
        return None
    return GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT.get(component)

def _coerce_fov_degrees_to_rad(value: Any) -> float | None:
    try:
        degrees = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(degrees) or degrees <= 0.0 or degrees >= 180.0:
        return None
    return math.radians(degrees)

def _coerce_positive_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result) or result <= 0.0:
        return None
    return result
