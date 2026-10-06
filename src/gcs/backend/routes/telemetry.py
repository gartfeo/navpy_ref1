"""WebSocket telemetry endpoint."""

from __future__ import annotations

import asyncio
import logging

import orjson
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.vehicle_manager import vehicle_mgr

log = logging.getLogger(__name__)
router = APIRouter()

# ---------------------------------------------------------------------------
# Manual-control repeater — replays last stick values at 20 Hz via MAVLink
# ---------------------------------------------------------------------------
_mc_state: dict[int, tuple[int, int, int, int]] = {}
_mc_task: asyncio.Task | None = None


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def _parse_mc_values(msg: dict) -> tuple[int, int, int, int]:
    x = _clamp(int(msg.get("x", 0)), -1000, 1000)
    y = _clamp(int(msg.get("y", 0)), -1000, 1000)
    z = _clamp(int(msg.get("z", 500)), 0, 1000)
    r = _clamp(int(msg.get("r", 0)), -1000, 1000)
    return x, y, z, r


async def _mc_send_loop() -> None:
    """Send stored MANUAL_CONTROL values at 20 Hz until state is empty."""
    global _mc_task
    try:
        while _mc_state:
            for sys_id, (x, y, z, r) in list(_mc_state.items()):
                entry = vehicle_mgr.get_vehicle(sys_id)
                if entry:
                    try:
                        entry.vehicle.send_manual_control(x, y, z, r)
                    except Exception:
                        log.exception("Failed to send manual_control to %d", sys_id)
            await asyncio.sleep(0.05)
    finally:
        _mc_task = None


def _ensure_mc_loop() -> None:
    global _mc_task
    if _mc_task is None or _mc_task.done():
        _mc_task = asyncio.get_running_loop().create_task(_mc_send_loop())


def _handle_manual_control(msg: dict) -> None:
    sys_id = msg.get("sys_id", -1)
    entry = vehicle_mgr.get_vehicle(sys_id)
    if not entry:
        log.warning("manual_control: vehicle %d not found", sys_id)
        return
    vals = _parse_mc_values(msg)
    prev = _mc_state.get(sys_id)
    _mc_state[sys_id] = vals
    # Send immediately so the first message has no latency
    entry.vehicle.send_manual_control(*vals)
    if prev is None:
        log.info("manual_control started for vehicle %d (src_sys=%d)",
                 sys_id, entry.vehicle.transport_source_system)
    _ensure_mc_loop()


def _handle_manual_control_stop(msg: dict) -> None:
    sys_id = msg.get("sys_id", -1)
    _mc_state.pop(sys_id, None)
    entry = vehicle_mgr.get_vehicle(sys_id)
    if entry:
        entry.vehicle.send_manual_control(0, 0, 500, 0)


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------
@router.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket):
    await ws_manager.connect(ws)
    try:
        # Send initial state on connect
        snapshots = vehicle_mgr.get_all_snapshots()
        if snapshots:
            await ws.send_bytes(orjson.dumps({
                "type": "telemetry",
                "vehicles": snapshots,
                "ts": __import__("time").time(),
            }))

        # Keep connection alive, process incoming messages
        while True:
            data = await ws.receive_text()
            if data == "ping":
                await ws.send_text("pong")
                continue
            try:
                msg = orjson.loads(data)
            except Exception:
                log.debug("Ignoring unparseable WS message")
                continue
            msg_type = msg.get("type")
            try:
                if msg_type == "manual_control":
                    _handle_manual_control(msg)
                elif msg_type == "manual_control_stop":
                    _handle_manual_control_stop(msg)
            except Exception:
                log.exception("Error handling WS message type=%s", msg_type)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        log.error("WS error: %s", e)
    finally:
        # Stop any active manual control when the client disconnects
        _mc_state.clear()
        await ws_manager.disconnect(ws)
