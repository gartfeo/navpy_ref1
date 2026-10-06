"""Listener for task assignment requests and responses.

Intercepts NAVLINK messages (TaskAssignRequestMsg, TaskAssignResponseMsg)
from VehicleMav and broadcasts them to WebSocket clients so the GCS can
track peer-assigned POIs on the map and sidebar.
"""
from __future__ import annotations

import asyncio
import logging

from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.msg_abc import MsgRegistry
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.companion_identity import is_from_companion
from gcs.backend.broadcast import manager as ws_manager

log = logging.getLogger(__name__)


class TaskAssignListener:
    """Listens for task assignment requests/responses on VehicleMav connections.

    Callbacks fire in MavBus reader threads, so we use
    asyncio.run_coroutine_threadsafe() to schedule WS broadcasts.
    """


    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop
        self._registered_ids: set[int] = set()

    def register_vehicle(self, sys_id: int, vehicle: VehicleMav) -> None:
        """Register NAVLINK callback on a VehicleMav instance."""
        self._registered_ids.add(sys_id)
        vehicle.on_message("NAVLINK", lambda msg, sid=sys_id: self._on_navlink(sid, msg))
        log.info("TaskAssignListener registered for vehicle %d", sys_id)

    def _on_navlink(self, sys_id: int, msg) -> None:
        """Handle NAVLINK messages — dispatch assign requests and responses.

        On the shared MAVLink bus a single packet is delivered once per
        vehicle link, so only the listener that owns the sending companion may
        act on it; otherwise every assignment is rebroadcast once per
        connected UAV (three times for a 3-UAV demo).
        """
        msg_cls = MsgRegistry.get_class_by_mav_id(msg.get_msgId())
        if msg_cls is None:
            return

        if issubclass(msg_cls, AvailableTaskRequestMsg):
            handler = self._handle_available_task_request
        elif issubclass(msg_cls, TaskAssignRequestMsg):
            handler = self._handle_assign_request
        elif issubclass(msg_cls, TaskAssignResponseMsg):
            handler = self._handle_assign_response
        else:
            return

        if self._is_from_registered_companion(sys_id, msg):
            handler(sys_id, msg_cls, msg)

    def _handle_available_task_request(self, sys_id: int, msg_cls, msg) -> None:
        """Parse AvailableTaskRequestMsg and broadcast to WS clients.

        Skips broadcast when the sender is one of our own vehicles (self-originated)
        or when there are no peers (only one vehicle connected).
        """
        req: AvailableTaskRequestMsg = msg_cls.from_mavlink(msg)
        if len(self._registered_ids) < 2:
            return
        tasks = [
            {
                "task_id": t.task_id,
                "task_type": t.task_type.name,
                "lat": t.location.lat,
                "lon": t.location.lng,
                "alt": t.location.alt,
            }
            for t in req.tasks
        ]
        payload = {
            "type": "available_task_request",
            "sender_id": req.sender_id,
            "tasks": tasks,
        }
        log.info(
            "Available task request: sender=%d tasks=%d",
            req.sender_id, len(tasks),
        )
        self._broadcast(payload)

    def _handle_assign_request(self, sys_id: int, msg_cls, msg) -> None:
        """Parse TaskAssignRequestMsg and broadcast to WS clients."""
        req: TaskAssignRequestMsg = msg_cls.from_mavlink(msg)
        task = req.task
        payload = {
            "type": "task_assign_request",
            "sender_id": req.sender_id,
            "receiver_id": req.receiver_id,
            "task_id": task.task_id,
            "task_type": task.task_type.name,
            "lat": task.location.lat,
            "lon": task.location.lng,
            "alt": task.location.alt,
        }
        log.info(
            "Task assign request: sender=%d receiver=%d task_id=%d at (%.6f, %.6f)",
            req.sender_id, req.receiver_id, task.task_id,
            task.location.lat, task.location.lng,
        )
        self._broadcast(payload)

    def _handle_assign_response(self, sys_id: int, msg_cls, msg) -> None:
        """Parse TaskAssignResponseMsg and broadcast to WS clients."""
        resp: TaskAssignResponseMsg = msg_cls.from_mavlink(msg)
        payload = {
            "type": "task_assign_response",
            "sender_id": resp.sender_id,
            "receiver_id": resp.receiver_id,
            "task_id": resp.task_id,
            "is_accepted": resp.is_accepted,
        }
        log.info(
            "Task assign response: sender=%d receiver=%d task_id=%d accepted=%s",
            resp.sender_id, resp.receiver_id, resp.task_id, resp.is_accepted,
        )
        self._broadcast(payload)

    @staticmethod
    def _is_from_registered_companion(sys_id: int, msg) -> bool:
        """Match a shared-bus packet to the listener that owns its sender.

        The companion shares its aircraft's system id, so the component id
        (191) is what separates it from the autopilot's own traffic.
        """
        return is_from_companion(msg, sys_id)

    def _broadcast(self, payload: dict) -> None:
        """Thread-safe broadcast via asyncio event loop."""
        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(payload), self._loop)
