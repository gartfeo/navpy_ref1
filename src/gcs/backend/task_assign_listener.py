"""Listener for the task-assignment handshake.

Intercepts NAVLINK messages (adverts, TaskAssignRequestMsg,
TaskAssignResponseMsg and the owners' SWARM_ACKs) from VehicleMav and
broadcasts them to WebSocket clients so the GCS can track peer-assigned POIs
on the map and sidebar. Requests and responses repeat with a fresh UID per
copy (docs/design/swarm-task-assignment-ack.md): every copy is forwarded
with its ``uid``, and an owner's APPLIED of a helper's "doing" is reported
as ``task_assign_ack``.
"""
from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable
from typing import Any, Optional

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.msg_abc import MsgABC, MsgRegistry
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from gcs.backend.companion_identity import is_from_companion
from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.task_assign_rounds import AssignedTask, AssignRounds, Uid

log = logging.getLogger(__name__)


class TaskAssignListener:
    """Listens for the task-assignment handshake on VehicleMav connections.

    Callbacks fire in MavBus reader threads, so we use
    asyncio.run_coroutine_threadsafe() to schedule WS broadcasts.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._registered_ids: set[int] = set()
        self._rounds = AssignRounds()
        self._handlers: dict[type[MsgABC], Callable[[Any], None]] = {
            AvailableTaskRequestMsg: self._handle_available_task_request,
            TaskAssignRequestMsg: self._handle_assign_request,
            TaskAssignResponseMsg: self._handle_assign_response,
            SwarmAckMsg: self._handle_ack,
        }

    def register_vehicle(self, sys_id: int, vehicle: VehicleMav) -> None:
        """Register NAVLINK callback on a VehicleMav instance."""
        self._registered_ids.add(sys_id)
        vehicle.on_message("NAVLINK", lambda msg, sid=sys_id: self._on_navlink(sid, msg))
        log.info("TaskAssignListener registered for vehicle %d", sys_id)

    def _on_navlink(self, sys_id: int, msg: MAVLink_message) -> None:
        """Dispatch a handshake message to its handler.

        On the shared MAVLink bus a single packet is delivered once per
        vehicle link, so only the listener that owns the sending companion may
        act on it; otherwise every assignment is rebroadcast once per
        connected UAV (three times for a 3-UAV demo).
        """
        msg_cls = MsgRegistry.get_class_by_mav_id(msg.get_msgId())
        handler = self._handlers.get(msg_cls)
        if handler is not None and self._is_from_registered_companion(sys_id, msg):
            handler(msg_cls.from_mavlink(msg))

    def _handle_available_task_request(self, req: AvailableTaskRequestMsg) -> None:
        """Broadcast an advert to WS clients.

        Skips broadcast when the sender is one of our own vehicles (self-originated)
        or when there are no peers (only one vehicle connected).
        """
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
            "uid": _uid_payload(_uid(req)),
            "tasks": tasks,
        }
        log.info(
            "Available task request: sender=%d tasks=%d",
            req.sender_id, len(tasks),
        )
        self._broadcast(payload)

    def _handle_assign_request(self, req: TaskAssignRequestMsg) -> None:
        """Broadcast one step-3 copy to WS clients."""
        task = req.task
        uid = _uid(req)
        payload = {
            "type": "task_assign_request",
            "sender_id": req.sender_id,
            "receiver_id": req.receiver_id,
            "task_id": task.task_id,
            "task_type": task.task_type.name,
            "lat": task.location.lat,
            "lon": task.location.lng,
            "alt": task.location.alt,
            "uid": _uid_payload(uid),
        }
        log.info(
            "Task assign request: sender=%d receiver=%d task_id=%d at (%.6f, %.6f)"
            " uid=%d:%d",
            req.sender_id, req.receiver_id, task.task_id,
            task.location.lat, task.location.lng, *uid,
        )
        self._broadcast(payload)

    def _handle_assign_response(self, resp: TaskAssignResponseMsg) -> None:
        """Broadcast one step-4 copy; an accepted copy joins its helper's round."""
        uid = _uid(resp)
        payload = {
            "type": "task_assign_response",
            "sender_id": resp.sender_id,
            "receiver_id": resp.receiver_id,
            "task_id": resp.task_id,
            "is_accepted": resp.is_accepted,
            "uid": _uid_payload(uid),
        }
        log.info(
            "Task assign response: sender=%d receiver=%d task_id=%d accepted=%s"
            " uid=%d:%d",
            resp.sender_id, resp.receiver_id, resp.task_id, resp.is_accepted, *uid,
        )
        self._broadcast(payload)
        if resp.is_accepted:
            self._report(self._rounds.on_accepted_copy(
                resp.sender_id, resp.receiver_id, resp.task_id, uid,
            ))

    def _handle_ack(self, ack: SwarmAckMsg) -> None:
        """Report an owner's APPLIED of a step-4 copy; other acks are not shown."""
        if (
            ack.status != ACK_STATUS_APPLIED
            or ack.ref_msg_type != MsgType.TASK_ASSIGN_RESPONSE.value
        ):
            return
        self._report(self._rounds.on_applied(
            ack.sender_id,
            ack.receiver_id,
            (ack.ref_boot_id, ack.ref_msg_seq),
            _uid(ack),
        ))

    def _report(self, assigned: Optional[AssignedTask]) -> None:
        if assigned is None:
            return
        log.info(
            "Task assign ack: owner=%d helper=%d task_id=%d status=APPLIED"
            " ref=%d:%d uid=%d:%d",
            assigned.owner_id, assigned.helper_id, assigned.task_id,
            *assigned.ref, *assigned.uid,
        )
        self._broadcast({
            "type": "task_assign_ack",
            "owner_id": assigned.owner_id,
            "helper_id": assigned.helper_id,
            "task_id": assigned.task_id,
            "status": "APPLIED",
            "ref": _uid_payload(assigned.ref),
            "uid": _uid_payload(assigned.uid),
        })

    @staticmethod
    def _is_from_registered_companion(sys_id: int, msg: MAVLink_message) -> bool:
        """Match a shared-bus packet to the listener that owns its sender.

        The companion shares its aircraft's system id, so the component id
        (191) is what separates it from the autopilot's own traffic.
        """
        return is_from_companion(msg, sys_id)

    def _broadcast(self, payload: dict) -> None:
        """Thread-safe broadcast via asyncio event loop."""
        asyncio.run_coroutine_threadsafe(ws_manager.broadcast(payload), self._loop)


def _uid(message: MsgABC) -> Uid:
    """The message's ``(boot_id, msg_seq)``; legacy meta reads as 0:0."""
    meta = message.meta
    if meta is None:
        return 0, 0
    return meta.boot_id, meta.msg_seq


def _uid_payload(uid: Uid) -> dict[str, int]:
    boot_id, msg_seq = uid
    return {"boot_id": boot_id, "msg_seq": msg_seq}
