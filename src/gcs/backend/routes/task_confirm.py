"""Task confirmation response endpoint."""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, HTTPException

from gcs.backend.models import TaskConfirmResponseRequest
from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.broadcast import manager as ws_manager
from gcs.backend.task_confirm_transport import correlated_meta
from navpy.modules.comm.messages.available_task_msg import TaskConfirmResponseMsg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType

log = logging.getLogger(__name__)
router = APIRouter()

# The confirm response is a single unacknowledged MAVLink packet and the
# companion resolves its 30 s window by timeout, so one lost packet leaves
# the UI showing a decision the UAV never received. There is no response-ack
# in the protocol; blind retransmission is the compensator. Duplicates are
# safe: the companion pops the pending-confirmation token on the first copy
# and logs later copies as duplicates.
TASK_CONFIRM_RESPONSE_SENDS = 3
TASK_CONFIRM_RESPONSE_RESEND_GAP_S = 0.25


# Operator action -> the is_confirmed value it must carry. The vehicle only
# understands confirm/reject; the action enriches UI/audit. Any mismatch is a
# malformed request (e.g. action="cancel" with is_confirmed=true), rejected
# before any MAVLink is sent or any WS event is broadcast. ("abort" is NOT a
# confirm action -- a per-UAV abort is the destructive E-STOP command.)
_ACTION_CONFIRMED = {
    "approve": True,
    "timeout_approve": True,
    "deny": False,
    "cancel": False,
    "timeout_deny": False,
}


def _response_meta(round_uid: str | None) -> MsgMeta | None:
    if round_uid is None:
        return None
    try:
        return correlated_meta(
            round_uid,
            MsgType.TASK_CONFIRM_RESPONSE,
        )
    except ValueError as exc:
        raise HTTPException(
            400,
            f"Invalid confirmation round uid: {round_uid}",
        ) from exc


@router.post("/task_confirm")
async def respond_to_task_confirm(req: TaskConfirmResponseRequest):
    """Send approve/deny/cancel response for a task confirmation request.

    ``action`` records operator intent for the UI/audit; ``is_confirmed`` is
    what the vehicle acts on. ``cancel`` recalls an already-approved target
    (recoverable); the vehicle treats any negative response as a target reject.
    """
    if req.action is not None:
        if req.action not in _ACTION_CONFIRMED:
            raise HTTPException(400, f"Unknown action: {req.action}")
        if _ACTION_CONFIRMED[req.action] != req.is_confirmed:
            raise HTTPException(
                400,
                f"Action '{req.action}' is inconsistent with is_confirmed={req.is_confirmed}",
            )

    entry = vehicle_mgr.get_vehicle(req.sys_id)
    if not entry:
        raise HTTPException(404, f"Vehicle {req.sys_id} not connected")

    # The action string is operator-intent audit only; is_confirmed is what
    # the vehicle acts on. Falls back to the plain approve/deny label when no
    # action was supplied (dev's pre-existing request shape).
    action = req.action or ("approve" if req.is_confirmed else "deny")

    # The companion shares its aircraft's sysid (it is distinguished by
    # component 191), so the response is addressed to the plain sys_id.
    # ArduPilot forwards a packet targeted at its own system id onward to the
    # companion on serial0 rather than swallowing it (SITL-proven 2026-07-28).
    response_msg = TaskConfirmResponseMsg(
        receiver_id=req.sys_id,
        task_id=req.task_id,
        is_confirmed=req.is_confirmed,
        meta=_response_meta(req.round_uid),
    )
    for attempt in range(TASK_CONFIRM_RESPONSE_SENDS):
        entry.vehicle.send_mavlink_message(response_msg.to_mavlink())
        if attempt < TASK_CONFIRM_RESPONSE_SENDS - 1:
            await asyncio.sleep(TASK_CONFIRM_RESPONSE_RESEND_GAP_S)

    status = "approved" if req.is_confirmed else "denied"
    log.info(
        "Task %d %s (%s) response sent x%d for vehicle %d (unacknowledged transport)",
        req.task_id, status, action, TASK_CONFIRM_RESPONSE_SENDS, req.sys_id,
    )

    # D-10/D-12: this round is decided -- end the listener's popup-dedup and
    # thumbnail-fetch bookkeeping AND record the decision. All three copies
    # above can be lost, and the companion then keeps resending its request;
    # the listener answers those repeats with this stored decision instead of
    # re-opening a popup for a target the operator already settled.
    #
    # req.round_uid identifies the round the operator was shown. It matters
    # that the decision is bound to THAT round and not to whatever is open
    # now: the x3 burst above takes ~0.5s, long enough for the companion to
    # open a new round in the meantime, and that new round must get its own
    # popup rather than inherit this answer.
    listener = vehicle_mgr.get_task_confirm_listener()
    if listener is not None:
        listener.mark_decided(req.sys_id, req.task_id, req.is_confirmed, req.round_uid)

    await ws_manager.broadcast({
        "type": "task_confirm_response",
        "sys_id": req.sys_id,
        "task_id": req.task_id,
        "is_confirmed": req.is_confirmed,
        "action": action,
        # The round this answers. Every client needs it: the same target can
        # be asked again while this response is still going out, and that new
        # round's card must not be marked decided by this one.
        "round_uid": req.round_uid,
        # Transport truth: the packet was sent, not acknowledged. The
        # authoritative outcome is the vehicle's subsequent behavior.
        "delivery": "sent",
    })

    return {
        "status": status,
        "action": action,
        "delivery": "sent",
        "sends": TASK_CONFIRM_RESPONSE_SENDS,
        "sys_id": req.sys_id,
        "task_id": req.task_id,
    }
