"""CONF-03 "Ask me anyway" one-shot gate-bypass override endpoint (D-18/D-19/D-20)."""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, HTTPException

from gcs.backend.models import TaskConfirmOverrideRequest
from gcs.backend.vehicle_manager import vehicle_mgr
from navpy.modules.comm.messages.swarm_request_msg import (
    REQUEST_TYPE_FORCE_CONFIRM,
    SwarmRequestMsg,
)

log = logging.getLogger(__name__)
router = APIRouter()

# SWARM_REQUEST is never acked -- this is a single unacknowledged packet, same
# transport-reliability shape as TaskConfirmResponseMsg in task_confirm.py.
# Blind retransmission compensates.
#
# The drone's one-shot force set is keyed by task_id, so a duplicate DELIVERY
# would be a harmless no-op ONLY if the resend loop's copies are actually
# collapsed to one delivery. That requires a valid (non-zero) message UID on
# the wire: MessageFilter._is_valid_meta treats boot_id=0/msg_seq=0 (an
# unset ``meta``) as legacy/no-metadata and lets EVERY copy through
# permissively, bypassing dedup entirely (message_filter.py's
# should_process/_is_valid_meta). Without a shared UID, each of the 3
# physical sends is delivered as a SEPARATE message to
# ConfirmOverrideListener, which unconditionally re-adds task_id to the
# force set on every delivery -- if the first copy is already consumed
# (bypasses the gate, discarded) before the 2nd/3rd copies arrive (routine:
# the nav loop ticks every ~40ms, the resend gap is 250ms), those later
# copies leave a STALE, un-consumed force flag sitting on that task_id. A
# later, unrelated confirm round for the SAME task_id (e.g. a D-14 bounded
# re-ask after a timeout) would then silently bypass the recognition gate
# with no fresh operator press-and-hold -- exactly the accidental-bypass
# threat T-02-05-02 exists to prevent.
#
# Fix: set the meta ONCE, before the loop, so all 3 physical sends carry the
# IDENTICAL (non-zero) boot_id/msg_seq -- the receiver's dedup cache then
# collapses copies 2/3 into copy 1 and ConfirmOverrideListener sees exactly
# one delivery per logical override, restoring the one-shot invariant this
# module's docstring already claims.
TASK_CONFIRM_OVERRIDE_SENDS = 3
TASK_CONFIRM_OVERRIDE_RESEND_GAP_S = 0.25


@router.post("/task_confirm_override")
async def force_confirm_override(req: TaskConfirmOverrideRequest):
    """Force one confirm request past the recognition gate for a single
    POI (D-18). Deliberate one-shot: the drone consumes the force flag
    the instant it bypasses the gate for this task_id; the gate stays on
    for every other/future POI.
    """
    entry = vehicle_mgr.get_vehicle(req.sys_id)
    if not entry:
        raise HTTPException(404, f"Vehicle {req.sys_id} not connected")

    # The companion shares its aircraft's sysid (distinguished by component
    # 191), so address it by plain sys_id — mirroring task_confirm.py.
    force_msg = SwarmRequestMsg(
        sender_id=0,
        receiver_id=req.sys_id,
        request_type=REQUEST_TYPE_FORCE_CONFIRM,
        subject_type=0,
        subject_id=req.task_id,
    )
    # One stable UID for the whole resend batch (see module docstring above)
    # -- must be set BEFORE the loop, not per-attempt, so every physical
    # send carries the SAME boot_id/msg_seq and the receiver's dedup cache
    # collapses the 3 copies into a single delivery.
    force_msg.set_meta_from_provider()
    for attempt in range(TASK_CONFIRM_OVERRIDE_SENDS):
        entry.vehicle.send_mavlink_message(force_msg.to_mavlink())
        if attempt < TASK_CONFIRM_OVERRIDE_SENDS - 1:
            await asyncio.sleep(TASK_CONFIRM_OVERRIDE_RESEND_GAP_S)

    log.info(
        "Force-confirm override sent x%d for vehicle %d task %d (unacknowledged transport)",
        TASK_CONFIRM_OVERRIDE_SENDS, req.sys_id, req.task_id,
    )

    return {
        "status": "sent",
        "delivery": "sent",
        "sends": TASK_CONFIRM_OVERRIDE_SENDS,
        "sys_id": req.sys_id,
        "task_id": req.task_id,
    }
