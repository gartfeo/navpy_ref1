"""Prepared launch trigger and abort endpoints."""
from __future__ import annotations

import logging
from fastapi import HTTPException
from gcs.backend.models import TriggerRequest
from gcs.backend.launch_controller import LaunchController

log = logging.getLogger("gcs.backend.routes.control")

async def trigger_vehicle(req: TriggerRequest, *, launch_controller: LaunchController) -> dict:
    """Trigger a single vehicle in the prepared container session."""
    log.info("[launch] /launch/trigger called for sys_id=%d", req.sys_id)
    if not launch_controller.is_prepared:
        log.error("[launch] /launch/trigger rejected — controller not prepared")
        raise HTTPException(400, "Launch controller not prepared")
    try:
        await launch_controller.trigger_vehicle(req.sys_id)
    except RuntimeError as e:
        log.error("[launch] /launch/trigger V%d RuntimeError: %s", req.sys_id, e)
        raise HTTPException(409, str(e))
    except ValueError as e:
        log.error("[launch] /launch/trigger V%d ValueError: %s", req.sys_id, e)
        raise HTTPException(400, str(e))
    log.info("[launch] /launch/trigger V%d accepted", req.sys_id)
    return {"status": "triggered", "sys_id": req.sys_id}

async def abort_launch(*, launch_controller: LaunchController) -> dict:
    """Abort an in-progress container launch sequence."""
    if not launch_controller.is_running and not launch_controller.is_prepared:
        return {"status": "no_launch"}
    await launch_controller.abort()
    return {"status": "aborted"}
