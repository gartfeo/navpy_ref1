"""Frontend diagnostic ingestion and operator session export."""
from __future__ import annotations

import tempfile
import re
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from starlette.background import BackgroundTask
from pydantic import BaseModel, Field

from gcs.backend.diagnostics import ALLOWED_FRONTEND_EVENTS, diagnostic_session

router = APIRouter(prefix="/api/diagnostics")


class FrontendDiagnosticEvent(BaseModel):
    event: str
    fields: dict[str, Any] = Field(default_factory=dict)


_EVENT_FIELDS = {
    "client_request_started": {"client_id", "client_seq", "request_id", "sys_id", "phase"},
    "client_request_finished": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome", "status_code", "total", "error_code"},
    "client_timeout": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome"},
    "mission_result_accepted": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "total"},
    "mission_result_rejected": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "total"},
    "plan_rendered": {"client_id", "client_seq", "zone_count", "zone_sys_ids", "waypoint_counts"},
    "parameter_snapshot_accepted": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "status_code", "record_count", "stale"},
    "parameter_snapshot_rejected": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "status_code", "record_count", "stale"},
    "parameter_write_finished": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "status_code", "accepted_count", "rejected_count", "stale"},
    "vehicle_operation_finished": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome"},
}
_EVENT_REQUIRED = {
    "client_request_started": {"client_id", "client_seq", "request_id", "sys_id", "phase"},
    "client_request_finished": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome"},
    "client_timeout": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome"},
    "mission_result_accepted": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "total"},
    "mission_result_rejected": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "total"},
    "plan_rendered": {"client_id", "client_seq", "zone_count", "zone_sys_ids", "waypoint_counts"},
    "parameter_snapshot_accepted": {"client_id", "client_seq", "request_id", "sys_id", "outcome"},
    "parameter_snapshot_rejected": {"client_id", "client_seq", "request_id", "sys_id", "outcome"},
    "parameter_write_finished": {"client_id", "client_seq", "request_id", "sys_id", "outcome", "rejected_count"},
    "vehicle_operation_finished": {"client_id", "client_seq", "request_id", "sys_id", "phase", "outcome"},
}
_TOKEN = re.compile(r"^[A-Za-z0-9_-]{1,96}$")
_TOKEN_FIELDS = {"client_id", "request_id", "phase", "outcome", "error_code"}
_COUNT_FIELDS = {"client_seq", "total", "record_count", "accepted_count", "rejected_count", "zone_count"}


def _validated_fields(event: str, fields: dict[str, Any]) -> dict[str, Any]:
    allowed = _EVENT_FIELDS[event]
    if set(fields) - allowed:
        raise HTTPException(422, detail="Unsupported diagnostic fields")
    if not _EVENT_REQUIRED[event].issubset(fields):
        raise HTTPException(422, detail="Missing diagnostic fields")
    result: dict[str, Any] = {}
    for key, value in fields.items():
        if key in _TOKEN_FIELDS:
            if not isinstance(value, str) or not _TOKEN.fullmatch(value):
                raise HTTPException(422, detail=f"Invalid diagnostic field: {key}")
        elif key == "sys_id":
            if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 255:
                raise HTTPException(422, detail="Invalid diagnostic field: sys_id")
        elif key == "status_code":
            if not isinstance(value, int) or isinstance(value, bool) or not 100 <= value <= 599:
                raise HTTPException(422, detail="Invalid diagnostic field: status_code")
        elif key in _COUNT_FIELDS:
            if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 1_000_000:
                raise HTTPException(422, detail=f"Invalid diagnostic field: {key}")
        elif key in {"zone_sys_ids", "waypoint_counts"}:
            if not isinstance(value, list) or len(value) > 20 or any(
                not isinstance(item, int) or isinstance(item, bool) or item < 0 or item > 1_000_000
                for item in value
            ):
                raise HTTPException(422, detail=f"Invalid diagnostic field: {key}")
        elif key == "stale" and not isinstance(value, bool):
            raise HTTPException(422, detail="Invalid diagnostic field: stale")
        result[key] = value
    return result


@router.post("/events", status_code=204)
async def record_frontend_event(body: FrontendDiagnosticEvent):
    if body.event not in ALLOWED_FRONTEND_EVENTS:
        raise HTTPException(422, detail="Unsupported diagnostic event")
    diagnostic_session.emit(body.event, source="frontend", **_validated_fields(body.event, body.fields))


@router.get("/session")
async def session_info():
    return {"session_id": diagnostic_session.session_id}


@router.get("/export")
async def export_session():
    if not diagnostic_session.active:
        raise HTTPException(503, detail="Diagnostic session is not active")
    handle = tempfile.NamedTemporaryFile(
        prefix=f"gcs-diagnostics-{diagnostic_session.session_id[:8]}-",
        suffix=".zip", delete=False,
    )
    handle.close()
    tmp = Path(handle.name)
    diagnostic_session.emit("diagnostics_exported", source="backend")
    diagnostic_session.export_zip(tmp)
    return FileResponse(
        tmp,
        media_type="application/zip",
        filename=f"gcs-diagnostics-{diagnostic_session.session_id[:8]}.zip",
        background=BackgroundTask(tmp.unlink, missing_ok=True),
    )
