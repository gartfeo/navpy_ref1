"""API route for downloading simulation logs."""
from __future__ import annotations

import io
import zipfile
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

router = APIRouter()

LOGS_ROOT = Path(".logs")


def _find_latest_session(sys_id: int) -> list[Path]:
    """Return log files from the latest session for the given sys_id.

    Scans .logs/{date}/{time}/ directories sorted newest-first and returns
    navigation logs plus confirmation image artifacts from the first session
    containing ``uav_{sys_id}_navigation*`` files.
    """
    if not LOGS_ROOT.is_dir():
        return []

    # Collect all (date_dir, time_dir) pairs, sorted newest first
    session_dirs: list[Path] = []
    for date_dir in sorted(LOGS_ROOT.iterdir(), reverse=True):
        if not date_dir.is_dir():
            continue
        for time_dir in sorted(date_dir.iterdir(), reverse=True):
            if not time_dir.is_dir():
                continue
            session_dirs.append(time_dir)

    navigation_prefix = f"uav_{sys_id}_navigation"
    confirmation_prefix = f"uav_{sys_id}_confirmation_"
    for session in session_dirs:
        navigation_files = [
            f for f in session.iterdir()
            if f.is_file() and f.name.startswith(navigation_prefix)
        ]
        if navigation_files:
            confirmation_files = [
                f for f in session.iterdir()
                if f.is_file() and f.name.startswith(confirmation_prefix)
            ]
            return sorted(navigation_files + confirmation_files)
    return []


@router.get("/api/logs/download/{sys_id}")
def download_logs(sys_id: int):
    """Download the latest session's log files for a UAV as a zip archive."""
    files = _find_latest_session(sys_id)
    if not files:
        raise HTTPException(404, detail=f"No logs found for UAV {sys_id}")

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in files:
            zf.write(f, f.name)
    buf.seek(0)

    session_dir = files[0].parent
    zip_name = f"uav_{sys_id}_{session_dir.parent.name}_{session_dir.name}.zip"

    return StreamingResponse(
        buf,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{zip_name}"'},
    )
