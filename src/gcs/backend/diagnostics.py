"""Bounded, privacy-conscious GCS operator session diagnostics.

The timeline intentionally stores only allow-listed identifiers, states, counts,
and durations.  It must never receive mission geometry, parameter names/values,
connection strings, or raw telemetry.
"""
from __future__ import annotations

import json
import threading
import uuid
import zipfile
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from gcs.backend.runtime_paths import gcs_log_dir


MAX_FILE_BYTES = 5_000_000
MAX_SESSION_FILES = 2
MAX_RETAINED_SESSIONS = 10
MAX_SESSION_AGE_DAYS = 14

ALLOWED_FIELDS = {
    "client_id", "client_seq", "request_id", "operation_id", "sys_id", "sys_ids",
    "method", "route", "status_code", "duration_ms", "outcome", "reason",
    "phase", "source", "cached", "current", "total", "bytes_read",
    "size_estimate", "record_count", "accepted_count", "rejected_count",
    "zone_count", "zone_sys_ids", "waypoint_counts", "vehicle_count",
    "connected_count", "missing_sys_ids", "running", "exit_code", "attempt",
    "stale", "error_code",
}

ALLOWED_FRONTEND_EVENTS = {
    "client_request_started", "client_request_finished", "client_timeout",
    "mission_result_accepted", "mission_result_rejected", "plan_rendered",
    "parameter_snapshot_accepted", "parameter_snapshot_rejected",
    "parameter_write_finished", "vehicle_operation_finished",
}


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _safe_scalar(value: Any) -> str | int | float | bool | None:
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        return value[:160]
    return str(value)[:160]


def _safe_value(value: Any) -> Any:
    if isinstance(value, (list, tuple, set)):
        return [_safe_scalar(item) for item in list(value)[:20]]
    return _safe_scalar(value)


class DiagnosticSession:
    def __init__(self, root: Path | None = None) -> None:
        self.root = root or gcs_log_dir() / "sessions"
        self.session_id = ""
        self.started_at: datetime | None = None
        self.session_dir: Path | None = None
        self.recording_enabled = False
        self._lock = threading.RLock()

    @property
    def active(self) -> bool:
        return self.session_dir is not None

    def start(self) -> str:
        with self._lock:
            if self.active:
                return self.session_id
            try:
                self.root.mkdir(parents=True, exist_ok=True)
                self._prune()
                self.started_at = _utc_now()
                self.session_id = uuid.uuid4().hex
                stamp = self.started_at.strftime("%Y%m%dT%H%M%SZ")
                self.session_dir = self.root / f"{stamp}_{self.session_id[:8]}"
                self.session_dir.mkdir(parents=True, exist_ok=False)
                self.recording_enabled = True
                self.emit("session_started", source="backend")
                return self.session_id
            except Exception:
                # Diagnostics are support tooling. Storage failure must never
                # prevent the GCS or a vehicle operation from starting.
                self.session_dir = None
                self.recording_enabled = False
                return ""

    def stop(self) -> None:
        with self._lock:
            if not self.active:
                return
            self.emit("session_finished", source="backend")
            self.session_dir = None
            self.recording_enabled = False

    def _prune(self) -> None:
        cutoff = _utc_now() - timedelta(days=MAX_SESSION_AGE_DAYS)
        dirs = sorted((p for p in self.root.iterdir() if p.is_dir()), reverse=True)
        for index, path in enumerate(dirs):
            try:
                modified = datetime.fromtimestamp(path.stat().st_mtime, timezone.utc)
                # Pruning happens immediately before a new session is created,
                # so retain at most N-1 existing directories here.
                if index >= MAX_RETAINED_SESSIONS - 1 or modified < cutoff:
                    for child in path.iterdir():
                        if child.is_file():
                            child.unlink()
                    path.rmdir()
            except OSError:
                continue

    def _timeline_path(self) -> Path:
        if self.session_dir is None:
            raise RuntimeError("diagnostic session has not started")
        return self.session_dir / "timeline.jsonl"

    def _rotate_if_needed(self, encoded_size: int) -> None:
        target = self._timeline_path()
        if not target.exists() or target.stat().st_size + encoded_size <= MAX_FILE_BYTES:
            return
        rollover = target.with_name("timeline.1.jsonl")
        if rollover.exists():
            rollover.unlink()
        target.replace(rollover)

    def emit(self, event: str, **fields: Any) -> None:
        with self._lock:
            if not self.active or not self.recording_enabled:
                return
            try:
                record = {
                    "ts": _utc_now().isoformat(timespec="milliseconds"),
                    "session_id": self.session_id,
                    "event": str(event)[:80],
                }
                for key, value in fields.items():
                    if key in ALLOWED_FIELDS and value is not None:
                        record[key] = _safe_value(value)
                encoded = (json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")
                self._rotate_if_needed(len(encoded))
                with self._timeline_path().open("ab") as stream:
                    stream.write(encoded)
                    stream.flush()
            except Exception:
                # Disable repeated writes for this process. Existing partial
                # files remain exportable, and request handling continues.
                self.recording_enabled = False

    def timeline_files(self) -> list[Path]:
        if self.session_dir is None:
            return []
        return [p for p in (self.session_dir / "timeline.1.jsonl", self.session_dir / "timeline.jsonl") if p.exists()]

    def export_zip(self, target: Path) -> None:
        with self._lock:
            manifest = {
                "session_id": self.session_id,
                "started_at": self.started_at.isoformat() if self.started_at else None,
                "format": "newline-delimited JSON, chronological across timeline.1.jsonl then timeline.jsonl",
                "privacy": "No coordinates, parameter names/values, request bodies, connection strings, or continuous telemetry.",
                "retention": {
                    "max_file_bytes": MAX_FILE_BYTES,
                    "max_files_per_session": MAX_SESSION_FILES,
                    "max_sessions": MAX_RETAINED_SESSIONS,
                    "max_age_days": MAX_SESSION_AGE_DAYS,
                },
            }
            with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as archive:
                for path in self.timeline_files():
                    archive.write(path, path.name)
                archive.writestr("manifest.json", json.dumps(manifest, indent=2))
                archive.writestr("summary.json", json.dumps(self.summary(), indent=2))

    def summary(self) -> dict[str, Any]:
        """Return compact last-known per-UAV outcomes and rendered plan shape."""
        with self._lock:
            vehicles: dict[str, dict[str, Any]] = {}
            final_plan = None
            for path in self.timeline_files():
                for line in path.read_text(encoding="utf-8").splitlines():
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    event = record.get("event")
                    if event == "plan_rendered":
                        final_plan = {
                            key: record.get(key)
                            for key in ("zone_count", "zone_sys_ids", "waypoint_counts")
                            if key in record
                        }
                    sys_id = record.get("sys_id")
                    if sys_id is None:
                        continue
                    vehicle = vehicles.setdefault(str(sys_id), {})
                    if event in {"vehicle_link_state", "vehicle_final_state"}:
                        vehicle["link"] = record.get("outcome")
                    elif event in {"mission_request_finished", "mission_backend_finished"}:
                        vehicle.setdefault("mission", {})["backend"] = {
                            "event": event, "outcome": record.get("outcome"),
                            "request_id": record.get("request_id"), "total": record.get("total"),
                        }
                    elif event in {"mission_result_accepted", "mission_result_rejected"} or (
                        event == "client_timeout" and record.get("phase") == "mission"
                    ):
                        vehicle.setdefault("mission", {})["client"] = {
                            "event": event, "outcome": record.get("outcome"),
                            "request_id": record.get("request_id"), "total": record.get("total"),
                        }
                    elif event == "parameter_request_finished":
                        vehicle.setdefault("parameters", {})["backend"] = {
                            "event": event, "outcome": record.get("outcome"),
                            "request_id": record.get("request_id"),
                            "record_count": record.get("record_count"),
                        }
                    elif event in {"parameter_snapshot_accepted", "parameter_snapshot_rejected"}:
                        vehicle.setdefault("parameters", {})["client"] = {
                            "event": event, "outcome": record.get("outcome"),
                            "request_id": record.get("request_id"),
                            "record_count": record.get("record_count"),
                        }
            return {"session_id": self.session_id, "vehicles": vehicles, "final_plan": final_plan}


diagnostic_session = DiagnosticSession()


def emit(event: str, **fields: Any) -> None:
    diagnostic_session.emit(event, **fields)
