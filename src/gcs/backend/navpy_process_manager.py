"""Generation-fenced registry for GCS-managed NavPy subprocesses."""

from __future__ import annotations

import logging
import os
import subprocess
import threading

from gcs.backend.navpy_process_instance import (
    NavpyInstance,
    read_navpy_output,
    snapshot_navpy_instance,
    terminate_navpy_instance,
)
from gcs.backend.navpy_process_launch import (
    _build_env,
    _foreign_udp_holders,
    _normalize_log_level,
    _normalize_vision_profile,
    _ready_udp_owner_error,
    launch_navpy_instance,
)


log = logging.getLogger(__name__)

try:
    _speedup = float(os.environ.get("GCS_SIM_NAVIGATION_SPEEDUP", "0"))
except (TypeError, ValueError):
    _speedup = 0.0
if not (0.0 < _speedup < float("inf")):
    _speedup = 0.0
_GCS_SIM_NAVIGATION_SPEEDUP = _speedup


class NavpyProcessManager:
    """Own the exact active subprocess generation for each vehicle."""

    def __init__(self) -> None:
        self._instances: dict[int, NavpyInstance] = {}
        self._lock = threading.Lock()

    def start(
        self,
        sys_id: int,
        connection: str,
        detector_debug_show: bool = False,
        vision_profile: str | None = None,
        log_level: str | None = None,
    ) -> NavpyInstance:
        """Launch and register one NavPy companion generation."""
        squatters = _foreign_udp_holders(connection)
        if squatters:
            raise ValueError(
                f"companion UDP port for sys_id {sys_id} ({connection}) is "
                f"held by pid(s) {squatters} — refusing to double-bind"
            )
        with self._lock:
            existing = self._instances.get(sys_id)
            if existing is not None:
                if existing.process.poll() is None:
                    raise ValueError(
                        f"NavPy instance for sys_id {sys_id} already running"
                    )
                self._terminate(existing)
                if self._instances.get(sys_id) is existing:
                    del self._instances[sys_id]
            inst = launch_navpy_instance(
                sys_id,
                connection,
                detector_debug_show=detector_debug_show,
                vision_profile=vision_profile,
                log_level=log_level,
                navigation_speedup=_GCS_SIM_NAVIGATION_SPEEDUP,
                popen_factory=subprocess.Popen,
                output_reader=self._read_output,
                logger=log,
            )
            self._instances[sys_id] = inst
            log.info(
                "NavPy started: sys_id=%d, conn=%s, pid=%d",
                sys_id,
                connection,
                inst.process.pid,
            )
            return inst

    def stop(self, sys_id: int) -> bool:
        with self._lock:
            inst = self._instances.get(sys_id)
        if inst is None:
            return False
        self._terminate(inst)
        with self._lock:
            if self._instances.get(sys_id) is inst:
                self._instances.pop(sys_id, None)
        return True

    def stop_generation(self, sys_id: int, pid: int) -> bool:
        """Stop only the exact tracked generation, never a replacement."""
        with self._lock:
            inst = self._instances.get(sys_id)
            if inst is None or inst.process.pid != pid:
                return False
        self._terminate(inst)
        with self._lock:
            if self._instances.get(sys_id) is inst:
                self._instances.pop(sys_id, None)
        return True

    def stop_all(self) -> None:
        """Attempt every termination and retain generations that failed."""
        with self._lock:
            instances = list(self._instances.items())
        stopped: list[int] = []
        for sys_id, inst in instances:
            try:
                self._terminate(inst)
                stopped.append(sys_id)
            except Exception:
                log.exception(
                    "Failed to terminate NavPy instance sys_id=%d, pid=%s",
                    sys_id,
                    inst.process.pid,
                )
        original = dict(instances)
        with self._lock:
            for sys_id in stopped:
                if self._instances.get(sys_id) is original.get(sys_id):
                    self._instances.pop(sys_id, None)

    def get_status(self, sys_id: int) -> dict[str, object] | None:
        with self._lock:
            inst = self._instances.get(sys_id)
            return None if inst is None else self._snapshot(inst)

    def get_all_status(self) -> list[dict[str, object]]:
        with self._lock:
            return [self._snapshot(inst) for inst in self._instances.values()]

    def wait_ready(
        self,
        sys_id: int,
        pid: int,
        timeout_s: float,
    ) -> dict[str, object]:
        """Wait for one exact process generation to finish healthy startup."""
        with self._lock:
            inst = self._instances.get(sys_id)
            if inst is None or inst.process.pid != pid:
                raise RuntimeError(
                    f"NavPy instance sys_id={sys_id} pid={pid} is not tracked"
                )
        if not inst._ready_or_exit.wait(max(float(timeout_s), 0.0)):
            raise TimeoutError(
                f"NavPy instance sys_id={sys_id} pid={pid} did not become ready"
            )
        with self._lock:
            if self._instances.get(sys_id) is not inst:
                raise RuntimeError(
                    f"NavPy instance sys_id={sys_id} changed generation while waiting"
                )
            exit_code = inst.process.poll()
            payload = inst.ready_payload
        if payload is None:
            if inst._ready_error is not None:
                raise RuntimeError(
                    f"NavPy instance sys_id={sys_id} pid={pid} rejected "
                    f"readiness: {inst._ready_error}"
                )
            raise RuntimeError(
                f"NavPy instance sys_id={sys_id} pid={pid} exited before readiness "
                f"(exit_code={exit_code})"
            )
        if exit_code is not None:
            raise RuntimeError(
                f"NavPy instance sys_id={sys_id} pid={pid} exited after readiness "
                f"(exit_code={exit_code})"
            )
        return dict(payload)

    @staticmethod
    def _snapshot(inst: NavpyInstance) -> dict[str, object]:
        return snapshot_navpy_instance(inst)

    @staticmethod
    def _terminate(inst: NavpyInstance) -> None:
        terminate_navpy_instance(inst, log)

    def _read_output(self, inst: NavpyInstance) -> None:
        read_navpy_output(inst, self._validate_ready_owner)

    @staticmethod
    def _validate_ready_owner(
        inst: NavpyInstance,
        payload: dict[str, object],
    ) -> str | None:
        error = _ready_udp_owner_error(
            inst.connection,
            payload.get("process_pid"),
        )
        if error is None:
            return None
        try:
            terminate_navpy_instance(inst, log)
        except Exception as cleanup_error:
            return (
                f"{error}; failed to terminate rejected generation: "
                f"{type(cleanup_error).__name__}: {cleanup_error}"
            )
        return error


__all__ = [
    "NavpyInstance",
    "NavpyProcessManager",
    "_build_env",
    "_foreign_udp_holders",
    "_normalize_log_level",
]
