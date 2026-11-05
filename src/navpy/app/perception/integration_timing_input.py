"""The HITL runner's integration timing for one navpy run (stage 1 step 13).

``--integration-timing`` names the run's record (tools/hitl/approach_records.py): navpy reads only its run id,
mode and ``integration_timing`` section, and ``IntegrationTimingConfig.problems`` refuses anything else.
``--integration-log`` names a new JSONL file the associations write to (integration_log.py); a clean stop
ends it with one ``log_end`` record that counts the records before it, so a reader can tell a whole log from
one cut off at a line boundary.
"""
from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig, IntegrationTimingLimits
from navpy.core.perception.pipeline.integration_log import EVENT_LOG_END
from navpy.core.ports.frame_capture_ports import CaptureBackendWrapper
from navpy.app.approach.law_airspeed import LawAirspeed
from navpy.plugins.vehicle.ardupilot.airspeed_attach import AirspeedVehicle


def load_integration_timing(path: str | Path) -> IntegrationTimingConfig:
    """The run record's integration timing, refused when it is missing or not usable."""
    with open(path, encoding="utf-8") as handle:
        record: Any = json.load(handle)
    try:
        run, section = record["run"], record["integration_timing"]
        config = IntegrationTimingConfig(
            run_id=run["run_id"], mode=run["mode"], source_id=section["source_id"], link_id=section["link_id"],
            limits=IntegrationTimingLimits(**section["limits"]),
            attitude_correction_s=section["attitude_correction_s"],
            gimbal_correction_s=section["gimbal_correction_s"])
    except (KeyError, TypeError) as exc:
        raise ValueError(f"{path}: no usable integration_timing section ({exc!r})") from None
    problems = config.problems()
    if problems:
        raise ValueError(f"{path}: " + "; ".join(problems))
    return config


class JsonlTimingLog:
    """One run's integration log: a JSON line per record, flushed at once; ``close`` writes ``log_end``."""

    def __init__(self, path: str | Path, run_id: str) -> None:
        self._file = open(path, "x", encoding="utf-8")       # one log per run: never append to another's
        self._run_id = run_id
        self._records = 0
        self._lock = rt.Lock()

    def __call__(self, record: dict[str, object]) -> None:
        line = json.dumps(record, separators=(",", ":"))
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()
            self._records += 1

    def close(self) -> None:
        with self._lock:
            if self._file.closed:
                return
            end = {"event": EVENT_LOG_END, "run_id": self._run_id, "records": self._records}
            self._file.write(json.dumps(end, separators=(",", ":")) + "\n")
            self._file.close()


@dataclass(frozen=True)
class IntegrationTimingInput:
    config: IntegrationTimingConfig
    log: JsonlTimingLog
    # A commissioning launcher's capture wrapper (tools/hitl/media_fault_injector.py): only with integration timing,
    # whose config admits only pc-sitl, so never in flight or Qualification.
    capture_wrapper: CaptureBackendWrapper | None = None
    # HITL step 20: the law's only airspeed input (app/approach/law_airspeed.py), attached to the vehicle for the
    # config's mode by the vision composition; never VFR_HUD.
    airspeed: LawAirspeed | None = None
    # HITL step 21: the SIYI command fence's ``closed``, the search sweep's revoke source.
    gimbal_revoked: Callable[[], bool] | None = None


def open_integration_input(timing_path: str, log_path: str, detector_type: str,
                           capture_wrapper: CaptureBackendWrapper | None = None) -> IntegrationTimingInput | None:
    """Both files or neither, real detector only; the log is opened last, so a refused record leaves no file.
    A capture wrapper needs both."""
    if not timing_path and not log_path:
        if capture_wrapper is not None:
            raise ValueError("a capture wrapper needs --integration-timing and --integration-log (a pc-sitl run)")
        return None
    if not timing_path or not log_path:
        raise ValueError("--integration-timing and --integration-log go together")
    if detector_type != "real":
        raise ValueError("integration timing needs --detector-type=real")
    config = load_integration_timing(timing_path)
    return IntegrationTimingInput(config, JsonlTimingLog(log_path, config.run_id), capture_wrapper)


def with_law_airspeed(integration: IntegrationTimingInput | None, vehicle: AirspeedVehicle,
                      ) -> IntegrationTimingInput | None:
    """HITL step 20: attach the eligible airspeed source for the config's mode. A mode without a simulated pitot
    profile (flight, Qualification) refuses before any claim."""
    if integration is None:
        return None
    return replace(integration, airspeed=LawAirspeed.attach(vehicle, integration.config.mode))


class _ClosableFence(Protocol):
    @property
    def closed(self) -> bool: ...


def with_gimbal_revoke(integration: IntegrationTimingInput | None, fence: _ClosableFence | None,
                       ) -> IntegrationTimingInput | None:
    """HITL step 21: the search sweep stops for good once the SIYI command fence closes (terminal, deadline,
    abort); no fence, no revoke source, and a profile search is then refused."""
    if integration is None:
        return None
    return replace(integration, gimbal_revoked=None if fence is None else lambda: fence.closed)


__all__ = ["IntegrationTimingInput", "JsonlTimingLog", "load_integration_timing", "open_integration_input",
           "with_gimbal_revoke", "with_law_airspeed"]
