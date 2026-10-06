"""Active-runtime source-time certification for the exact ideal demo."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from scripts.eval_certificate_source_time import (
    SOURCE_TIME_SUBDIR,
    source_time_row_metrics,
    source_time_summary,
)
from scripts.eval_ideal_three_uav_runtime_identity import runtime_identities
from scripts.eval_ideal_three_uav_source_gates import (
    REQUIRED_SOURCE_STREAMS,
    source_time_errors,
)


@dataclass
class SourceTimeRunEvidence:
    metrics_by_sys_id: dict[int, dict[str, Any]] = field(default_factory=dict)
    errors_by_sys_id: dict[int, list[str]] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)


def load_source_time_evidence(
    log_dir: Path,
    sys_ids: tuple[int, int, int],
) -> SourceTimeRunEvidence:
    """Select the exact ready Python generation and gate each source stage."""
    evidence = SourceTimeRunEvidence(
        errors_by_sys_id={sys_id: [] for sys_id in sys_ids}
    )
    try:
        identities = runtime_identities(log_dir, sys_ids)
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        evidence.errors.append(f"invalid active-runtime evidence: {error}")
        return evidence
    source_dir = log_dir / SOURCE_TIME_SUBDIR
    for sys_id in sys_ids:
        process_pid, scheduler_rate_hz = identities[sys_id]
        expected_names = {
            stream: (
                f"pose_cadence_{stream}_uav_{sys_id}_pid_{process_pid}.csv"
            )
            for stream in REQUIRED_SOURCE_STREAMS
        }
        missing = [
            name for name in expected_names.values()
            if not (source_dir / name).is_file()
        ]
        if missing:
            evidence.errors_by_sys_id[sys_id].append(
                f"active runtime pid {process_pid} is missing source-time files "
                f"{missing}"
            )
            continue
        include_files = [
            path.name
            for path in source_dir.glob(
                f"pose_cadence_*_uav_{sys_id}_pid_{process_pid}.csv"
            )
        ]
        try:
            summary = source_time_summary(
                source_dir,
                sys_id=sys_id,
                include_files=include_files,
            )
            metrics = source_time_row_metrics(summary)
        except (OSError, UnicodeError, TypeError, ValueError) as error:
            evidence.errors_by_sys_id[sys_id].append(
                f"invalid source-time evidence for active pid {process_pid}: "
                f"{error}"
            )
            continue
        metrics["process_pid"] = process_pid
        metrics["scheduler_rate_hz"] = scheduler_rate_hz
        evidence.metrics_by_sys_id[sys_id] = metrics
        evidence.errors_by_sys_id[sys_id].extend(
            source_time_errors(summary, scheduler_rate_hz)
        )
    return evidence


__all__ = ["SourceTimeRunEvidence", "load_source_time_evidence"]
