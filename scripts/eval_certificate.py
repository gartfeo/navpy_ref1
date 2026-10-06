"""Compatibility facade for certificate-grade evaluation support.

Implementations live in focused ``eval_certificate_*`` modules.  Imports here
are direct re-exports so existing callers observe the original object identities.
"""

from __future__ import annotations

from scripts.eval_certificate_clock import (
    MIN_RATE_SPAN_S,
    SOURCE_RESET_BACKSTEP_S,
    ClockRate,
    ClockRateTracker,
)
from scripts.eval_certificate_identity import (
    MISSION_IDENTITY_FIELDS,
    ParameterSnapshot,
    autopilot_identity,
    mission_identity,
    navpy_identity,
    parameter_snapshot_identity,
)
from scripts.eval_certificate_policy import (
    CERTIFICATE_IDENTITY_KEYS,
    CERTIFICATE_METRIC_KEYS,
    CERTIFICATE_RATE_TOLERANCE,
    certificate_consistency_errors,
    certificate_identity_errors,
    certificate_rate_error,
    certificate_summary,
)
from scripts.eval_certificate_profile import (
    CERTIFICATE_PARAM_TOLERANCE,
    CERTIFICATE_READBACK_PARAMS,
    CERTIFICATE_SPEEDUP,
    CERTIFICATE_WIND_PARAMS,
    IDENTITY_PROBE_TIMEOUT_S,
)
from scripts.eval_certificate_source_time import (
    SOURCE_TIME_ENV,
    SOURCE_TIME_FLUSH_GRACE_S,
    SOURCE_TIME_SUBDIR,
    source_time_row_metrics,
    source_time_summary,
)
from scripts.eval_certificate_statistics import (
    DistributionStats,
    MetricStats,
    distribution,
    percentile,
    summarize,
)
from scripts.eval_certificate_wsl import eeprom_identity, wsl_ardupilot_identity


__all__ = [
    "CERTIFICATE_IDENTITY_KEYS",
    "CERTIFICATE_METRIC_KEYS",
    "CERTIFICATE_PARAM_TOLERANCE",
    "CERTIFICATE_RATE_TOLERANCE",
    "CERTIFICATE_READBACK_PARAMS",
    "CERTIFICATE_SPEEDUP",
    "CERTIFICATE_WIND_PARAMS",
    "ClockRate",
    "ClockRateTracker",
    "DistributionStats",
    "IDENTITY_PROBE_TIMEOUT_S",
    "MIN_RATE_SPAN_S",
    "MISSION_IDENTITY_FIELDS",
    "MetricStats",
    "ParameterSnapshot",
    "SOURCE_RESET_BACKSTEP_S",
    "SOURCE_TIME_ENV",
    "SOURCE_TIME_FLUSH_GRACE_S",
    "SOURCE_TIME_SUBDIR",
    "autopilot_identity",
    "certificate_consistency_errors",
    "certificate_identity_errors",
    "certificate_rate_error",
    "certificate_summary",
    "distribution",
    "eeprom_identity",
    "mission_identity",
    "navpy_identity",
    "parameter_snapshot_identity",
    "percentile",
    "source_time_row_metrics",
    "source_time_summary",
    "summarize",
    "wsl_ardupilot_identity",
]
