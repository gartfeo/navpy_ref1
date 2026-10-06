"""Canonical configuration values for certificate evaluation."""

from __future__ import annotations


CERTIFICATE_SPEEDUP = 10
CERTIFICATE_WIND_PARAMS: tuple[tuple[str, float], ...] = (
    ("SIM_WIND_SPD", 0.0),
    ("SIM_WIND_DIR", 0.0),
    ("SIM_WIND_TURB", 0.0),
)
CERTIFICATE_READBACK_PARAMS: tuple[str, ...] = (
    ("SIM_SPEEDUP",) + tuple(name for name, _ in CERTIFICATE_WIND_PARAMS)
)
CERTIFICATE_PARAM_TOLERANCE = 1e-3
IDENTITY_PROBE_TIMEOUT_S = 20.0


__all__ = [
    "CERTIFICATE_PARAM_TOLERANCE",
    "CERTIFICATE_READBACK_PARAMS",
    "CERTIFICATE_SPEEDUP",
    "CERTIFICATE_WIND_PARAMS",
    "IDENTITY_PROBE_TIMEOUT_S",
]
