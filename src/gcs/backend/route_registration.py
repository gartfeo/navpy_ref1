"""Optional GCS API route registration."""

from __future__ import annotations

import importlib
import logging
from dataclasses import dataclass

from fastapi import FastAPI


@dataclass(frozen=True)
class _RouteSpec:
    module_name: str
    prefix: str = ""
    unavailable_message: str = "Route module not available"


_ROUTES = (
    _RouteSpec("vehicles", "/api/vehicles", "Vehicle routes not available"),
    _RouteSpec("missions", "/api/vehicles", "Mission routes not available"),
    _RouteSpec("params", "/api/vehicles", "Params routes not available"),
    _RouteSpec("full_params", "/api/vehicles", "Full-params routes not available"),
    _RouteSpec("control", "/api/control", "Control routes not available"),
    _RouteSpec("navpy_sim", "/api/control", "NavPy sim routes not available"),
    _RouteSpec("task_confirm", "/api/control", "Task confirm routes not available"),
    _RouteSpec(
        "task_confirm_override",
        "/api/control",
        "Task confirm-override routes not available",
    ),
    _RouteSpec("settings", unavailable_message="Settings routes not available"),
    _RouteSpec(
        "vision_profiles",
        unavailable_message="Vision profile routes not available",
    ),
    _RouteSpec("logs", unavailable_message="Logs routes not available"),
    _RouteSpec("diagnostics", unavailable_message="Diagnostics routes not available"),
)


def register_optional_routes(
    app: FastAPI,
    logger: logging.Logger,
) -> list[str]:
    """Register every optional route module that can be imported.

    Returns the modules that failed to import, so ``/health`` can show a
    deployment with missing packages instead of only a log line.
    """
    unavailable: list[str] = []
    for spec in _ROUTES:
        try:
            module = importlib.import_module(f"gcs.backend.routes.{spec.module_name}")
        except ImportError:
            logger.warning(spec.unavailable_message)
            unavailable.append(spec.module_name)
            continue
        app.include_router(module.router, prefix=spec.prefix)
    return unavailable
