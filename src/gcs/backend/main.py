"""FastAPI application entry point for AAS GCS backend."""
from __future__ import annotations

import asyncio
import logging
import re
from contextlib import asynccontextmanager
from logging.handlers import RotatingFileHandler
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.routes import health, telemetry
from gcs.backend.diagnostics import diagnostic_session
from gcs.backend.producer_inventory import producers
from gcs.backend.route_registration import register_optional_routes
from gcs.backend.frontend_static import mount_frontend
from gcs.backend.runtime_paths import frontend_dist_dir, gcs_log_dir
from gcs.backend.startup_autoconnect import startup_autoconnect
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging import pose_cadence_debug
from navpy.core.kernel.logging.log_output_latch import LatchingRotatingFileHandler, OutputLatch
from navpy.core.kernel.logging.log_rotation import rotation_max_bytes

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)
#: Set by any backend file-log write (or setup) failure; a lockstep drain checks it.
file_log_latch = OutputLatch()
# The backend's own MAVLink readers record arrivals too; keep them apart from
# the companions' files.
pose_cadence_debug.set_producer("gcs")


def _add_file_logging(log_dir: Path | None = None) -> Path | None:
    """Persist backend logs (incl. the [launch] sequence) to a rotating file so
    runs can be inspected after the fact — the console handler alone isn't
    capturable. Idempotent; failures are non-fatal."""
    if log_dir is None:
        log_dir = gcs_log_dir()
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        target = log_dir / "backend.log"
        root = logging.getLogger()
        for h in root.handlers:
            if isinstance(h, RotatingFileHandler) and getattr(h, "baseFilename", None) == str(target):
                return target  # already attached
        handler = LatchingRotatingFileHandler(
            target, maxBytes=rotation_max_bytes(5_000_000), backupCount=3, encoding="utf-8",
            latch=file_log_latch,
        )
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(name)s] %(levelname)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        root.addHandler(handler)
        log.info("Backend file logging -> %s", target.as_posix())
        return target
    except Exception as exc:  # never let logging setup break startup
        file_log_latch.latch()
        log.warning("Could not set up backend file logging: %s", exc)
        return None


def check_file_logging() -> None:
    """Raise ``LogOutputFailed`` if a backend log line failed to reach its file."""
    for handler in logging.getLogger().handlers:
        handler.flush()
    file_log_latch.check()


async def startup() -> asyncio.Task:
    """Lifespan startup; returns the auto-connect task shutdown cancels.

    The lockstep backend participant runs it at its admission tick and has
    its own strict shutdown (``gcs.backend.lockstep.shutdown``).
    """
    _add_file_logging()
    diagnostic_session.start()
    log.info("AAS GCS backend starting...")
    # Task confirmation listener — intercepts NAVLINK + image messages
    try:
        from gcs.backend.task_confirm_listener import TaskConfirmListener
        listener = TaskConfirmListener(asyncio.get_running_loop())
        vehicle_mgr.set_task_confirm_listener(listener)
        log.info("TaskConfirmListener initialized")
    except ImportError:
        log.warning("TaskConfirmListener not available")

    # Task assignment listener — intercepts assign request/response NAVLINK messages
    try:
        from gcs.backend.task_assign_listener import TaskAssignListener
        assign_listener = TaskAssignListener(asyncio.get_running_loop())
        vehicle_mgr.set_task_assign_listener(assign_listener)
        log.info("TaskAssignListener initialized")
    except ImportError:
        log.warning("TaskAssignListener not available")

    # Compass-cal listener — rebroadcasts MAG_CAL progress/report to the UI
    try:
        from gcs.backend.compass_cal_listener import CompassCalListener
        compass_cal_listener = CompassCalListener(asyncio.get_running_loop())
        vehicle_mgr.set_compass_cal_listener(compass_cal_listener)
        log.info("CompassCalListener initialized")
    except ImportError:
        log.warning("CompassCalListener not available")

    await vehicle_mgr.start_telemetry_loop()
    # Server-side auto-connect to this chat's running SITL (no browser needed).
    return producers.create_task("autoconnect", startup_autoconnect())


async def shutdown(autoconnect_task: asyncio.Task) -> None:
    """Lifespan shutdown: best effort, as a desktop backend exits."""
    autoconnect_task.cancel()
    log.info("AAS GCS backend shutting down...")
    # Stop ESP32 simulator if it was started via the API
    try:
        from gcs.backend.routes.control import stop_esp32_sim_if_running
        stop_esp32_sim_if_running()
    except Exception:
        pass
    # Stop NavPy simulation instances if any were started
    try:
        from gcs.backend.routes.navpy_sim import stop_all_navpy_if_running
        await asyncio.to_thread(stop_all_navpy_if_running)
    except Exception:
        pass
    await vehicle_mgr.stop_telemetry_loop()
    vehicle_mgr.shutdown()
    diagnostic_session.stop()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Wall lifecycle (lockstep runs ``startup`` itself): the CPU share or background is
    set before any NavPy starts, released after they stop (``bounded_cpu_share.md``)."""
    from gcs.backend.cpu_share_holder import stack_scheduling

    _add_file_logging()  # so the share's outcome reaches backend.log
    log.info("Stack scheduling: %s", stack_scheduling.start_from_env())
    autoconnect_task = await startup()
    yield
    await shutdown(autoconnect_task)
    await asyncio.to_thread(stack_scheduling.give_up, "backend shutdown")


app = FastAPI(
    title="AAS GCS",
    version="1.0.0",
    lifespan=lifespan,
)


_DIAGNOSTIC_HTTP_PREFIXES = (
    "/api/vehicles", "/api/control/navpy-sim", "/api/diagnostics/export",
)
_SYS_ID_PATH = re.compile(r"/api/vehicles/(\d+)(?:/|$)")


def _diagnostic_route(path: str) -> str:
    return _SYS_ID_PATH.sub("/api/vehicles/{sys_id}/", path)


@app.middleware("http")
async def diagnostic_http_summary(request, call_next):
    """Persist bounded HTTP outcomes without query strings or bodies."""
    path = request.url.path
    request_id = request.headers.get("x-gcs-request-id") or rt.uuid4().hex
    client_id = request.headers.get("x-gcs-client-id")
    relevant = path.startswith(_DIAGNOSTIC_HTTP_PREFIXES) and path != "/api/diagnostics/events"
    match = _SYS_ID_PATH.match(path)
    sys_id = int(match.group(1)) if match else None
    route = _diagnostic_route(path)
    started = rt.perf_counter()
    if relevant:
        diagnostic_session.emit(
            "http_request_started", source="http", method=request.method,
            route=route, request_id=request_id, client_id=client_id, sys_id=sys_id,
        )
    try:
        response = await call_next(request)
    except BaseException as exc:
        if relevant:
            diagnostic_session.emit(
                "http_request_finished", source="http", method=request.method,
                route=route, request_id=request_id, client_id=client_id, sys_id=sys_id,
                duration_ms=round((rt.perf_counter() - started) * 1000),
                outcome="cancelled" if exc.__class__.__name__ == "CancelledError" else "error",
                error_code=exc.__class__.__name__,
            )
        raise
    if relevant:
        diagnostic_session.emit(
            "http_request_finished", source="http", method=request.method,
            route=route, request_id=request_id, client_id=client_id, sys_id=sys_id,
            status_code=response.status_code,
            duration_ms=round((rt.perf_counter() - started) * 1000),
            outcome="success" if response.status_code < 400 else "failure",
        )
    response.headers["X-GCS-Session-ID"] = diagnostic_session.session_id
    response.headers["X-GCS-Request-ID"] = request_id
    return response

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routes
app.include_router(health.router)
app.include_router(telemetry.router)
app.state.unavailable_routes = register_optional_routes(app, log)
mount_frontend(app, frontend_dist_dir(), log)
