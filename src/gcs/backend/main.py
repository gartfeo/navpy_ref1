"""FastAPI application entry point for AAS GCS backend."""
from __future__ import annotations

import asyncio
import logging
import re
import time
import uuid
from contextlib import asynccontextmanager
from logging.handlers import RotatingFileHandler
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from gcs.backend.vehicle_manager import vehicle_mgr
from gcs.backend.routes import health, telemetry
from gcs.backend.diagnostics import diagnostic_session
from gcs.backend.route_registration import register_optional_routes
from gcs.backend.frontend_static import mount_frontend
from gcs.backend.runtime_paths import frontend_dist_dir, gcs_log_dir

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

STARTUP_AUTOCONNECT_ATTEMPTS = 24
STARTUP_AUTOCONNECT_SCAN_TIMEOUT_S = 5.0
STARTUP_AUTOCONNECT_RETRY_S = 2.0


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
        handler = RotatingFileHandler(
            target, maxBytes=5_000_000, backupCount=3, encoding="utf-8",
        )
        handler.setLevel(logging.INFO)
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(name)s] %(levelname)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        root.addHandler(handler)
        log.info("Backend file logging -> %s", target)
        return target
    except Exception as exc:  # never let logging setup break startup
        log.warning("Could not set up backend file logging: %s", exc)
        return None


async def _startup_autoconnect() -> None:
    """In launcher-managed sim mode, connect to THIS chat's SITL on startup so
    vehicles appear without waiting for a browser.

    Uses the chat's GCS monitor UDP port — the port the running SITL's router
    publishes — and retries quietly until SITL is up. Idempotent: ``add_vehicle``
    returns the existing entry if a vehicle is already connected (e.g. a browser
    also auto-connected), so this never double-connects.
    """
    from gcs.backend import instance_ports as ip
    from gcs.backend.settings_store import settings_store

    n = ip.chat_index()
    if n is None:
        return
    settings = settings_store.get()
    if not settings.simulation.sim_mode or not settings.connection.auto_connect:
        return

    device = ip.monitor_device(n)
    expected_sysids = set(ip.sysids_for_chat(n))
    connected_sysids = {
        sid for sid in vehicle_mgr.vehicles
        if sid in expected_sysids
    }
    navpy_started_sysids: set[int] = set()
    loop = asyncio.get_running_loop()
    for _ in range(STARTUP_AUTOCONNECT_ATTEMPTS):  # ~2 min of quiet retries while SITL boots
        try:
            entries = await loop.run_in_executor(
                None, vehicle_mgr.discover_and_connect,
                device, STARTUP_AUTOCONNECT_SCAN_TIMEOUT_S,
            )
        except Exception as exc:
            log.debug("Startup auto-connect scan failed: %s", exc)
            entries = []

        for entry in entries:
            sys_id = getattr(entry, "sys_id", None)
            if sys_id in expected_sysids:
                connected_sysids.add(sys_id)
        connected_sysids.update(
            sid for sid in vehicle_mgr.vehicles
            if sid in expected_sysids
        )

        to_start = sorted(connected_sysids - navpy_started_sysids)
        if to_start:
            try:
                from gcs.backend.routes.navpy_sim import auto_start_navpy_sim
                for sys_id in to_start:
                    if await asyncio.to_thread(auto_start_navpy_sim, sys_id):
                        navpy_started_sysids.add(sys_id)
            except Exception as exc:
                log.warning("Startup auto-connect NavPy start failed: %s", exc)

        missing = expected_sysids - connected_sysids
        if connected_sysids and not missing:
            log.info(
                "Startup auto-connect: %d/%d vehicle(s) on %s",
                len(connected_sysids), len(expected_sysids), device,
            )
            return
        await asyncio.sleep(STARTUP_AUTOCONNECT_RETRY_S)

    if connected_sysids:
        log.info(
            "Startup auto-connect: %d/%d vehicle(s) on %s; missing sys_ids %s",
            len(connected_sysids), len(expected_sysids), device, sorted(expected_sysids - connected_sysids),
        )
    else:
        log.info("Startup auto-connect: no vehicles found on %s", device)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup/shutdown lifecycle."""
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
    autoconnect_task = asyncio.create_task(_startup_autoconnect())
    yield
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
    request_id = request.headers.get("x-gcs-request-id") or uuid.uuid4().hex
    client_id = request.headers.get("x-gcs-client-id")
    relevant = path.startswith(_DIAGNOSTIC_HTTP_PREFIXES) and path != "/api/diagnostics/events"
    match = _SYS_ID_PATH.match(path)
    sys_id = int(match.group(1)) if match else None
    route = _diagnostic_route(path)
    started = time.perf_counter()
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
                duration_ms=round((time.perf_counter() - started) * 1000),
                outcome="cancelled" if exc.__class__.__name__ == "CancelledError" else "error",
                error_code=exc.__class__.__name__,
            )
        raise
    if relevant:
        diagnostic_session.emit(
            "http_request_finished", source="http", method=request.method,
            route=route, request_id=request_id, client_id=client_id, sys_id=sys_id,
            status_code=response.status_code,
            duration_ms=round((time.perf_counter() - started) * 1000),
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
