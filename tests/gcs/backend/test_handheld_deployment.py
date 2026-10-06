"""Single-origin handheld deployment: env paths, frontend mount, route health."""
from __future__ import annotations

import logging

import pytest
from fastapi import APIRouter, FastAPI
from fastapi.testclient import TestClient

from gcs.backend import diagnostics, navpy_process_launch, route_registration, runtime_paths
from gcs.backend.frontend_static import mount_frontend
from gcs.backend.routes import health

_log = logging.getLogger(__name__)


def _dist(tmp_path):
    dist = tmp_path / "dist"
    (dist / "Cesium").mkdir(parents=True)
    (dist / "index.html").write_text("<html>gcs</html>", encoding="utf-8")
    (dist / "Cesium" / "Cesium.js").write_text("cesium", encoding="utf-8")
    return dist


def _app_with_api():
    app = FastAPI()
    router = APIRouter()

    @router.get("/api/ping")
    async def ping():
        return {"pong": True}

    @router.post("/api/command")
    async def command():
        return {"ok": True}

    app.include_router(router)
    return app


def _routing_replies(app):
    """Router replies that serving the frontend must leave unchanged."""
    client = TestClient(app, raise_server_exceptions=False)
    try:
        with client.websocket_connect("/ws/unknown"):
            websocket = "connected"
    except Exception as exc:  # noqa: BLE001 - the exception type is the reply
        websocket = type(exc).__name__
    return {
        "wrong method": client.get("/api/command").status_code,
        "unknown post": client.post("/api/nonexistent").status_code,
        "trailing slash": client.get("/api/ping/", follow_redirects=False).status_code,
        "unknown websocket": websocket,
    }


def test_log_dir_defaults_to_repo_logs(monkeypatch):
    monkeypatch.delenv(runtime_paths.LOG_DIR_ENV, raising=False)
    assert (runtime_paths._REPO_ROOT / "pyproject.toml").is_file()
    assert runtime_paths.gcs_log_dir() == runtime_paths._REPO_ROOT / ".logs" / "gcs"


def test_log_dir_env_override_reaches_diagnostics(monkeypatch, tmp_path):
    monkeypatch.setenv(runtime_paths.LOG_DIR_ENV, str(tmp_path / "logs"))
    assert runtime_paths.gcs_log_dir() == tmp_path / "logs"
    assert diagnostics.DiagnosticSession().root == tmp_path / "logs" / "sessions"


def test_no_frontend_dist_mounts_nothing(monkeypatch):
    monkeypatch.delenv(runtime_paths.FRONTEND_DIST_ENV, raising=False)
    app = _app_with_api()
    assert runtime_paths.frontend_dist_dir() is None
    assert mount_frontend(app, None, _log) is False
    assert TestClient(app).get("/").status_code == 404


def test_frontend_served_without_shadowing_api(tmp_path):
    app = _app_with_api()
    assert mount_frontend(app, _dist(tmp_path), _log) is True
    client = TestClient(app)
    assert client.get("/").text == "<html>gcs</html>"
    assert client.get("/Cesium/Cesium.js").text == "cesium"
    assert client.get("/api/ping").json() == {"pong": True}


def test_frontend_leaves_api_routing_replies_unchanged(tmp_path):
    served = _app_with_api()
    mount_frontend(served, _dist(tmp_path), _log)

    assert _routing_replies(served) == _routing_replies(_app_with_api()) == {
        "wrong method": 405,
        "unknown post": 404,
        "trailing slash": 307,
        "unknown websocket": "WebSocketDisconnect",
    }
    assert TestClient(served).get("/missing.js").status_code == 404


def test_frontend_dist_without_build_fails_startup(tmp_path):
    with pytest.raises(RuntimeError, match="index.html"):
        mount_frontend(_app_with_api(), tmp_path, _log)


def test_unavailable_route_modules_are_returned(monkeypatch):
    real_import = route_registration.importlib.import_module

    def fake_import(name, *args, **kwargs):
        if name == "gcs.backend.routes.logs":
            raise ImportError("missing dependency")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(route_registration.importlib, "import_module", fake_import)
    assert route_registration.register_optional_routes(FastAPI(), _log) == ["logs"]


def test_health_lists_unavailable_routes():
    app = FastAPI()
    app.include_router(health.router)
    app.state.unavailable_routes = ["logs"]
    assert TestClient(app).get("/health").json()["unavailable_routes"] == ["logs"]


def test_companion_pythonpath_uses_platform_separator(monkeypatch):
    # Android/Linux join PYTHONPATH with ":"; a hard-coded ";" would hand the
    # companion one bogus path entry instead of two.
    monkeypatch.setattr(navpy_process_launch.os, "pathsep", ":")
    monkeypatch.setenv("PYTHONPATH", "other")
    env = navpy_process_launch._build_env()
    assert env["PYTHONPATH"] == f"{navpy_process_launch._SRC_DIR}:other"
