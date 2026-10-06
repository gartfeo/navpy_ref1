"""Tests for GET /api/logs/download/{sys_id}."""
import io
import zipfile
from unittest.mock import patch

import pytest
from fastapi.testclient import TestClient

from gcs.backend.main import app


@pytest.fixture
def client():
    return TestClient(app)


class TestLogsDownloadRoute:
    def test_returns_zip_with_log_files(self, tmp_path, client):
        """Latest session's log files are returned as a zip."""
        date_dir = tmp_path / "2026-02-15"
        session = date_dir / "090056"
        session.mkdir(parents=True)
        (session / "uav_1_navigation.log").write_text("log line")
        (session / "uav_1_navigation_compact.csv").write_text("csv,data")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/1")

        assert resp.status_code == 200
        assert resp.headers["content-type"] == "application/zip"
        assert "uav_1_" in resp.headers["content-disposition"]

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        names = sorted(zf.namelist())
        assert names == ["uav_1_navigation.log", "uav_1_navigation_compact.csv"]
        assert zf.read("uav_1_navigation.log") == b"log line"

    def test_returns_latest_session(self, tmp_path, client):
        """When multiple sessions exist, the newest one is returned."""
        old = tmp_path / "2026-02-14" / "100000"
        old.mkdir(parents=True)
        (old / "uav_1_navigation.log").write_text("old")

        new = tmp_path / "2026-02-15" / "120000"
        new.mkdir(parents=True)
        (new / "uav_1_navigation.log").write_text("new")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/1")

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        assert zf.read("uav_1_navigation.log") == b"new"

    def test_filters_by_sys_id(self, tmp_path, client):
        """Only files for the requested sys_id are included."""
        session = tmp_path / "2026-02-15" / "090056"
        session.mkdir(parents=True)
        (session / "uav_1_navigation.log").write_text("uav1")
        (session / "uav_2_navigation.log").write_text("uav2")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/2")

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        assert zf.namelist() == ["uav_2_navigation.log"]

    def test_includes_confirmation_artifacts(self, tmp_path, client):
        """Confirmation image artifacts from the selected session are zipped."""
        session = tmp_path / "2026-02-15" / "090056"
        session.mkdir(parents=True)
        (session / "uav_1_navigation.log").write_text("log")
        (session / "uav_1_confirmation_t10_090057000001_source.png").write_bytes(b"source")
        (session / "uav_1_confirmation_t10_090057000001_sent_thumb.jpg").write_bytes(b"thumb")
        (session / "uav_1_confirmation_t10_090057000001_meta.json").write_text("{}")
        (session / "uav_2_confirmation_t10_090057000001_source.png").write_bytes(b"other")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/1")

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        names = sorted(zf.namelist())
        assert names == [
            "uav_1_confirmation_t10_090057000001_meta.json",
            "uav_1_confirmation_t10_090057000001_sent_thumb.jpg",
            "uav_1_confirmation_t10_090057000001_source.png",
            "uav_1_navigation.log",
        ]
        assert zf.read("uav_1_confirmation_t10_090057000001_source.png") == b"source"

    def test_404_when_no_logs(self, tmp_path, client):
        """Returns 404 when no logs exist for the given sys_id."""
        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/99")

        assert resp.status_code == 404

    def test_404_when_logs_dir_missing(self, tmp_path, client):
        """Returns 404 when .logs/ directory doesn't exist."""
        missing = tmp_path / "nonexistent"
        with patch("gcs.backend.routes.logs.LOGS_ROOT", missing):
            resp = client.get("/api/logs/download/1")

        assert resp.status_code == 404

    def test_skips_sessions_without_matching_files(self, tmp_path, client):
        """Sessions with no matching files are skipped; older session used."""
        empty = tmp_path / "2026-02-16" / "120000"
        empty.mkdir(parents=True)
        (empty / "uav_2_navigation.log").write_text("other uav")

        older = tmp_path / "2026-02-15" / "100000"
        older.mkdir(parents=True)
        (older / "uav_1_navigation.log").write_text("found it")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/1")

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        assert zf.read("uav_1_navigation.log") == b"found it"

    def test_confirmation_only_session_does_not_change_latest_log_selection(self, tmp_path, client):
        """A newer artifact-only session does not replace the latest navigation session."""
        artifact_only = tmp_path / "2026-02-16" / "120000"
        artifact_only.mkdir(parents=True)
        (artifact_only / "uav_1_confirmation_t10_120001000000_source.png").write_bytes(b"source")

        older = tmp_path / "2026-02-15" / "100000"
        older.mkdir(parents=True)
        (older / "uav_1_navigation.log").write_text("found it")

        with patch("gcs.backend.routes.logs.LOGS_ROOT", tmp_path):
            resp = client.get("/api/logs/download/1")

        zf = zipfile.ZipFile(io.BytesIO(resp.content))
        assert zf.namelist() == ["uav_1_navigation.log"]
        assert zf.read("uav_1_navigation.log") == b"found it"
