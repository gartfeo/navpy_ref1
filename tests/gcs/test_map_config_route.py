"""GET /api/map-config: the Cesium Ion token comes from outside the repository."""
import pytest
from fastapi.testclient import TestClient

from gcs.backend import map_config
from gcs.backend.main import app


@pytest.fixture
def client():
    return TestClient(app)


@pytest.fixture
def token_file(tmp_path, monkeypatch):
    path = tmp_path / "cesium_ion_token"
    monkeypatch.delenv(map_config.CESIUM_ION_TOKEN_ENV, raising=False)
    monkeypatch.setenv(map_config.CESIUM_ION_TOKEN_FILE_ENV, str(path))
    return path


def test_no_token_configured(client, token_file):
    resp = client.get("/api/map-config")
    assert resp.status_code == 200
    assert resp.json() == {"cesium_ion_token": None}
    assert resp.headers["cache-control"] == "no-store"


def test_token_from_file_is_trimmed(client, token_file):
    token_file.write_text("  abc.def \n", encoding="utf-8")
    assert client.get("/api/map-config").json() == {"cesium_ion_token": "abc.def"}


def test_blank_file_means_no_token(client, token_file):
    token_file.write_text(" \n", encoding="utf-8")
    assert client.get("/api/map-config").json() == {"cesium_ion_token": None}


def test_env_var_wins_over_file(client, token_file, monkeypatch):
    token_file.write_text("from-file", encoding="utf-8")
    monkeypatch.setenv(map_config.CESIUM_ION_TOKEN_ENV, " from-env ")
    assert client.get("/api/map-config").json() == {"cesium_ion_token": "from-env"}


def test_file_added_later_needs_no_restart(client, token_file):
    assert client.get("/api/map-config").json()["cesium_ion_token"] is None
    token_file.write_text("later", encoding="utf-8")
    assert client.get("/api/map-config").json()["cesium_ion_token"] == "later"


def test_no_home_directory_means_no_token(client, monkeypatch):
    """A lockstep participant's environment has no home: the route answers, with no token."""
    monkeypatch.delenv(map_config.CESIUM_ION_TOKEN_ENV, raising=False)
    monkeypatch.delenv(map_config.CESIUM_ION_TOKEN_FILE_ENV, raising=False)

    def no_home():
        raise RuntimeError("Could not determine home directory.")

    monkeypatch.setattr(map_config.Path, "home", no_home)
    resp = client.get("/api/map-config")
    assert resp.status_code == 200
    assert resp.json() == {"cesium_ion_token": None}


def test_default_path_is_machine_level(monkeypatch, tmp_path):
    monkeypatch.delenv(map_config.CESIUM_ION_TOKEN_FILE_ENV, raising=False)
    monkeypatch.setattr(map_config.Path, "home", lambda: tmp_path)
    assert map_config.cesium_ion_token_path() == tmp_path / ".gcs" / "cesium_ion_token"
