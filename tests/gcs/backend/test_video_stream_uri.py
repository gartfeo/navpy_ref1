"""Announced RTSP URI validation and WHEP URL derivation (section 2)."""
from __future__ import annotations

import ipaddress

import pytest
from pydantic import ValidationError

from gcs.backend import instance_ports as ip
from gcs.backend.settings_model import GcsSettings, VideoSettings
from gcs.backend.video_stream_policy import video_policy
from gcs.backend.video_stream_uri import RtspLocation, validate_rtsp_uri, whep_url

RADIO_LAN = (ipaddress.ip_network("192.168.144.0/24"),)


def _settings(sim: bool, test_source: bool = False) -> GcsSettings:
    settings = GcsSettings()
    settings.simulation.sim_mode = sim
    settings.simulation.video_test_source = test_source
    return settings


@pytest.mark.parametrize("uri, host, path", [
    ("rtsp://192.168.144.25:8554/main.264", "192.168.144.25", "main.264"),
    ("rtsp://192.168.144.10/tracking", "192.168.144.10", "tracking"),
    ("rtsp://192.168.144.10:8554/uav_1/cam-A.x", "192.168.144.10", "uav_1/cam-A.x"),
])
def test_accepts_rtsp_literal_ip_in_network(uri: str, host: str, path: str) -> None:
    assert validate_rtsp_uri(uri, RADIO_LAN).location == RtspLocation(host, path)


@pytest.mark.parametrize("uri, reason", [
    ("http://192.168.144.10:8554/main", "scheme"),
    ("rtsps://192.168.144.10:8554/main", "scheme"),
    ("rtsp://user:pw@192.168.144.10:8554/main", "userinfo"),
    ("rtsp://user@192.168.144.10/main", "userinfo"),
    ("rtsp://camera.local:8554/main", "host is not a literal IP"),
    ("rtsp://example.com/main", "host is not a literal IP"),
    ("rtsp://192.168.144.010/main", "host is not a literal IP"),
    ("rtsp://192.168.144.10:99999/main", "port"),
    ("rtsp://10.0.0.5:8554/main", "host outside allowed networks"),
    ("rtsp://127.0.0.1:20550/uav1", "host outside allowed networks"),
    ("rtsp://192.168.144.10:8554/", "path"),
    ("rtsp://192.168.144.10:8554", "path"),
    ("rtsp://192.168.144.10:8554/a//b", "path"),
    ("rtsp://192.168.144.10:8554/a/../b", "path"),
    ("rtsp://192.168.144.10:8554/..", "path"),
    ("rtsp://192.168.144.10:8554/a/./b", "path"),
    ("rtsp://192.168.144.10:8554/a%2e%2e", "path"),
    ("rtsp://192.168.144.10:8554/a b", "non-printable character"),
    ("rtsp://192.168.144.10:8554/a\tb", "non-printable character"),
    ("rtsp://192.168.144.10:8554/main?x=1", "query"),
    ("rtsp://192.168.144.10:8554/main?", "query"),
    ("rtsp://192.168.144.10:8554/main#frag", "fragment"),
    ("", "empty"),
    ("rtsp://[192.168.144.25/stream", "malformed authority"),
])
def test_rejects(uri: str, reason: str) -> None:
    verdict = validate_rtsp_uri(uri, RADIO_LAN)
    assert verdict.location is None
    assert verdict.reason == reason


def test_rejects_non_text() -> None:
    assert validate_rtsp_uri(None, RADIO_LAN).location is None


def test_loopback_only_in_sim_mode() -> None:
    uri = "rtsp://127.0.0.1:20550/uav1"
    assert validate_rtsp_uri(uri, video_policy(_settings(sim=False), 0).networks).location is None
    assert validate_rtsp_uri(uri, video_policy(_settings(sim=True), 0).networks).location \
        == RtspLocation("127.0.0.1", "uav1")


def test_whep_url_layout() -> None:
    assert whep_url(RtspLocation("192.168.144.25", "main.264"), 8889) \
        == "http://192.168.144.25:8889/main.264/whep"
    assert whep_url(RtspLocation("192.168.144.25", "a/b"), 9000) \
        == "http://192.168.144.25:9000/a/b/whep"


def test_whep_port_defaults_to_settings() -> None:
    assert video_policy(_settings(sim=False), 3).whep_port == 8889
    assert video_policy(_settings(sim=True), 3).whep_port == 8889
    # The test source applies only in sim mode.
    assert video_policy(_settings(sim=False, test_source=True), 3).whep_port == 8889


def test_whep_port_uses_slot_sim_port_with_test_source() -> None:
    policy = video_policy(_settings(sim=True, test_source=True), 3)
    assert policy.whep_port == ip.sim_video_whep_port(3) == 21553
    # Without a slot (no launcher) there is no sim MediaMTX port to use.
    assert video_policy(_settings(sim=True, test_source=True), None).whep_port == 8889


def test_video_settings_defaults_and_validation() -> None:
    settings = GcsSettings()
    assert settings.video.allowed_networks == ["192.168.144.0/24"]
    assert settings.video.whep_port == 8889
    assert settings.simulation.video_test_source is False
    with pytest.raises(ValidationError):
        VideoSettings(allowed_networks=["camera.local/24"])
    with pytest.raises(ValidationError):
        VideoSettings(whep_port=0)
