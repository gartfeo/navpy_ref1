"""Isolated contact (step 26): who may open the contact link, the link itself, and where the service's dock comes
from. No flight profile or mode gets a contact; the UDP end drops foreign senders and leaves no thread."""
from __future__ import annotations

import json
import math
import socket
import threading
from argparse import ArgumentParser
from pathlib import Path
from unittest.mock import Mock

import pytest

from navpy.app.perception import hitl_contact_input as hci
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig, IntegrationTimingLimits
from navpy.core.perception.profile.vision_profile_loader import resolve_profile
from navpy.plugins.contact import hitl_contact_wire as wire
from navpy.plugins.contact.hitl_contact_sensor import ACCEPTED
from navpy.plugins.contact.hitl_contact_link import ContactLinkConfig, UdpContactLink, config_from_record, load_link
from tests.tools.hitl.contact_rig import KEY, RUN, VEHICLE, sign
from tools.hitl import contact_service as cs
from tools.hitl import scenario_public as sp
from tools.hitl import scenario_service as service
from tools.hitl.contact_scene import read_dock
from tools.hitl.json_checks import RecordError
from tools.tuning.hitl_bridge import unreal_pose

PROFILES = Path(__file__).resolve().parents[3] / "src/navpy/core/perception/profile/vision_profiles.json"
HITL_PROFILE = "siyi_zr10_hitl_encoded"


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _timing(mode: str = "pc-sitl", run_id: str = RUN) -> IntegrationTimingConfig:
    return IntegrationTimingConfig(run_id=run_id, mode=mode, source_id="s", link_id="l",
                                   limits=Mock(spec=IntegrationTimingLimits), attitude_correction_s=0.0,
                                   gimbal_correction_s=0.0)


def _link_file(tmp_path, mode="pc-sitl", run_id=RUN, service_port=None, navpy_port=None) -> Path:
    config = ContactLinkConfig(run_id, mode, KEY, ("127.0.0.1", service_port or _free_port()),
                               ("127.0.0.1", navpy_port or _free_port()))
    path = tmp_path / "contact_link.json"
    path.write_text(json.dumps(config.record()), encoding="utf-8")
    return path


def _profile(name: str):
    return resolve_profile(name, Mock())[1]


# --- 0 contact capability in flight profiles -------------------------------------------------------------------------

def test_only_the_hitl_profile_allows_a_contact_service():
    names = json.loads(PROFILES.read_text(encoding="utf-8"))["profiles"]
    allowed = sorted(name for name in names if hci.profile_allows_contact(_profile(name)))
    assert allowed == [HITL_PROFILE]
    assert not hci.profile_allows_contact(_profile("siyi_zr10"))     # the flight profile it extends


def test_the_flight_profile_is_refused_even_with_a_pc_sitl_link(tmp_path):
    with pytest.raises(ValueError, match="hitl_contact: true"):
        hci.open_hitl_contact(str(_link_file(tmp_path)), _timing(), _profile("siyi_zr10"), lambda: 1.0)


@pytest.mark.parametrize("timing, link, problem", [
    (None, {}, "needs --integration-timing"),
    (_timing(run_id="run-27"), {}, "another run's"),
    (_timing(mode="flight"), {}, "another run's"),
    (_timing(mode="flight"), {"mode": "flight"}, "may not use a contact service"),
    (_timing(mode="qualification"), {"mode": "qualification"}, "may not use a contact service"),
])
def test_a_link_without_a_pc_sitl_run_record_of_the_same_run_is_refused(tmp_path, timing, link, problem):
    with pytest.raises(ValueError, match=problem):
        hci.open_hitl_contact(str(_link_file(tmp_path, **link)), timing, _profile(HITL_PROFILE), lambda: 1.0)


def test_no_link_path_means_no_contact_and_a_malformed_profile_value_is_refused(tmp_path):
    assert hci.open_hitl_contact("", _timing(), _profile(HITL_PROFILE), lambda: 1.0) is None
    assert hci.with_hitl_contact(None, "", {}, Mock()) is None
    with pytest.raises(ValueError, match="true or false"):
        hci.profile_allows_contact({"hitl_contact": "yes"})


@pytest.mark.parametrize("change, problem", [
    ({"key_hex": "ab" * 16}, "32 bytes"), ({"service": "10.0.0.5:47001"}, "loopback"),
    ({"navpy": "localhost:47002"}, "IPv4"), ({"extra": 1}, "not a contact link"),
    ({"schema": "other"}, "not a contact link"), ({"run_id": "../x"}, "run id"),
])
def test_a_malformed_link_record_is_refused(change, problem):
    record = ContactLinkConfig(RUN, "pc-sitl", KEY, ("127.0.0.1", 1), ("127.0.0.1", 2)).record()
    with pytest.raises(ValueError, match=problem):
        config_from_record({**record, **change})


# --- the UDP end -----------------------------------------------------------------------------------------------------

def test_the_link_delivers_the_services_contact_drops_a_foreign_sender_and_closes_its_thread(tmp_path):
    service_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    service_sock.bind(("127.0.0.1", 0))
    path = _link_file(tmp_path, service_port=service_sock.getsockname()[1])
    contact = hci.open_hitl_contact(str(path), _timing(), _profile(HITL_PROFILE), lambda: 100.0)
    events, got = [], threading.Event()
    try:
        identity = Mock(vehicle=VEHICLE, task_id=10, dock_uid=4, episode=1)
        contact.arm(identity, lambda event: (events.append(event), got.set()))
        arm_datagram, navpy_address = service_sock.recvfrom(4096)
        arm = wire.decode(arm_datagram, KEY, frozenset({wire.KIND_ARM}))
        body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": arm["session"],
                "vehicle": VEHICLE, "task_id": 10, "dock_uid": 4, "episode": 1, "arm_id": arm["arm_id"],
                "board_time_us": 100_100_000}
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stranger:
            stranger.sendto(sign(body), navpy_address)            # right MAC, wrong sender: dropped unread
        service_sock.sendto(sign(body), navpy_address)
        assert got.wait(10.0)                                    # a hang guard, not a timing assertion
        assert len(events) == 1 and contact.link.foreign == 1
        assert contact.sensor.receive(sign(body)) == "replayed"
    finally:
        contact.close()
        contact.close()
        service_sock.close()
    assert not contact.link.running


def test_a_busy_receiver_keeps_the_link_open_until_a_later_close_joins_it(tmp_path):
    """No wall clock is asserted: the receiver is blocked on an event, so the first join must miss it."""
    from navpy.plugins.contact.hitl_contact_link import load_link

    entered, release = threading.Event(), threading.Event()

    def busy(_datagram):
        entered.set()
        release.wait(10.0)

    service_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    service_sock.bind(("127.0.0.1", 0))
    config = load_link(_link_file(tmp_path, service_port=service_sock.getsockname()[1]))
    link = UdpContactLink(config, busy, join_s=0.05)
    try:
        service_sock.sendto(b"x", config.navpy)
        assert entered.wait(10.0)
        assert link.close() is False and link.running
        release.set()
        assert link.close() is True and not link.running
    finally:
        release.set()
        link.close()
        service_sock.close()


@pytest.mark.parametrize("quiescent, joined, stopped", [
    (True, True, True), (False, True, False), (True, False, False), (False, False, False)])
def test_the_controller_closes_the_contact_on_every_stop_and_a_busy_link_leaves_it_incomplete(
        quiescent, joined, stopped):
    from types import SimpleNamespace

    from navpy.app.perception.vision_controller import VisionController

    closed = []
    contact = hci.HitlContact(Mock(), SimpleNamespace(close=lambda: closed.append(1) or joined))
    controller = VisionController.__new__(VisionController)
    controller._ports = SimpleNamespace(lifecycle=SimpleNamespace(stop=lambda: quiescent), integration_log=None,
                                        dock_contact=contact)
    assert controller.stop() is stopped and closed == [1]


@pytest.mark.parametrize("receipt_s, board_s", [
    (100.0, 42.0),                 # received now: the board time
    (100.0 - 1.0, 42.0),           # exactly the age limit
    (100.0 - 1.01, None),          # telemetry frozen past the limit: no board time
    (100.5, None),                 # a receipt ahead of now (a clock that went back): refused
    (None, None), (math.nan, None), (True, None),
])
def test_the_contact_board_clock_is_only_a_fresh_attitude(receipt_s, board_s):
    from types import SimpleNamespace

    from navpy.app.attitude_boot_time import fresh_attitude_boot_s

    # The wall receipt is deliberately fresh: only the monotonic capture receipt counts.
    vehicle = SimpleNamespace(attitude_sample=SimpleNamespace(time_boot_s=42.0, receipt_time_s=100.0,
                                                              capture_receipt_s=receipt_s))
    assert fresh_attitude_boot_s(vehicle, 1.0, now_s=lambda: 100.0) == board_s
    assert fresh_attitude_boot_s(SimpleNamespace(attitude_sample=None), 1.0, now_s=lambda: 100.0) is None


def test_a_frozen_attitude_refuses_a_late_contact_through_the_wiring(tmp_path):
    """Telemetry stops at board time 100 s; a contact stamped 100.2 s that arrives 5 s later is refused."""
    from types import SimpleNamespace

    from navpy.app.perception.integration_timing_input import IntegrationTimingInput

    wall = [1000.0]
    sample = SimpleNamespace(time_boot_s=100.0, capture_receipt_s=1000.0)
    vehicle = SimpleNamespace(attitude_sample=sample)
    service_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    service_sock.bind(("127.0.0.1", 0))
    path = _link_file(tmp_path, service_port=service_sock.getsockname()[1])
    integration = IntegrationTimingInput(_timing(), Mock())
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr("navpy.core.kernel.runtime_time.perf_counter", lambda: wall[0])
        contact = hci.with_hitl_contact(integration, str(path), _profile(HITL_PROFILE), vehicle).contact
        try:
            contact.arm(Mock(vehicle=VEHICLE, task_id=10, dock_uid=4, episode=1), lambda event: None)
            arm = wire.decode(service_sock.recvfrom(4096)[0], KEY, frozenset({wire.KIND_ARM}))
            body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": RUN, "session": arm["session"],
                    "vehicle": VEHICLE, "task_id": 10, "dock_uid": 4, "episode": 1, "arm_id": arm["arm_id"],
                    "board_time_us": 100_200_000}
            wall[0] = 1005.0                                   # no new ATTITUDE for 5 s
            assert contact.sensor.receive(sign(body)) == "no board time"
            sample.capture_receipt_s = 1005.0                  # telemetry back, the board clock at 100 s: fresh
            assert contact.sensor.receive(sign(body)) == ACCEPTED
        finally:
            assert contact.close()
            service_sock.close()


def test_navpy_takes_the_link_file_on_its_command_line():
    from navpy.app.vision_args import VisionArgs

    parser = ArgumentParser()
    VisionArgs.add_args(parser)
    assert VisionArgs.from_args(parser.parse_args(["--hitl-contact", "link.json"])).hitl_contact == "link.json"


# --- the link file and the service's dock ----------------------------------------------------------------------------

def test_the_link_file_is_written_once_and_only_in_the_private_folder(tmp_path):
    config = cs.new_link({"run": {"run_id": RUN, "mode": "pc-sitl"}}, "127.0.0.1:47001", "127.0.0.1:47002")
    assert len(config.key) == wire.KEY_BYTES
    with pytest.raises(RecordError, match="private folder"):
        cs.write_link(config, tmp_path / "outside" / "link.json", private_root=tmp_path / "private")
    path = cs.write_link(config, tmp_path / "private" / "run" / "link.json", private_root=tmp_path / "private")
    assert load_link(path) == config
    with pytest.raises(RecordError, match="one link per run"):
        cs.write_link(config, path, private_root=tmp_path / "private")
    with pytest.raises(ValueError, match="differ"):
        cs.new_link({"run": {"run_id": RUN, "mode": "pc-sitl"}}, "127.0.0.1:1", "127.0.0.1:1")


def _scenario(tmp_path, **applied_changes) -> Path:
    series = service.new_series("s1", service.datetime(2026, 11, 8, tzinfo=service.timezone.utc))
    path = tmp_path / "s1.json"
    path.write_text(json.dumps(series), encoding="utf-8")
    folder = tmp_path / "scenario"
    service.prepare(path, 0, folder)
    own = json.loads(sp.consumer_file(folder, "contact").read_text(encoding="utf-8"))
    board = {"location_cm": [1.0, 2.0, 3.0], "quat": [0.0, 0.0, 0.0, 1.0], "scale": [1.0, 1.0, 1.0],
             "width_cm": 100.0, "height_cm": 100.0, "centre_lat_lon_height_m": [40.1, 44.2, 1101.5]}
    applied = {"schema_version": 1.0, "scene_hash": own["scene_hash"],
               **{key: float(value) for key, value in own["scene"]["dock"].items()},
               "terrain_snap_cm": -1.0, "level": own["scene"]["level"], "board": board, **applied_changes}
    (folder / sp.PRIVATE_DIR / "unreal_applied.json").write_text(json.dumps(applied), encoding="utf-8")
    return folder


def test_the_service_takes_the_board_centre_unreal_built_for_its_own_scene(tmp_path):
    dock = read_dock(_scenario(tmp_path))
    assert (dock.lat, dock.lng, dock.alt) == (40.1, 44.2, 1101.5)


@pytest.mark.parametrize("change", [{"scene_hash": "f" * 64}, {"east_mm": 12345.0}])
def test_a_readback_of_another_scene_is_refused(tmp_path, change):
    with pytest.raises(RecordError, match="not the scene delivered"):
        read_dock(_scenario(tmp_path, **change))


def test_a_scenario_without_the_contact_consumer_is_refused(tmp_path):
    folder = _scenario(tmp_path)
    sp.consumer_file(folder, "contact").unlink()
    with pytest.raises(RecordError, match="no contact file"):
        read_dock(folder)


def test_every_new_series_names_the_contact_service():
    assert "contact" in sp.REQUIRED_CONSUMERS
    series = service.new_series("s1", service.datetime(2026, 11, 8, tzinfo=service.timezone.utc))
    assert series["consumers"] == ["unreal", "contact", "scoring"]


def test_the_bridge_sends_vehicle_state_to_every_copy():
    parser = unreal_pose.add_unreal_arguments(ArgumentParser())
    args = parser.parse_args(["--unreal-pose", "127.0.0.1:1", "30.0", "--vehicle-state-copy", "127.0.0.1:2",
                              "--vehicle-state-copy", "127.0.0.1:3"])
    sender = unreal_pose.unreal_pose_option(args)
    assert sender.copies == (("127.0.0.1", 2), ("127.0.0.1", 3))
    alone = unreal_pose.unreal_pose_option(parser.parse_args(["--vehicle-state-copy", "127.0.0.1:2",
                                                              "--vehicle-state-copy", "127.0.0.1:3"]))
    assert alone.endpoint == ("127.0.0.1", 2) and alone.copies == (("127.0.0.1", 3),)
    sender.close()
    alone.close()
