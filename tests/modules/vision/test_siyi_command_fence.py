"""The SIYI command fence of the HITL terminal handoff and the real gimbal path (B3), design hitl_final_approach.md
sections 2, 4 and 7: once closed, only queries and the two stops reach the gimbal; a real run never gets a simulated
gimbal, and its SIYI SDK is built through the fence."""
from __future__ import annotations

import socket
import threading
import time
from unittest.mock import Mock, patch

import pytest

from navpy.plugins.gimbal.siyi.hardware.command_fence import SiyiCommandFence, passes_closed_fence
from navpy.plugins.gimbal.siyi.hardware.composition import create_siyi_sdk
from navpy.plugins.gimbal.siyi.siyi_message import SIYIMESSAGE

WAIT_S = 2.0                     # loopback answers in microseconds; this only bounds a failing test
LOOPBACK = "127.0.0.1"
CLOCK_TICK_S = 0.016             # Windows time.monotonic resolution: a 0.3 s join can measure up to one tick short

_MSG = SIYIMESSAGE()
QUERIES = {
    "firmware": _MSG.firmwareVerMsg(),
    "hardware id": _MSG.hwIdMsg(),
    "gimbal info": _MSG.gimbalInfoMsg(),
    "function feedback": _MSG.funcFeedbackMsg(),
    "attitude": _MSG.gimbalAttMsg(),
    "zoom reading": _MSG.requestCurrentZoomMsg(),
    "attitude stream": _MSG.dataStreamMsg(1, 50),
}
STOPS = {"zero rate": _MSG.gimbalSpeedMsg(0, 0), "zoom hold": _MSG.stopZoomMsg()}
COMMANDS = {
    "yaw rate": _MSG.gimbalSpeedMsg(1, 0),
    "pitch rate": _MSG.gimbalSpeedMsg(0, -1),
    "set angles": _MSG.setGimbalAttitude(100, -300),
    "centre": _MSG.centerMsg(),
    "lock mode": _MSG.lockModeMsg(),
    "follow mode": _MSG.followModeMsg(),
    "fpv mode": _MSG.fpvModeMsg(),
    "zoom in": _MSG.zoomInMsg(),
    "zoom out": _MSG.zoomOutMsg(),
    "absolute zoom": _MSG.absoluteZoomMsg(1),
    "autofocus": _MSG.autoFocusMsg(),
    "long focus": _MSG.longFocusMsg(),
    "focus hold": _MSG.stopFocusMsg(),
    "photo": _MSG.takePhotoMsg(),
    "record": _MSG.recordMsg(),
}


@pytest.mark.parametrize("name", sorted(QUERIES) + sorted(STOPS))
def test_queries_and_stops_pass_the_closed_fence(name):
    assert passes_closed_fence({**QUERIES, **STOPS}[name])


@pytest.mark.parametrize("name", sorted(COMMANDS))
def test_every_other_command_is_held(name):
    assert not passes_closed_fence(COMMANDS[name])


@pytest.mark.parametrize("msg", ["", "zz", "5566", "aa66" + _MSG.centerMsg()[4:], None])
def test_a_frame_that_cannot_be_decoded_is_held(msg):
    assert not passes_closed_fence(msg)


# ---- the fence on the real SDK, on the wire -----------------------------------------------------
@pytest.fixture
def gimbal_port():
    """A loopback socket standing in for the gimbal: the test reads what the SDK sends."""
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((LOOPBACK, 0))
    yield receiver
    receiver.close()


def received_commands(receiver: socket.socket) -> list[tuple[int, bytes]]:
    """Command id and data of every frame waiting on the socket."""
    frames = []
    receiver.settimeout(0.2)
    while True:
        try:
            frame, _ = receiver.recvfrom(1024)
        except socket.timeout:
            return frames
        frames.append((frame[7], frame[8:-2]))


def fenced_sdk(fence: SiyiCommandFence, receiver: socket.socket):
    return fence.wrap(create_siyi_sdk)(server_ip=LOOPBACK, port=receiver.getsockname()[1])


def test_the_closed_fence_holds_commands_on_the_wire_and_counts_them(gimbal_port):
    fence = SiyiCommandFence()
    sdk = fenced_sdk(fence, gimbal_port)
    assert sdk.requestGimbalSpeed(30, 0) is True
    assert [cmd for cmd, _ in received_commands(gimbal_port)] == [0x07]
    fence.close()
    for request in (lambda: sdk.requestGimbalSpeed(30, 0), sdk.requestCenterGimbal, sdk.requestLockMode,
                    lambda: sdk.requestAbsoluteZoom(5), sdk.requestZoomIn, sdk.requestAutoFocus):
        assert request() is True                 # a held command reports success: the hold raises nothing
    assert sdk.requestGimbalAttitude() is True and sdk.requestCurrentZoomLevel() is True
    assert received_commands(gimbal_port) == [(0x0D, b""), (0x18, b"")]
    assert fence.dropped == 6


def test_stop_sends_exactly_zero_rate_and_zoom_hold(gimbal_port):
    fence = SiyiCommandFence()
    fenced_sdk(fence, gimbal_port)
    fence.close()
    assert fence.quiesce_and_stop() == "sent"
    assert received_commands(gimbal_port) == [(0x07, bytes(2)), (0x05, bytes(1))]
    assert fence.dropped == 0


def test_an_sdk_created_after_the_close_is_fenced_too(gimbal_port):
    """A reconnect's setup (centre, lock, zoom reset, sdk_owner._initialize_payload) never reaches the gimbal."""
    fence = SiyiCommandFence()
    fenced_sdk(fence, gimbal_port)
    fence.close()
    reconnected = fenced_sdk(fence, gimbal_port)
    reconnected.requestCenterGimbal()
    reconnected.requestLockMode()
    reconnected.requestAbsoluteZoom(1)
    assert received_commands(gimbal_port) == [] and fence.dropped == 3
    assert fence.quiesce_and_stop() == "sent"
    assert [cmd for cmd, _ in received_commands(gimbal_port)] == [0x07, 0x05]


def test_an_open_fence_changes_nothing(gimbal_port):
    fence = SiyiCommandFence()
    sdk = fenced_sdk(fence, gimbal_port)
    sdk.requestCenterGimbal()
    sdk.requestAbsoluteZoom(2)
    assert len(received_commands(gimbal_port)) == 2 and fence.dropped == 0 and not fence.closed


STOPPED = [(0x07, bytes(2)), (0x05, bytes(1))]


def decoded(frames: list[str]) -> list[tuple[int, bytes]]:
    """Command id and data of each frame (sequence numbers differ between encoders)."""
    return [(bytes.fromhex(f)[7], bytes.fromhex(f)[8:-2]) for f in frames]


class FakeSdk:
    """An SDK whose send can be held (a send in progress) or fail."""

    def __init__(self, result: bool = True) -> None:
        self.entered, self.release = threading.Event(), threading.Event()
        self.release.set()
        self.sent: list[str] = []
        self.result = result

    def sendMsg(self, msg: str) -> bool:
        self.entered.set()
        assert self.release.wait(WAIT_S)
        self.sent.append(msg)
        return self.result

    def requestGimbalSpeed(self, yaw: int, pitch: int) -> bool:
        return self.sendMsg(_MSG.gimbalSpeedMsg(yaw, pitch))

    def requestZoomHold(self) -> bool:
        return self.sendMsg(_MSG.stopZoomMsg())


def test_the_stop_waits_for_the_send_in_progress():
    fence, sdk = SiyiCommandFence(stop_timeout_s=WAIT_S), FakeSdk()
    fence.wrap(lambda **_: sdk)(server_ip=LOOPBACK, port=1)
    sdk.release.clear()
    sender = threading.Thread(target=sdk.sendMsg, args=(COMMANDS["yaw rate"],))
    sender.start()
    assert sdk.entered.wait(WAIT_S)
    fence.close()
    result: list = []
    stopper = threading.Thread(target=lambda: result.append(fence.quiesce_and_stop()))
    stopper.start()
    stopper.join(0.2)
    assert stopper.is_alive() and sdk.sent == []
    sdk.release.set()
    stopper.join(WAIT_S)
    sender.join(WAIT_S)
    assert result == ["sent"]
    assert decoded(sdk.sent) == [(0x07, bytes([1, 0]))] + STOPPED


def test_a_send_stuck_for_good_is_a_bounded_timeout_and_the_fence_stays_closed():
    fence, sdk = SiyiCommandFence(stop_timeout_s=0.3), FakeSdk()
    fence.wrap(lambda **_: sdk)(server_ip=LOOPBACK, port=1)
    sdk.release.clear()
    threading.Thread(target=sdk.sendMsg, args=(COMMANDS["yaw rate"],), daemon=True).start()
    assert sdk.entered.wait(WAIT_S)
    fence.close()
    start = time.monotonic()
    assert fence.quiesce_and_stop() == "timeout"
    assert 0.3 - CLOCK_TICK_S <= time.monotonic() - start < WAIT_S
    assert fence.closed
    sdk.release.set()                            # let the stuck send and the late stop finish before the test ends


def test_a_stuck_endpoint_never_keeps_another_from_its_stop():
    fence, stuck, healthy = SiyiCommandFence(stop_timeout_s=0.3), FakeSdk(), FakeSdk()
    fence.wrap(lambda **_: stuck)(server_ip="192.168.144.25", port=37260)
    fence.wrap(lambda **_: healthy)(server_ip="192.168.144.26", port=37260)
    healthy.sendMsg(COMMANDS["yaw rate"])        # gimbal B is turning
    stuck.release.clear()
    threading.Thread(target=stuck.sendMsg, args=(QUERIES["attitude"],), daemon=True).start()
    assert stuck.entered.wait(WAIT_S)            # gimbal A's poll is stuck in its send
    fence.close()
    assert fence.quiesce_and_stop() == "timeout"
    assert decoded(healthy.sent) == [(0x07, bytes([1, 0]))] + STOPPED
    stuck.release.set()


def test_every_gimbal_endpoint_gets_the_stop():
    fence, first, second = SiyiCommandFence(), FakeSdk(), FakeSdk()
    fence.wrap(lambda **_: first)(server_ip="192.168.144.25", port=37260)
    fence.wrap(lambda **_: second)(server_ip="192.168.144.26", port=37260)
    fence.close()
    assert fence.quiesce_and_stop() == "sent"
    assert decoded(first.sent) == decoded(second.sent) == STOPPED


def test_one_gimbal_failing_its_stop_fails_the_stop():
    fence, good, bad = SiyiCommandFence(), FakeSdk(), FakeSdk(result=False)
    fence.wrap(lambda **_: good)(server_ip="192.168.144.25", port=37260)
    fence.wrap(lambda **_: bad)(server_ip="192.168.144.26", port=37260)
    fence.close()
    assert fence.quiesce_and_stop() == "failed"
    assert decoded(good.sent) == STOPPED


def test_a_reconnect_replaces_the_sdk_of_its_endpoint():
    fence, old, new = SiyiCommandFence(), FakeSdk(), FakeSdk()
    fence.wrap(lambda **_: old)(server_ip=LOOPBACK, port=1)
    fence.wrap(lambda **_: new)(server_ip=LOOPBACK, port=1)
    fence.close()
    assert fence.quiesce_and_stop() == "sent"
    assert old.sent == [] and decoded(new.sent) == STOPPED


def test_stop_reports_no_gimbal_and_failures():
    assert SiyiCommandFence().quiesce_and_stop() == "no_gimbal"
    failing = SiyiCommandFence()
    failing.wrap(lambda **_: FakeSdk(result=False))(server_ip=LOOPBACK, port=1)
    assert failing.quiesce_and_stop() == "failed"
    raising = SiyiCommandFence()
    sdk = FakeSdk()
    raising.wrap(lambda **_: sdk)(server_ip=LOOPBACK, port=1)
    sdk.requestZoomHold = Mock(side_effect=OSError("socket closed"))
    assert raising.quiesce_and_stop() == "failed"


def test_an_sdk_without_the_send_point_cannot_be_fenced():
    with pytest.raises(TypeError, match="sendMsg"):
        SiyiCommandFence().wrap(lambda **_: object())(server_ip=LOOPBACK, port=1)


# ---- B3: the real path builds the real SIYI client, through the fence ---------------------------
def _siyi_profile() -> dict:
    from navpy.core.perception.profile.vision_profile_loader import resolve_profile
    _, profile, _ = resolve_profile("siyi_zr10", Mock())
    return profile


def test_a_real_run_builds_the_siyi_client_through_the_fence():
    from navpy.app.perception.vision_camera_factory import build_camera_mounts
    fence = SiyiCommandFence()
    with patch("navpy.plugins.gimbal.siyi.gimbal_siyi.GimbalSiyi") as gimbal_siyi:
        build_camera_mounts(_siyi_profile(), Mock(), Mock(), use_sim_gimbal=False, siyi_fence=fence)
    assert gimbal_siyi.call_count == 1
    args, kwargs = gimbal_siyi.call_args
    assert args[1:3] == ("192.168.144.25", 37260)
    sdk = FakeSdk()
    factory = kwargs["sdk_factory"]
    with patch("navpy.plugins.gimbal.siyi.siyi_sdk.SIYISDK", return_value=sdk):
        assert factory(server_ip=LOOPBACK, port=1) is sdk
    fence.close()
    sdk.sendMsg(COMMANDS["centre"])
    assert fence.dropped == 1 and sdk.sent == []


def test_a_real_run_without_a_handoff_builds_the_plain_siyi_client():
    from navpy.app.perception.vision_camera_factory import build_camera_mounts
    with patch("navpy.plugins.gimbal.siyi.gimbal_siyi.GimbalSiyi") as gimbal_siyi:
        build_camera_mounts(_siyi_profile(), Mock(), Mock(), use_sim_gimbal=False)
    assert gimbal_siyi.call_args.kwargs["sdk_factory"] is None


def _all_profile_names() -> list[str]:
    import json
    from importlib import resources
    text = resources.files("navpy.core.perception.profile").joinpath("vision_profiles.json").read_text("utf-8")
    return sorted(name for name in json.loads(text)["profiles"] if name != "ideal_360")


@pytest.mark.parametrize("name", _all_profile_names())
def test_a_real_run_never_gets_a_simulated_gimbal(name):
    from navpy.app.perception.vision_profile_composition import build_vision_profile
    assembly = build_vision_profile(name, Mock(), Mock(), None, use_sim_gimbal=False, siyi_fence=SiyiCommandFence())
    assert assembly.mount_specs
    for spec in assembly.mount_specs:
        assert not type(spec.mount.gimbal).__module__.startswith("navpy.sim."), (name, type(spec.mount.gimbal))


@pytest.mark.parametrize(("detector_type", "use_sim_gimbal"), [("real", False), ("sim", True)])
def test_the_detector_type_selects_the_gimbal_path(detector_type, use_sim_gimbal):
    from navpy.app.perception.vision_controller_composition import build_vision_controller
    stop = RuntimeError("stop after the profile")
    fence = SiyiCommandFence()
    with patch("navpy.app.perception.vision_controller_composition.build_vision_profile",
               side_effect=stop) as build_profile:
        with pytest.raises(RuntimeError, match="stop after the profile"):
            build_vision_controller(Mock(), Mock(), Mock(detector_type=detector_type, profile_name="siyi_zr10"),
                                    Mock(), None, None, fence)
    assert build_profile.call_args.kwargs == {"use_sim_gimbal": use_sim_gimbal, "siyi_fence": fence}


def test_every_build_vision_profile_call_chooses_the_gimbal_path():
    """use_sim_gimbal is required: a caller that omits it fails at once, so every caller in the repo names it."""
    import ast
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    callers = []
    for path in [*root.joinpath("src").rglob("*.py"), *root.joinpath("scripts").rglob("*.py")]:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "build_vision_profile":
                callers.append(path.name)
                assert "use_sim_gimbal" in {kw.arg for kw in node.keywords}, path
    assert sorted(callers) == ["siyi_geo_track_child.py", "siyi_pixel_pn_child.py",
                               "vision_controller_composition.py"]
