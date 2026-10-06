"""Single-drain dispatch and stream-request behavior of the evaluator link."""

from __future__ import annotations

from collections import deque
from types import SimpleNamespace

from pymavlink import mavutil

from scripts import eval_navigation_telemetry as telemetry


SYSID = 121


class _FakeMaster:
    """Queue-backed stand-in reproducing recv_match's discard semantics."""

    def __init__(self, messages) -> None:
        self._queue = deque(messages)
        self.target_system = SYSID
        self.target_component = 1
        self.sent = []
        self.mav = SimpleNamespace(
            command_long_send=lambda *args: self.sent.append(args)
        )

    def recv_match(self, type=None, blocking=False, timeout=None):
        # EXACT pymavlink semantics (mavutil.py recv_match): any filter that
        # is not a list or set -- a tuple included -- is wrapped as one
        # element, so a tuple filter matches nothing and every message is
        # discarded.  The first truth-scoring flight recorded zero SIM_STATE
        # rows through a healthy 40 Hz stream because this fake used to be
        # more lenient than the real API and let a tuple filter pass tests.
        if type is not None and not isinstance(type, (list, set)):
            type = [type]
        while self._queue:
            message = self._queue.popleft()
            if type is None or message.get_type() in type:
                return message
        return None


def _gpi(sysid: int = SYSID, time_boot_ms: int = 1000) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "GLOBAL_POSITION_INT",
        get_srcSystem=lambda: sysid,
        time_boot_ms=time_boot_ms,
        lat=430000000,
        lon=340000000,
        alt=500_000,
        relative_alt=60_000,
    )


def _sim_state(sysid: int = SYSID, time_us: int = 1_000_000) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "SIM_STATE",
        get_srcSystem=lambda: sysid,
        lat_int=430000000,
        lon_int=340000000,
        alt=500.0,
        time_us=time_us,
    )


class _Recorder:
    def __init__(self) -> None:
        self.calls = []

    def add_message(self, message, received_wall_time_s, *, scoring_active) -> None:
        self.calls.append((message.get_type(), scoring_active))


def _drain(master, *, scorer=None, truth=None, truth_scoring_active=False):
    anchors = deque(maxlen=2)
    completed = telemetry.drain_position_messages(
        master,
        sysid=SYSID,
        live_anchors=anchors,
        scorer=scorer,
        truth=truth,
        truth_scoring_active=truth_scoring_active,
    )
    return completed, anchors


def test_one_drain_routes_both_message_types() -> None:
    scorer = SimpleNamespace(samples=[], add=lambda s: scorer.samples.append(s))
    truth = _Recorder()
    master = _FakeMaster([
        _gpi(time_boot_ms=1000),
        _sim_state(time_us=1_000_000),
        _gpi(time_boot_ms=1033),
        _sim_state(time_us=1_025_000),
    ])
    completed, anchors = _drain(
        master, scorer=scorer, truth=truth, truth_scoring_active=True
    )
    assert completed
    assert len(scorer.samples) == 2
    assert truth.calls == [("SIM_STATE", True), ("SIM_STATE", True)]
    assert len(anchors) == 1


def test_drain_filter_type_survives_real_recv_match_semantics() -> None:
    """A tuple filter matches nothing in real pymavlink; the drain must not
    regress to one (live zero-truth-rows failure, 2026-09-03)."""
    assert isinstance(telemetry._DRAIN_MESSAGE_TYPES, (list, set))


def test_sim_state_alone_does_not_mark_the_gpi_live_edge() -> None:
    truth = _Recorder()
    master = _FakeMaster([_sim_state(), _sim_state()])
    completed, anchors = _drain(master, truth=truth)
    assert completed
    assert truth.calls == [("SIM_STATE", False), ("SIM_STATE", False)]
    assert len(anchors) == 0


def test_sim_state_is_discarded_without_a_truth_consumer() -> None:
    scorer = SimpleNamespace(samples=[], add=lambda s: scorer.samples.append(s))
    master = _FakeMaster([_sim_state(), _gpi()])
    completed, _ = _drain(master, scorer=scorer)
    assert completed
    assert len(scorer.samples) == 1


def test_wrong_sysid_sim_state_is_dropped() -> None:
    truth = _Recorder()
    master = _FakeMaster([_sim_state(sysid=7), _sim_state()])
    _drain(master, truth=truth)
    assert len(truth.calls) == 1


def _ack(result: int) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "COMMAND_ACK",
        get_srcSystem=lambda: SYSID,
        command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        result=result,
    )


def test_truth_stream_request_sends_sim_state_interval() -> None:
    master = _FakeMaster([_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED)])
    accepted = telemetry.request_message_interval_stream(
        master, mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 40.0
    )
    assert accepted
    assert len(master.sent) == 1
    args = master.sent[0]
    assert args[2] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    assert args[4] == mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE
    assert args[5] == 25000


def test_coordinate_stream_request_uses_the_shared_helper() -> None:
    master = _FakeMaster([_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED)])
    assert telemetry.request_coordinate_score_stream(master)
    args = master.sent[0]
    assert args[4] == mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
    assert args[5] == 33333


def test_denied_interval_request_returns_false() -> None:
    master = _FakeMaster([_ack(mavutil.mavlink.MAV_RESULT_DENIED)])
    assert not telemetry.request_message_interval_stream(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
        40.0,
        timeout_s=0.2,
    )
