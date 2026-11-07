"""The navigation command fence at the MAVLink send boundary (docs/design/navpy_recovery.md, step 2).

Contract: a write is admitted only under the send lock with the fence open and the generation current; closing takes
the fence lock only, so a send in flight or a hung worker cannot block it; a write admitted before closing goes out,
none after; an old generation is dropped; mode requests other than GUIDED are never fenced.
"""
from __future__ import annotations

import threading
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import MagicMock

from navpy.core.approach.navigation_command_session import formed_runtime_session
from navpy.core.approach.navigation_command_worker import NavigationCommandWorker, NavigationCommandWorkerPorts
from navpy.core.kernel.location import Location
from navpy.core.ports.flight_mode import FlightMode
from navpy.plugins.vehicle.ardupilot.attitude_command import AttitudeCommand
from navpy.plugins.vehicle.ardupilot.guided_navigation import GuidedNavigation
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.mode_control import ModeControl
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity

IDENTITY = VehicleIdentity(target_system=1, source_system=1, source_component=191, mav_type=1)
MODES = {"GUIDED": 15, "LOITER": 12, "AUTO": 10, "RTL": 11}
WAIT_S = 5.0


def _rig():
    connection = SimpleNamespace(mav=MagicMock(), mode_mapping=lambda: MODES)
    send_lock = threading.RLock()
    transport = MavTransport(connection, send_lock)
    logger = MagicMock()
    logger_ref = LoggerRef(logger)
    heartbeat = SimpleNamespace(message=None, count=0)  # no heartbeat: mode unknown, so goto asks for GUIDED
    mode = ModeControl(IDENTITY, transport, heartbeat, logger_ref)
    navigation = GuidedNavigation(
        IDENTITY, transport, mode,
        position=SimpleNamespace(home_location=None),
        parameters=MagicMock(),
        logger_ref=logger_ref,
    )
    attitude = AttitudeCommand(IDENTITY, transport, diagnostics=MagicMock())
    return SimpleNamespace(connection=connection, mav=connection.mav, send_lock=send_lock, transport=transport,
                           fence=transport.fence, logger=logger, mode=mode, navigation=navigation,
                           attitude=attitude)


def _sent_modes(mav):
    return [call.args[2] for call in mav.set_mode_send.call_args_list]


def test_write_admitted_before_closing_goes_out_and_none_after():
    rig = _rig()
    in_write, release = threading.Event(), threading.Event()

    def blocking_send(*_args):
        in_write.set()
        assert release.wait(WAIT_S)

    rig.mav.set_attitude_target_send.side_effect = blocking_send
    sender = threading.Thread(target=rig.attitude.set, args=(0.1, 0.0))
    sender.start()
    assert in_write.wait(WAIT_S)

    # The write is in flight and holds the send lock; closing must not wait for it.
    closer = threading.Thread(target=rig.fence.close)
    closer.start()
    closer.join(WAIT_S)
    assert not closer.is_alive()
    assert not rig.fence.is_open

    release.set()
    sender.join(WAIT_S)
    assert rig.mav.set_attitude_target_send.call_count == 1   # the one admitted before closing

    rig.mav.set_attitude_target_send.side_effect = None
    rig.attitude.set(0.2, 0.0)
    assert rig.mav.set_attitude_target_send.call_count == 1   # none after
    assert rig.fence.dropped == 1


def test_closing_never_takes_the_send_lock_held_by_a_hung_worker():
    rig = _rig()
    held, done = threading.Event(), threading.Event()

    def hung_worker():
        with rig.send_lock:
            held.set()
            done.wait(WAIT_S)

    worker = threading.Thread(target=hung_worker)
    worker.start()
    assert held.wait(WAIT_S)
    closed_generation = rig.fence.close()   # would block here if closing took the send lock
    assert closed_generation == 1 and not rig.fence.is_open
    done.set()
    worker.join(WAIT_S)


def test_command_formed_in_an_old_generation_is_dropped():
    rig = _rig()
    old = rig.fence.generation
    rig.fence.close()
    new = rig.fence.reopen()
    assert new > old

    with rig.fence.formed_in(old):
        rig.attitude.set(0.1, 0.0)
    rig.mav.set_attitude_target_send.assert_not_called()

    with rig.fence.formed_in(new):
        rig.attitude.set(0.1, 0.0)
    assert rig.mav.set_attitude_target_send.call_count == 1
    # A stated generation belongs to its thread and its block only.
    rig.attitude.set(0.1, 0.0)
    assert rig.mav.set_attitude_target_send.call_count == 2
    assert rig.fence.dropped == 1


def test_stated_generation_is_per_thread():
    rig = _rig()
    old = rig.fence.generation
    rig.fence.reopen()
    other_sent = []

    def other_thread():
        rig.attitude.set(0.1, 0.0)
        other_sent.append(rig.mav.set_attitude_target_send.call_count)

    with rig.fence.formed_in(old):
        thread = threading.Thread(target=other_thread)
        thread.start()
        thread.join(WAIT_S)
    assert other_sent == [1]   # the other thread stated nothing, so the open fence admits it


def test_closed_fence_drops_guided_targets_and_navpys_guided_request():
    rig = _rig()
    rig.fence.close()
    target = Location(47.0, 8.0, 100.0, is_absolute=False)

    rig.navigation.goto(target)
    rig.navigation.goto_loiter(target, 120.0)
    assert rig.mode.set(FlightMode.GUIDED) is False

    rig.mav.mission_item_send.assert_not_called()
    rig.mav.command_int_send.assert_not_called()
    rig.navigation._parameters.send_unverified.assert_not_called()  # the hold's LOITER radius stays
    assert _sent_modes(rig.mav) == []
    # One warning per generation, not one per 50 Hz command.
    assert sum("command fence" in str(c.args[0]) for c in rig.logger.warning.call_args_list) == 1


def test_open_fence_sends_guided_targets_as_before():
    rig = _rig()
    target = Location(47.0, 8.0, 100.0, is_absolute=False)
    rig.navigation.goto(target)
    rig.navigation.goto_loiter(target, 120.0)
    assert rig.mav.mission_item_send.call_count == 1
    assert rig.mav.command_int_send.call_count == 1
    assert _sent_modes(rig.mav) == [MODES["GUIDED"], MODES["GUIDED"]]
    assert rig.fence.dropped == 0


def test_hold_and_handover_modes_pass_a_closed_fence():
    rig = _rig()
    rig.fence.close()
    for mode in (FlightMode.LOITER, FlightMode.AUTO, FlightMode.RTL):
        assert rig.mode.set(mode) is True
    assert _sent_modes(rig.mav) == [MODES["LOITER"], MODES["AUTO"], MODES["RTL"]]


def test_close_is_idempotent_and_reopen_starts_a_new_generation():
    fence = MavCommandFence()
    assert fence.is_open and fence.generation == 0
    assert fence.close() == 1
    assert fence.close() == 1
    assert fence.reopen() == 2 and fence.is_open


def test_a_latched_fence_stays_closed_whichever_comes_first():
    fence = MavCommandFence()
    fence.latch_closed()
    fence.reopen()
    assert not fence.is_open
    fence = MavCommandFence()
    fence.close()
    generation = fence.reopen()
    assert fence.latch_closed() == generation + 1 and not fence.is_open
    fence.reopen()
    assert not fence.is_open


class _PausedRuntime:
    """One work item whose execution is paused across recovery's close and reopen, then sends its old command."""

    def __init__(self, rig, stop):
        self._rig, self._stop, self.taken = rig, stop, False

    def has_command_pending_or_in_flight(self):
        return not self.taken

    def take_work(self):
        self.taken = True
        return "work"

    def execute_work(self, _work):
        self._rig.fence.close()    # recovery's hold ...
        self._rig.fence.reopen()   # ... and resume, while this worker is paused
        self._rig.attitude.set(0.1, 0.0)
        self._stop.set()

    def postprocess_job(self, _work):
        return None

    def finish_work(self, _work):
        pass


def _run_worker(rig, formation=nullcontext):
    stop = threading.Event()
    runtime = _PausedRuntime(rig, stop)
    NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop, command_event=threading.Event(), wall_period_s=lambda period_s: period_s,
        runtime_session=formed_runtime_session(lambda: nullcontext(runtime), formation), logger=MagicMock(),
    )).run()


def test_a_command_formed_before_a_close_and_reopen_is_dropped():
    """Codex finding 2 on steps 1-4: the command worker states the generation it formed in."""
    rig = _rig()
    _run_worker(rig, rig.fence.formed_now)
    rig.mav.set_attitude_target_send.assert_not_called()
    assert rig.fence.dropped == 1
    rig.attitude.set(0.1, 0.0)   # formed after the reopen: goes out
    assert rig.mav.set_attitude_target_send.call_count == 1


def test_without_a_stated_generation_the_old_command_would_go_out():
    """What finding 2 described: by open or closed alone, the reopened fence admits the pre-recovery command."""
    rig = _rig()
    _run_worker(rig)
    assert rig.mav.set_attitude_target_send.call_count == 1
