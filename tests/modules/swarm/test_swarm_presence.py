import threading
import unittest
from unittest.mock import Mock, patch

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.modules.swarm.swarm_presence import PeerStatusPorts, SwarmPresence
from navpy.modules.swarm.task_actor_slots import SelectedTaskSlot
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_msg_refs import MsgRef
from navpy.modules.vehicle.vehicle_interface import IVehicle


def _presence(
    sender,
    *,
    lock=None,
    node_state=lambda: SwarmNodeState.FREE,
    discovered=None,
    reported=None,
    checked_out=None,
    tick=None,
    logger=None,
):
    return SwarmPresence(
        1,
        Mock(spec=IVehicle),
        sender,
        lock or threading.RLock(),
        node_state,
        PeerStatusPorts(
            discovered=discovered or Mock(),
            reported=reported or Mock(),
            checked_out=checked_out or Mock(),
            tick=tick or Mock(),
        ),
        logger or Mock(spec=ILogger),
    )


class SwarmPresenceTest(unittest.TestCase):
    def _presence(self):
        sender = Mock(spec=TaskMessageSender)
        return _presence(sender), sender

    def test_committed_start_announces_checkin_once_per_generation(self):
        presence, sender = self._presence()
        announced = threading.Event()
        sender.checkin.side_effect = lambda: announced.set() or True

        presence.start()
        self.assertTrue(announced.wait(1.0))
        presence.start()
        self.assertEqual(sender.checkin.call_count, 1)
        presence.stop()

        announced.clear()
        presence.start()
        try:
            self.assertTrue(announced.wait(1.0))
            self.assertEqual(sender.checkin.call_count, 2)
        finally:
            presence.stop()

    def test_checkin_admits_only_foreign_peer_and_replies_with_heartbeat(self):
        sender = Mock(spec=TaskMessageSender)
        discovered = Mock()
        presence = _presence(sender, discovered=discovered)

        presence.on_checkin(Mock(sender_id=1))
        presence.on_checkin(Mock(sender_id=7))

        discovered.assert_called_once_with(7)
        sender.heartbeat_message.assert_called_once_with(SwarmNodeState.FREE)
        sender.send_heartbeat.assert_called_once_with(
            sender.heartbeat_message.return_value,
        )

    def test_checkin_failure_stops_new_runtime_and_reraises_exact_failure(self):
        presence, sender = self._presence()
        heartbeat = Mock(spec=SwarmHeartbeatRuntime)
        heartbeat.start.return_value = True
        presence._heartbeat = heartbeat
        failure = RuntimeError("checkin failed")
        sender.checkin.side_effect = failure

        with self.assertRaises(RuntimeError) as raised:
            presence.start()

        self.assertIs(raised.exception, failure)
        heartbeat.stop.assert_called_once_with()

    def test_lifecycle_queries_delegate_to_heartbeat_runtime(self):
        presence, _sender = self._presence()
        heartbeat = Mock(spec=SwarmHeartbeatRuntime)
        thread = Mock(spec=threading.Thread)
        heartbeat.is_started.return_value = True
        heartbeat.heartbeat_thread.return_value = thread
        presence._heartbeat = heartbeat

        self.assertTrue(presence.is_started())
        self.assertIs(presence.heartbeat_thread(), thread)
        presence.raise_if_failed()
        presence.stop()

        heartbeat.raise_if_failed.assert_called_once_with()
        heartbeat.stop.assert_called_once_with()


def _offer(slot, task_id=10):
    task = TaskAssignMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(1.0, 2.0, 3.0),
    )
    return slot.on_request(task, MsgRef(3, 7, task_id), flyable=True)


class HeartbeatNodeStateTest(unittest.TestCase):
    """The heartbeat state is FREE unless the slot holds a task or nav
    flies a final approach."""

    def setUp(self):
        self.lock = threading.RLock()
        self.slot = SelectedTaskSlot(self.lock)
        self.network = Mock(spec=NetworkAbc)
        self.presence = _presence(
            TaskMessageSender(1, self.network, Mock(spec=ILogger)),
            lock=self.lock,
            node_state=self.slot.node_state,
        )

    def _reported_state(self, report):
        self.network.reset_mock()
        report()
        (message,) = [call.args[0] for call in self.network.broadcast.call_args_list]
        self.assertIsInstance(message, SwarmHeartbeatMsg)
        return SwarmNodeState(message.state)

    def _assert_reported(self, expected):
        for report in (
            self.presence.heartbeat,
            lambda: self.presence.on_checkin(Mock(sender_id=7)),
        ):
            self.assertIs(self._reported_state(report), expected)

    def test_empty_slot_and_no_approach_is_free(self):
        self._assert_reported(SwarmNodeState.FREE)

    def test_final_approach_is_busy(self):
        self.slot.set_approaching(True)
        self._assert_reported(SwarmNodeState.BUSY)

    def test_waiting_task_is_busy(self):
        _offer(self.slot)
        self._assert_reported(SwarmNodeState.BUSY)

    def test_assigned_task_is_busy_with_or_without_approach(self):
        decision = _offer(self.slot)
        ref = MsgRef(1, 5, 1)
        self.slot.record_reply(decision.held.token, ref)
        self.slot.on_ack(ref, 2, MsgRef(3, 7, 99))
        self._assert_reported(SwarmNodeState.BUSY)
        self.slot.set_approaching(True)
        self._assert_reported(SwarmNodeState.BUSY)

    def test_state_is_stamped_under_the_actor_lock_and_sent_outside(self):
        sender = Mock(spec=TaskMessageSender)
        held = []
        sender.heartbeat_message.side_effect = (
            lambda _state: held.append(("stamp", self.lock._is_owned()))
        )
        sender.send_heartbeat.side_effect = (
            lambda _message: held.append(("send", self.lock._is_owned()))
        )
        presence = _presence(sender, lock=self.lock)

        presence.heartbeat()

        self.assertEqual(held, [("stamp", True), ("send", False)])


class PeerHeartbeatTest(unittest.TestCase):
    def test_peer_heartbeat_reports_its_state_with_its_uid(self):
        discovered, reported = Mock(), Mock()
        presence = _presence(
            Mock(spec=TaskMessageSender),
            discovered=discovered,
            reported=reported,
        )

        presence.on_heartbeat(SwarmHeartbeatMsg(
            sender_id=7,
            state=SwarmNodeState.BUSY,
            meta=MsgMeta(boot_id=4, msg_seq=9, time_ms=0, ttl_ms=5000),
        ))
        presence.on_heartbeat(SwarmHeartbeatMsg(sender_id=1, state=1))

        discovered.assert_called_once_with(7)
        reported.assert_called_once_with(
            7, SwarmNodeState.BUSY, MsgRef(7, 4, 9),
        )

    def test_state_code_round_trips_and_any_nonzero_code_is_busy(self):
        for state in SwarmNodeState:
            message = SwarmHeartbeatMsg(
                sender_id=7,
                state=state,
                meta=MsgMeta(boot_id=4, msg_seq=9, time_ms=0, ttl_ms=5000),
            )
            mav = message.to_mavlink()
            mav._header.srcSystem = 7
            decoded = SwarmHeartbeatMsg.from_mavlink(mav)
            self.assertIs(SwarmNodeState.from_code(decoded.state), state)
        self.assertIs(SwarmNodeState.from_code(7), SwarmNodeState.BUSY)


if __name__ == "__main__":
    unittest.main()


class PeerPresenceEventTest(unittest.TestCase):
    def test_own_heartbeat_runs_the_silence_check_after_sending(self):
        sender = Mock(spec=TaskMessageSender)
        order = []
        sender.send_heartbeat.side_effect = lambda _message: order.append("send")
        tick = Mock(side_effect=lambda: order.append("tick"))

        _presence(sender, tick=tick).heartbeat()

        self.assertEqual(order, ["send", "tick"])

    def test_a_failing_silence_check_does_not_stop_the_heartbeat(self):
        # The heartbeat loop latches any error as fatal; a replan error in
        # the silence check must not end this node's heartbeats.
        sender = Mock(spec=TaskMessageSender)
        beats = threading.Semaphore(0)
        sender.send_heartbeat.side_effect = lambda _message: beats.release()
        logger = Mock(spec=ILogger)
        presence = _presence(
            sender,
            tick=Mock(side_effect=RuntimeError("replan failed")),
            logger=logger,
        )

        with patch(
            "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            0.01,
        ):
            presence.start()
            try:
                self.assertTrue(beats.acquire(timeout=1.0))
                self.assertTrue(beats.acquire(timeout=1.0))
                self.assertTrue(presence.is_started())
            finally:
                presence.stop()
        logger.error.assert_called()

    def test_check_in_is_heard_and_check_out_silences_a_foreign_peer(self):
        reported, checked_out = Mock(), Mock()
        presence = _presence(
            Mock(spec=TaskMessageSender),
            reported=reported,
            checked_out=checked_out,
        )
        location = LocationMsgData(1.0, 2.0, 3.0)

        presence.on_checkin(Mock(sender_id=7))
        presence.on_checkout(Mock(sender_id=7, location=location))
        presence.on_checkout(Mock(sender_id=1, location=location))

        reported.assert_called_once_with(7, None, None)
        checked_out.assert_called_once_with(7)
