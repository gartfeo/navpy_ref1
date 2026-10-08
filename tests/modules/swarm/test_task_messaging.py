import unittest
from unittest.mock import Mock

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import (
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.ttl_defaults import get_ttl_ms
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.task_messaging import TaskMessageRouter, TaskMessageSender
from navpy.modules.swarm.task_msg_refs import msg_ref


class TaskMessageSenderErrorBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.network = Mock(spec=NetworkAbc)
        self.logger = Mock(spec=ILogger)
        self.sender = TaskMessageSender(1, self.network, self.logger)

    def test_programmer_type_error_propagates(self):
        self.network.broadcast.side_effect = TypeError("bad call contract")

        with self.assertRaisesRegex(TypeError, "bad call contract"):
            self._heartbeat()

    def test_os_error_is_contained_and_reported(self):
        self.network.broadcast.side_effect = OSError("link down")

        self.assertFalse(self._heartbeat())
        self.logger.error.assert_called_once_with(
            "Failed to broadcast swarm heartbeat: link down"
        )

    def _heartbeat(self):
        return self.sender.send_heartbeat(
            self.sender.heartbeat_message(SwarmNodeState.FREE),
        )


class TaskMessageSenderStampTest(unittest.TestCase):
    def setUp(self):
        self.network = Mock(spec=NetworkAbc)
        self.logger = Mock(spec=ILogger)
        self.sender = TaskMessageSender(1, self.network, self.logger)

    def _sent(self):
        return [call.args[0] for call in self.network.broadcast.call_args_list]

    def test_each_copy_is_stamped_before_sending_and_returns_its_uid(self):
        first = self.sender.assignment_response(2, 10, True)
        second = self.sender.assignment_response(2, 10, True)

        copies = self._sent()
        self.assertEqual([first, second], [msg_ref(copy) for copy in copies])
        self.assertEqual(first.sender_id, 1)
        self.assertEqual(first.boot_id, second.boot_id)
        self.assertGreater(second.msg_seq, first.msg_seq)
        self.assertEqual(
            copies[0].meta.ttl_ms,
            get_ttl_ms(MsgType.TASK_ASSIGN_RESPONSE),
        )

    def test_failed_send_returns_no_uid(self):
        self.network.broadcast.side_effect = OSError("link down")

        self.assertIsNone(self.sender.assignment_response(2, 10, True))

    def test_ack_names_the_message_and_lives_as_long_as_it(self):
        response = TaskAssignResponseMsg(
            sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
            meta=MsgMeta(boot_id=7, msg_seq=41, time_ms=0, ttl_ms=5000),
        )

        ref = self.sender.ack(response, ACK_STATUS_APPLIED)

        (ack,) = self._sent()
        self.assertIsInstance(ack, SwarmAckMsg)
        self.assertEqual(ref, msg_ref(ack))
        self.assertEqual(
            (ack.sender_id, ack.receiver_id, ack.ref_boot_id, ack.ref_msg_seq),
            (1, 2, 7, 41),
        )
        self.assertEqual(ack.ref_msg_type, MsgType.TASK_ASSIGN_RESPONSE.value)
        self.assertEqual(ack.status, ACK_STATUS_APPLIED)
        self.assertEqual(
            ack.meta.ttl_ms,
            get_ttl_ms(MsgType.TASK_ASSIGN_RESPONSE),
        )
        self.assertNotEqual(
            ack.meta.ttl_ms,
            get_ttl_ms(MsgType.SWARM_ACK),
        )

    def test_message_without_uid_cannot_be_acked(self):
        legacy = TaskAssignResponseMsg(
            sender_id=2, receiver_id=1, task_id=10, is_accepted=True,
        )

        self.assertIsNone(self.sender.ack(legacy, ACK_STATUS_APPLIED))

        self.network.broadcast.assert_not_called()
        self.logger.warning.assert_called_once()


class TaskMessageRouterErrorBoundaryTest(unittest.TestCase):
    def _router(self, handler, logger):
        return TaskMessageRouter(
            actor_id=1,
            is_started=lambda: True,
            peer_is_admitted=lambda _sender_id: True,
            handlers={MsgType.CHECK_IN: handler},
            logger=logger,
        )

    def test_programmer_attribute_error_propagates(self):
        logger = Mock(spec=ILogger)
        handler = Mock(side_effect=AttributeError("missing handler dependency"))

        with self.assertRaisesRegex(AttributeError, "missing handler dependency"):
            self._router(handler, logger).route(CheckInMsg(sender_id=2))

    def test_os_error_is_contained_and_reported(self):
        logger = Mock(spec=ILogger)
        handler = Mock(side_effect=OSError("listener transport failed"))

        self._router(handler, logger).route(CheckInMsg(sender_id=2))

        logger.error.assert_called_once_with(
            "Error handling message MsgType.CHECK_IN: listener transport failed"
        )


if __name__ == "__main__":
    unittest.main()
