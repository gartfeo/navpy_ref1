import unittest
from unittest.mock import Mock

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.task_messaging import TaskMessageRouter, TaskMessageSender


class TaskMessageSenderErrorBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.network = Mock(spec=NetworkAbc)
        self.logger = Mock(spec=ILogger)
        self.sender = TaskMessageSender(1, self.network, self.logger)

    def test_programmer_type_error_propagates(self):
        self.network.broadcast.side_effect = TypeError("bad call contract")

        with self.assertRaisesRegex(TypeError, "bad call contract"):
            self.sender.heartbeat()

    def test_os_error_is_contained_and_reported(self):
        self.network.broadcast.side_effect = OSError("link down")

        self.assertFalse(self.sender.heartbeat())
        self.logger.error.assert_called_once_with(
            "Failed to broadcast swarm heartbeat: link down"
        )


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
