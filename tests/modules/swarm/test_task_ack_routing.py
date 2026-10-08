"""Inbound SWARM_ACKs reach the owner of the message type they ack."""

from unittest.mock import Mock

import pytest

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_ack_routing import TaskAckRouter, acked_ref
from navpy.modules.swarm.task_messaging import TaskMessageRouter
from navpy.modules.swarm.task_msg_refs import MsgRef


def _ack(ref_msg_type: int, sender_id: int = 2, receiver_id: int = 1):
    return SwarmAckMsg(
        sender_id=sender_id,
        receiver_id=receiver_id,
        ref_boot_id=7,
        ref_msg_seq=41,
        ref_msg_type=ref_msg_type,
        status=ACK_STATUS_RECEIVED,
        meta=MsgMeta(boot_id=9, msg_seq=3, time_ms=0, ttl_ms=5000),
    )


def test_ack_is_dispatched_by_the_acked_message_type():
    on_request, on_response = Mock(), Mock()
    router = TaskAckRouter(
        {
            MsgType.TASK_ASSIGN_REQUEST: on_request,
            MsgType.TASK_ASSIGN_RESPONSE: on_response,
        },
        Mock(spec=ILogger),
    )
    request_ack = _ack(MsgType.TASK_ASSIGN_REQUEST.value)
    response_ack = _ack(MsgType.TASK_ASSIGN_RESPONSE.value)

    router.route(request_ack)
    router.route(response_ack)

    on_request.assert_called_once_with(request_ack)
    on_response.assert_called_once_with(response_ack)


@pytest.mark.parametrize(
    "ref_msg_type",
    [MsgType.TASK_CONFIRM_REQUEST.value, 999],
    ids=["unhandled-type", "unknown-type"],
)
def test_ack_of_an_unhandled_type_is_ignored(ref_msg_type):
    handler = Mock()
    router = TaskAckRouter(
        {MsgType.TASK_ASSIGN_REQUEST: handler},
        Mock(spec=ILogger),
    )

    router.route(_ack(ref_msg_type))

    handler.assert_not_called()


def test_acked_ref_names_a_message_the_receiver_sent():
    assert acked_ref(_ack(MsgType.TASK_ASSIGN_REQUEST.value)) == MsgRef(
        sender_id=1, boot_id=7, msg_seq=41,
    )


def _task_router(handler, admitted):
    return TaskMessageRouter(
        actor_id=1,
        is_started=lambda: True,
        peer_is_admitted=lambda sender_id: sender_id in admitted,
        handlers={MsgType.SWARM_ACK: handler},
        logger=Mock(spec=ILogger),
    )


def test_ack_from_an_admitted_peer_is_routed():
    handler = Mock()
    ack = _ack(MsgType.TASK_ASSIGN_REQUEST.value)

    _task_router(handler, admitted={2}).route(ack)

    handler.assert_called_once_with(ack)


def test_ack_from_an_unadmitted_peer_or_for_another_node_is_dropped():
    handler = Mock()
    router = _task_router(handler, admitted={2})

    router.route(_ack(MsgType.TASK_ASSIGN_REQUEST.value, sender_id=4))
    router.route(_ack(MsgType.TASK_ASSIGN_REQUEST.value, receiver_id=3))

    handler.assert_not_called()
