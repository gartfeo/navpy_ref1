"""ConfirmOverrideListener - handles SWARM_REQUEST(FORCE_CONFIRM) (CONF-03,
D-18/D-19/D-20).

Nav-owned receive path for the operator's "Ask me anyway" one-shot override:
registered via ``NavController.set_network`` alongside ``TaskActor`` and
``ConfirmationManager`` so ``confirmation_manager.py`` stays untouched by this plan
(02-05 file-diff scope guard) -- FORCE_CONFIRM is deliberately NOT handled
in ``ConfirmationManager._handle_swarm_request`` (see that method's docstring).

Import this module where ConfirmOverrideListener is produced or consumed so
the SwarmRequestMsg registration (@register_msg) has already run via its own
import chain (navpy has no central message loader; each consumer imports its
own message module).
"""
from typing import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.messages.swarm_request_msg import (
    REQUEST_TYPE_FORCE_CONFIRM,
    SwarmRequestMsg,
)
from navpy.modules.comm.messages.types import MsgType


class ConfirmOverrideListener(ListenerAbc):
    """Nav-owned listener for the CONF-03 "Ask me anyway" gate-bypass
    override.

    Receives ``SWARM_REQUEST(request_type=FORCE_CONFIRM, subject_id=task_id)``
    from the GCS and invokes one explicit confirmation-policy callback. The
    nav loop consumes that one-shot flag the next tick the pixel/zoom gate
    would otherwise block that POI.
    """

    def __init__(
            self,
            sys_id: int,
            request_override: Callable[[int], None],
            logger,
    ):
        """
        :param sys_id: Vehicle sysid (the node identity), mirroring
            ConfirmationManager's receiver-addressing filter.
        :param request_override: Callback that records one task-id override.
        :param logger: Logger.
        """
        self._sys_id = sys_id
        self._request_override = request_override
        self._logger = logger

    def on_message(self, msg: MsgABC) -> None:
        if msg.receiver_id is not None and msg.receiver_id != self._sys_id:
            return
        if msg.msg_type() != MsgType.SWARM_REQUEST:
            return
        if not isinstance(msg, SwarmRequestMsg):
            return
        if msg.request_type != REQUEST_TYPE_FORCE_CONFIRM:
            return

        task_id = msg.subject_id
        self._request_override(task_id)
        self._logger.warning(
            f"CONFIRM override received (Ask me anyway) for P{task_id}",
            key='nav', dest=LogStatusDest.DRONE,
        )
