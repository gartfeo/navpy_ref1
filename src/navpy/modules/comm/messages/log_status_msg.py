import logging
import traceback
from dataclasses import dataclass
from typing import Dict, Any

from navpy.modules.comm.messages.msg_abc import MsgABC, register_msg
from navpy.modules.comm.messages.types import MsgType
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_statustext_message, MAVLINK_MSG_ID_STATUSTEXT, MAV_SEVERITY_INFO)


_log = logging.getLogger(__name__)


def _make_non_string_status_tracer(max_traces: int = 5):
    """Diagnostic tracer for non-string LogStatusMsg.status values.

    LogStatusMsg wraps MAVLink STATUSTEXT which is text; any non-string
    status is a caller bug that will crash later inside to_mavlink().
    We trace the first few occurrences with the caller stack so the
    offending call site can be identified, then suppress to avoid log
    flooding if the bug repeats at a high rate.

    Delegated into a factory + closure so LogStatusMsg itself stays a
    pure message class (SRP) and the tracer is replaceable in tests
    without touching module state.
    """
    state = {"count": 0}

    def trace(sender_id, status) -> None:
        state["count"] += 1
        n = state["count"]
        if n <= max_traces:
            _log.error(
                "LogStatusMsg constructed with non-string status "
                "(count=%d, sender_id=%r, type=%s, repr=%r). Caller stack:\n%s",
                n,
                sender_id,
                type(status).__name__,
                status,
                "".join(traceback.format_stack()[:-2]),
            )
        elif n == max_traces + 1:
            _log.error(
                "LogStatusMsg non-string status repeated; suppressing "
                "further stacks (latest type=%s)",
                type(status).__name__,
            )

    return trace


_trace_non_string_status = _make_non_string_status_tracer()


@register_msg()
@dataclass
class LogStatusMsg(MsgABC):
    """
    Message class for logging status.
    """

    def __init__(self, sender_id, status):
        super().__init__(sender_id)
        # Diagnostic only: record the caller when a non-string status
        # slips through. We do NOT coerce to str -- that would mask the
        # root-cause bug. The downstream .encode() crash in to_mavlink()
        # still surfaces; this trace just tells us WHO to fix.
        if not isinstance(status, str):
            _trace_non_string_status(sender_id, status)
        self.status = status

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data.update({
            'status': self.status,
        })
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MsgABC':
        return cls(data['sid'], data['status'])

    def to_mavlink(self) -> MAVLink_statustext_message:
        return MAVLink_statustext_message(MAV_SEVERITY_INFO, self.status.encode('utf-8'))

    @classmethod
    def from_mavlink(cls, mav_msg: MAVLink_statustext_message) -> 'LogStatusMsg':
        status = mav_msg.text.rstrip('\x00')
        return cls(sender_id=mav_msg.get_srcSystem(), status=status)

    @classmethod
    def msg_type(cls) -> MsgType:
        return MsgType.LOG_STATUS

    @classmethod
    def mav_id(cls) -> int:
        return MAVLINK_MSG_ID_STATUSTEXT
