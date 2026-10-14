"""The confirm response's blind resend schedule (``routes/task_confirm.py``).

The confirm response is a single unacknowledged MAVLink packet and the
companion resolves its 30 s window by timeout, so one lost packet leaves
the UI showing a decision the UAV never received. There is no response-ack
in the protocol; blind retransmission is the compensator. Duplicates are
safe: the companion pops the pending-confirmation token on the first copy
and logs later copies as duplicates. The lockstep slice's confirmation
timing gate reads the same schedule (``scripts/eval_gcs_demo_confirmation_timing.py``).
"""

TASK_CONFIRM_RESPONSE_SENDS = 3
TASK_CONFIRM_RESPONSE_RESEND_GAP_S = 0.25

__all__ = ["TASK_CONFIRM_RESPONSE_RESEND_GAP_S", "TASK_CONFIRM_RESPONSE_SENDS"]
