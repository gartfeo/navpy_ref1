"""The determinism trace's vocabulary: what a row can say happened.

Its own module because it is the contract between three parties that must not
depend on each other -- the recorder that writes rows, the reader that reduces
them, and the source adapter that names its own decision points. A name added
here without a producer, or dropped while a reader still expects it, is the
kind of drift a shared vocabulary exists to prevent.

Row layouts (tuples, not dicts: the hot path allocates one tuple per event).
Indices 0-2 are the event, the epoch and the outcome (or reason) in every
layout. Indices 3-4 are a microsecond stamp and its slot in every layout but
three: LIFECYCLE, whose index 3 is the epoch the boundary made current,
SUBSCRIPTION, whose index 3 is a ruling, and VIOLATION::

    (ASSOCIATION,  epoch, outcome, source_us, slot, ruling)
    (TRUTH,        epoch, outcome, watermark_us, watermark_slot, raw_sample)
    (STAGE,        epoch, outcome, source_us, slot, watermark_us,
                   watermark_slot, digest)
    (DECIMATE,     epoch, reason,  source_us, slot, watermark_us,
                   watermark_slot)
    (OUTPUT,       epoch, outcome, source_us, slot, watermark_us,
                   watermark_slot, worker_iteration)
    (LIFECYCLE,    epoch, outcome, resulting_epoch, watermark_us,
                   watermark_slot)
    (SUBSCRIPTION, epoch, outcome, ruling)
    (VIOLATION,    epoch, reason,  detail)

OUTPUT's trailing ``worker_iteration`` is ``NavigationCommandWorker``'s own
loop count, NOT an autopilot slot index -- the worker skips a missed
deadline, so the two diverge. The commands those iterations issued live in
``determinism_command_log``, keyed the same way.

TRUTH's raw_sample is a v2-only atomic tail: (codec, status, raw_time_us,
source_system, source_component, original_frame_bytes). Its layout is defined
in determinism_truth_sample. V1 rows omit it and retain their original meaning.
It is diagnostic evidence, not a command input or proof of complete feeds.

LIFECYCLE is one row per leg boundary, ``activate()`` or ``close()``, at the
ENDING epoch and before the discards the boundary caused, whether or not
either slot held anything. ``resulting_epoch`` is the epoch the boundary made
current, read in the same transition, so neither epoch has to be inferred from
the rows around it. A dispatch that took its frame before the boundary and
settles after it is recorded at the ending epoch too.

A ``ruling`` is the router's number for one of its rulings on ATTITUDE
(``admission_tap``), read from the ATTITUDE ledger
(``determinism_admission_ledger``) before the row lock is taken. An
ASSOCIATION carries the latest ruling ADMITTED, a SUBSCRIPTION row the latest
OBSERVED, admitted or not; each is None while the ledger holds none. There is
one SUBSCRIPTION row once the source's message subscriptions are open and one
before they close. On the bus's one reader thread, every ruling strictly
between the two was made while the source was subscribed and dispatched
before it closed, so each one admitted is owed exactly one ASSOCIATION row,
carrying it, unless the source's callback raised, which the trace counts (D3
and D8.9 of the LANDING2 step-1 plan).

``EVENT_OUTCOMES`` and ``SLOT_PAIRS`` are the same layouts, as tables a reader
checks a row against: each event's outcomes, and where each layout keeps a
stamp and the slot made from it. The offline reader
(``determinism_trace_decode``, ``determinism_eligibility``) reads them here,
never from a copy of its own.
"""

from __future__ import annotations

from types import MappingProxyType


# Decision points on the direct-pixel path, in flow order.
# Trailing column indexes. STAGE and OUTPUT both end at index 7 carrying
# UNLIKE things -- a payload digest and a loop count -- so anything reading
# one must check the event first. Named here because this module is the
# layout contract, and an index spelled as a literal at the reader is how
# that check gets forgotten.
STAGE_PAYLOAD_DIGEST = 7
OUTPUT_WORKER_ITERATION = 7
# The same for the ruling an ASSOCIATION or a SUBSCRIPTION row names, and a
# VIOLATION's detail: (the stamp that stalled, the watermark it met).
ASSOCIATION_RULING = 5
SUBSCRIPTION_RULING = 3
VIOLATION_DETAIL = 3


EVENT_ASSOCIATION = "association"
EVENT_TRUTH = "truth"
EVENT_STAGE = "stage"
EVENT_DECIMATE = "decimate"
EVENT_OUTPUT = "output"
EVENT_LIFECYCLE = "lifecycle"
EVENT_SUBSCRIPTION = "subscription"
EVENT_VIOLATION = "violation"

# One ATTITUDE arrived and the associator ruled on it.
ASSOCIATION_COMMITTED = "committed"
ASSOCIATION_REFUSED = "refused"
ASSOCIATION_FENCED = "fenced"

# One SIM_STATE arrived.
TRUTH_RECORDED = "recorded"
TRUTH_UNREADABLE = "unreadable"

# A projected frame reached (or failed to reach) the publish slot.
STAGE_STAGED = "staged"
STAGE_FENCED = "fenced"
STAGE_INACTIVE = "inactive"
STAGE_PROJECTION_FAILED = "projection_failed"

# Why a sample was dropped before anything consumed it. The first two are
# newest-wins overwrites -- the coalescing this workstream removes. The third
# is a deliberate drop, kept distinct because it is not a host-timing effect.
DISCARD_PENDING_OVERWRITTEN = "pending_overwritten"
DISCARD_PUBLISH_OVERWRITTEN = "publish_overwritten"
DISCARD_UNBRACKETABLE = "unbracketable"
# A leg boundary emptied both slots. Not a timing effect either, but it must be
# recorded: W5 requires the scoring-window boundary to be VISIBLE, and without
# these rows a frame that activate() dropped simply vanishes from the ledger.
DISCARD_PENDING_LEG_ENDED = "pending_leg_ended"
DISCARD_PUBLISH_LEG_ENDED = "publish_leg_ended"

# One dispatch attempt = one logical output slot on this path.
OUTPUT_EMPTY = "empty"
OUTPUT_STALE_EPOCH = "stale_epoch"
OUTPUT_DISPATCHED = "dispatched"
OUTPUT_REJECTED = "rejected"
# The consumer RAISED. Distinct from a refusal: the frame still left the
# publish slot, and NavigationCommandWorker CATCHES the exception
# (navigation_command_worker.py:136-140), so without this row a crashing
# consumer erases frames from the ledger and from the tallies both.
OUTPUT_EXCEPTION = "exception"

# One leg boundary, activate() or close(). W5 needs the boundary VISIBLE, and
# the leg-ended discards alone show only a boundary that emptied a slot.
LIFECYCLE_ACTIVATED = "activated"
LIFECYCLE_CLOSED = "closed"

# The source's message subscriptions: open once start() has subscribed them,
# about to close in close(). The window its ATTITUDE rulings fall in (D3).
SUBSCRIPTION_OPENED = "opened"
SUBSCRIPTION_CLOSED = "closed"

# Preconditions W3 needs, which this landing can already observe.
VIOLATION_SOURCE_REGRESSED = "attitude_source_regressed"
VIOLATION_SOURCE_REPEATED = "attitude_source_repeated"

COUNTED_EVENTS = (
    EVENT_ASSOCIATION,
    EVENT_TRUTH,
    EVENT_STAGE,
    EVENT_DECIMATE,
    EVENT_OUTPUT,
    EVENT_LIFECYCLE,
    EVENT_SUBSCRIPTION,
    EVENT_VIOLATION,
)

# What index 2 may hold, by event: an outcome, or a DECIMATE's reason.
EVENT_OUTCOMES = MappingProxyType({
    EVENT_ASSOCIATION: (
        ASSOCIATION_COMMITTED, ASSOCIATION_REFUSED, ASSOCIATION_FENCED,
    ),
    EVENT_TRUTH: (TRUTH_RECORDED, TRUTH_UNREADABLE),
    EVENT_STAGE: (
        STAGE_STAGED, STAGE_FENCED, STAGE_INACTIVE, STAGE_PROJECTION_FAILED,
    ),
    EVENT_DECIMATE: (
        DISCARD_PENDING_OVERWRITTEN,
        DISCARD_PUBLISH_OVERWRITTEN,
        DISCARD_UNBRACKETABLE,
        DISCARD_PENDING_LEG_ENDED,
        DISCARD_PUBLISH_LEG_ENDED,
    ),
    EVENT_OUTPUT: (
        OUTPUT_EMPTY,
        OUTPUT_STALE_EPOCH,
        OUTPUT_DISPATCHED,
        OUTPUT_REJECTED,
        OUTPUT_EXCEPTION,
    ),
    EVENT_LIFECYCLE: (LIFECYCLE_ACTIVATED, LIFECYCLE_CLOSED),
    EVENT_SUBSCRIPTION: (SUBSCRIPTION_OPENED, SUBSCRIPTION_CLOSED),
    EVENT_VIOLATION: (VIOLATION_SOURCE_REGRESSED, VIOLATION_SOURCE_REPEATED),
})

# Where each layout keeps a microsecond stamp and the slot made from it, as
# (stamp index, slot index): the sample's own stamp first, then the
# watermark's, the take-time one on OUTPUT. A slot is its stamp // period_us,
# and None exactly when the stamp is (D8.3).
SLOT_PAIRS = MappingProxyType({
    EVENT_ASSOCIATION: ((3, 4),),
    EVENT_TRUTH: ((3, 4),),
    EVENT_STAGE: ((3, 4), (5, 6)),
    EVENT_DECIMATE: ((3, 4), (5, 6)),
    EVENT_OUTPUT: ((3, 4), (5, 6)),
    EVENT_LIFECYCLE: ((4, 5),),
    EVENT_SUBSCRIPTION: (),
    EVENT_VIOLATION: (),
})


__all__ = [
    "ASSOCIATION_COMMITTED",
    "ASSOCIATION_FENCED",
    "ASSOCIATION_REFUSED",
    "ASSOCIATION_RULING",
    "COUNTED_EVENTS",
    "DISCARD_PENDING_LEG_ENDED",
    "DISCARD_PENDING_OVERWRITTEN",
    "DISCARD_PUBLISH_LEG_ENDED",
    "DISCARD_PUBLISH_OVERWRITTEN",
    "DISCARD_UNBRACKETABLE",
    "EVENT_ASSOCIATION",
    "EVENT_DECIMATE",
    "EVENT_LIFECYCLE",
    "EVENT_OUTCOMES",
    "EVENT_OUTPUT",
    "EVENT_STAGE",
    "EVENT_SUBSCRIPTION",
    "EVENT_TRUTH",
    "EVENT_VIOLATION",
    "LIFECYCLE_ACTIVATED",
    "LIFECYCLE_CLOSED",
    "OUTPUT_DISPATCHED",
    "OUTPUT_EMPTY",
    "OUTPUT_EXCEPTION",
    "OUTPUT_REJECTED",
    "OUTPUT_STALE_EPOCH",
    "OUTPUT_WORKER_ITERATION",
    "SLOT_PAIRS",
    "STAGE_FENCED",
    "STAGE_INACTIVE",
    "STAGE_PAYLOAD_DIGEST",
    "STAGE_PROJECTION_FAILED",
    "STAGE_STAGED",
    "SUBSCRIPTION_CLOSED",
    "SUBSCRIPTION_OPENED",
    "SUBSCRIPTION_RULING",
    "TRUTH_RECORDED",
    "TRUTH_UNREADABLE",
    "VIOLATION_DETAIL",
    "VIOLATION_SOURCE_REGRESSED",
    "VIOLATION_SOURCE_REPEATED",
]
