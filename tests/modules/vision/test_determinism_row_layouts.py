"""The row layouts, pinned field by field.

``determinism_events`` documents each layout, and these builders are the only
place a row is assembled, so a field that moves here moves in every trace.
"""

from __future__ import annotations

import struct
from types import SimpleNamespace

import pytest
from navpy.modules.vision.sim.determinism_truth_sample import UNAVAILABLE

from navpy.modules.vision.sim.determinism_events import (
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    OUTPUT_WORKER_ITERATION,
    STAGE_PAYLOAD_DIGEST,
)
from navpy.modules.vision.sim.determinism_row_layouts import (
    association_row,
    discard_row,
    lifecycle_row,
    output_row,
    stage_row,
    subscription_row,
    truth_row,
)

PERIOD_US = 20_000
# Three stamps in three different slots, so a field that swaps with another
# changes the row.
SOURCE_US, WATERMARK_US, TAKEN_US = 1_250_000, 1_210_000, 1_230_000
SOURCE_SLOT, WATERMARK_SLOT, TAKEN_SLOT = 62, 60, 61


class _Outcome(str):
    """A str subclass: the layouts must still store a plain str."""


def _frame(source_s: float) -> SimpleNamespace:
    pixel = SimpleNamespace(
        u_px=1.0,
        v_px=2.0,
        aircraft_pitch_deg=3.0,
        aircraft_roll_deg=4.0,
        source_timestamp_s=source_s,
    )
    return SimpleNamespace(pixel=pixel)


def test_each_layout_puts_every_field_where_the_vocabulary_says() -> None:
    digest = struct.pack("<ddddd", 1.0, 2.0, 3.0, 4.0, 1.25)
    rows = {
        EVENT_ASSOCIATION: association_row(
            3.0, _Outcome("committed"), SOURCE_US, PERIOD_US, 9.0
        ),
        EVENT_TRUTH: truth_row(
            3.0, _Outcome("recorded"), WATERMARK_US, PERIOD_US
        ),
        EVENT_STAGE: stage_row(
            3.0,
            _Outcome("staged"),
            SOURCE_US,
            WATERMARK_US,
            _frame(1.25),
            PERIOD_US,
        ),
        EVENT_DECIMATE: discard_row(
            3.0, _Outcome("unbracketable"), SOURCE_US, WATERMARK_US, PERIOD_US
        ),
        EVENT_OUTPUT: output_row(
            3.0, _Outcome("delivered"), SOURCE_US, TAKEN_US, 7, PERIOD_US
        ),
        EVENT_LIFECYCLE: lifecycle_row(
            3.0, _Outcome("closed"), 4.0, WATERMARK_US, PERIOD_US
        ),
        EVENT_SUBSCRIPTION: subscription_row(3.0, _Outcome("opened"), 9.0),
    }

    assert rows == {
        EVENT_ASSOCIATION: (
            EVENT_ASSOCIATION, 3, "committed", SOURCE_US, SOURCE_SLOT, 9,
        ),
        EVENT_TRUTH: (
            EVENT_TRUTH, 3, "recorded", WATERMARK_US, WATERMARK_SLOT, UNAVAILABLE,
        ),
        EVENT_STAGE: (
            EVENT_STAGE, 3, "staged", SOURCE_US, SOURCE_SLOT,
            WATERMARK_US, WATERMARK_SLOT, digest,
        ),
        EVENT_DECIMATE: (
            EVENT_DECIMATE, 3, "unbracketable", SOURCE_US, SOURCE_SLOT,
            WATERMARK_US, WATERMARK_SLOT,
        ),
        EVENT_OUTPUT: (
            EVENT_OUTPUT, 3, "delivered", SOURCE_US, SOURCE_SLOT,
            TAKEN_US, TAKEN_SLOT, 7,
        ),
        EVENT_LIFECYCLE: (
            EVENT_LIFECYCLE, 3, "closed", 4, WATERMARK_US, WATERMARK_SLOT,
        ),
        EVENT_SUBSCRIPTION: (EVENT_SUBSCRIPTION, 3, "opened", 9),
    }
    # The epoch and the outcome are converted, not stored as handed in: a 3.0
    # equals 3 in a tuple comparison, so the types are checked on their own.
    for row in rows.values():
        assert type(row[1]) is int and type(row[2]) is str, row
    # So is the epoch a boundary made current.
    assert type(rows[EVENT_LIFECYCLE][3]) is int
    # And so is a ruling: the ledger's numbers are ints, and one a reader
    # joins on must not arrive as a float that merely compares equal.
    assert type(rows[EVENT_ASSOCIATION][5]) is int
    assert type(rows[EVENT_SUBSCRIPTION][3]) is int
    # The named trailing columns index the layouts they name.
    assert rows[EVENT_STAGE][STAGE_PAYLOAD_DIGEST] == digest
    assert rows[EVENT_OUTPUT][OUTPUT_WORKER_ITERATION] == 7


def test_a_ruling_the_ledger_did_not_have_stays_none() -> None:
    """None means the ledger held no ruling of that kind yet, which is not
    ruling 0: the router numbers its first ruling 1, so a 0 here would name a
    ruling that was never made."""
    assert association_row(3, "refused", None, PERIOD_US, None)[5] is None
    assert subscription_row(3, "opened", None)[3] is None


def test_the_payload_is_read_last_so_a_bad_field_never_reaches_the_frame() -> (
    None
):
    """What the module docstring claims: fields are evaluated left to right,
    as the recorders always did, and the payload digest comes last."""
    touched: list[str] = []

    class _Watched:
        @property
        def pixel(self) -> None:
            touched.append("pixel")
            return None

    with pytest.raises(TypeError):
        stage_row(object(), "staged", SOURCE_US, WATERMARK_US, _Watched(), PERIOD_US)
    assert touched == [], "the frame was read before the epoch had converted"


def test_no_layout_invents_a_slot_without_a_period() -> None:
    """Period 0 means no grid: every slot None, every stamp kept.

    Pinned per builder, with every stamp KNOWN. The source-level test of this
    rule read index 4 of every row, which is a LIFECYCLE row's watermark stamp
    rather than its slot, so a lifecycle builder inventing slot 0 passed every
    test (a review, on Landing 2 delivery step 2). A SUBSCRIPTION row has no
    stamp and no slot, so it has nothing to invent.
    """
    rows = [
        association_row(3, "committed", SOURCE_US, 0, 9),
        truth_row(3, "recorded", WATERMARK_US, 0),
        stage_row(3, "staged", SOURCE_US, WATERMARK_US, None, 0),
        discard_row(3, "unbracketable", SOURCE_US, WATERMARK_US, 0),
        output_row(3, "delivered", SOURCE_US, TAKEN_US, 7, 0),
        lifecycle_row(3, "closed", 4, WATERMARK_US, 0),
        subscription_row(3, "opened", 9),
    ]

    assert [row[3:] for row in rows] == [
        (SOURCE_US, None, 9),
        (WATERMARK_US, None, UNAVAILABLE),
        (SOURCE_US, None, WATERMARK_US, None, None),
        (SOURCE_US, None, WATERMARK_US, None),
        (SOURCE_US, None, TAKEN_US, None, 7),
        (4, WATERMARK_US, None),
        (9,),
    ]
