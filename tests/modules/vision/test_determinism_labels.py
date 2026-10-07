"""The clock labels D8's table gives each record of a case.

A label places a record on the autopilot's slot grid, or says why it cannot:
UNKNOWN, or NOT_APPLICABLE for the frame of an OUTPUT that took none. A
refused association is placed by the ledger's stamp for the ruling it names,
the ATTITUDE it acted on; a pass by its admitted cutoff's stamp, a lower
bound; an OUTPUT's pass through its iteration. In a case with a discontinuity
or a stampless admission every clock label is UNKNOWN. The table is D8 of
the LANDING2 step-1 plan.
"""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_REFUSED,
    DISCARD_PENDING_LEG_ENDED,
    EVENT_VIOLATION,
    OUTPUT_DISPATCHED,
    STAGE_PROJECTION_FAILED,
    STAGE_STAGED,
    VIOLATION_SOURCE_REPEATED,
)
from navpy.modules.vision.sim.determinism_labels import (
    NOT_APPLICABLE,
    UNKNOWN,
    case_labels,
)
from navpy.modules.vision.sim.determinism_row_layouts import (
    association_row,
    discard_row,
    output_row,
    stage_row,
)
from navpy.modules.vision.sim.determinism_trace import TraceCapture
from tests.modules.vision.determinism_case_factory import (
    PERIOD_US,
    eligible_capture,
    with_rows,
)

FRAME = SimpleNamespace(
    pixel=SimpleNamespace(
        u_px=1.5, v_px=2.5, aircraft_pitch_deg=-4.0,
        aircraft_roll_deg=3.0, source_timestamp_s=10.02,
    )
)
# ``eligible_trace``'s rows, labelled: 10_020, 10_040 and 10_060 ms are
# slots 501, 502 and 503 at 20 ms.
BASELINE = (
    (),  # OPENED: no label, it bounds coverage
    (("watermark_slot", UNKNOWN),),  # ACTIVATED, before any ATTITUDE
    (("attitude_slot", 501),),
    (("attitude_slot", 502),),
    (("watermark_slot", 502),),  # TRUTH
    (  # OUTPUT, EMPTY, in pass 2, whose admitted cutoff is ruling 2
        ("frame_slot", NOT_APPLICABLE),
        ("taken_slot", 502),
        ("pass_slot", 502),
    ),
    (("attitude_slot", 503),),
    (),  # CLOSED
    (("watermark_slot", 503),),  # the leg's end
)


def test_every_record_of_an_eligible_case_is_labelled_by_the_table() -> None:
    labels = case_labels(eligible_capture())
    assert labels.rows == BASELINE
    # Pass 1 began before any admission; pass 2 after ruling 2's.
    assert labels.passes == (
        (("admitted_slot", UNKNOWN),),
        (("admitted_slot", 502),),
    )


def _added(*rows: tuple) -> tuple:
    capture = eligible_capture()
    return case_labels(with_rows(capture, capture.rows + rows)).rows[
        len(capture.rows):
    ]


def test_a_refused_association_is_placed_by_the_ruling_it_names() -> None:
    """It carries no stamp of its own: the ledger's stamp for its ruling,
    10_040 ms, is the ATTITUDE it acted on."""
    refused = association_row(1, ASSOCIATION_REFUSED, None, PERIOD_US, 2)
    assert _added(refused) == ((("attitude_slot", 502),),)


def test_a_refusal_naming_no_admitted_ruling_is_unknown() -> None:
    refused = association_row(1, ASSOCIATION_REFUSED, None, PERIOD_US, 9)
    assert _added(refused) == ((("attitude_slot", UNKNOWN),),)


def test_frames_victims_and_outputs_carry_their_slots() -> None:
    rows = (
        stage_row(1, STAGE_STAGED, 10_020_000, 10_040_000, FRAME, PERIOD_US),
        stage_row(1, STAGE_PROJECTION_FAILED, None, None, None, PERIOD_US),
        discard_row(1, DISCARD_PENDING_LEG_ENDED, 10_020_000, None, PERIOD_US),
        output_row(1, OUTPUT_DISPATCHED, 10_020_000, 10_040_000, 1, PERIOD_US),
        output_row(1, OUTPUT_DISPATCHED, 10_020_000, 10_040_000, None, PERIOD_US),
    )
    assert _added(*rows) == (
        (("frame_slot", 501), ("watermark_slot", 502)),
        # No frame and no sample: nothing to place.
        (("frame_slot", UNKNOWN), ("watermark_slot", UNKNOWN)),
        (("victim_slot", 501), ("watermark_slot", UNKNOWN)),
        # Pass 1 began before any admission, so its label is unknown.
        (("frame_slot", 501), ("taken_slot", 502), ("pass_slot", UNKNOWN)),
        # No iteration: the pass is unknown.
        (("frame_slot", 501), ("taken_slot", 502), ("pass_slot", UNKNOWN)),
    )


def test_a_violation_is_labelled_by_its_own_detail() -> None:
    violation = (
        EVENT_VIOLATION, 1, VIOLATION_SOURCE_REPEATED,
        (10_040_000, 10_040_000),
    )
    assert _added(violation) == ((("detail", (10_040_000, 10_040_000)),),)


def _every_label_unknown(labels: tuple) -> bool:
    return all(
        value in (UNKNOWN, NOT_APPLICABLE)
        for record in labels
        for _, value in record
    )


def test_a_discontinuity_makes_every_clock_label_unknown() -> None:
    capture = eligible_capture()
    stepped_back = replace(
        capture.ledger,
        entries=capture.ledger.entries + ((4, 10_000, True, None),),
    )
    labels = case_labels(replace(capture, ledger=stepped_back))
    assert labels.rows != BASELINE
    assert _every_label_unknown(labels.rows)
    assert _every_label_unknown(labels.passes)
    # The frame an OUTPUT never took is still not applicable.
    assert labels.rows[5][0] == ("frame_slot", NOT_APPLICABLE)


def test_a_stampless_admission_makes_every_clock_label_unknown() -> None:
    capture = eligible_capture()
    stampless = replace(
        capture.ledger,
        entries=capture.ledger.entries + ((4, None, True, None),),
    )
    labels = case_labels(replace(capture, ledger=stampless))
    assert _every_label_unknown(labels.rows)
    assert _every_label_unknown(labels.passes)


# Ruling 4, REJECTED, after the three the eligible case admits.
REJECTED = (4, 10_070, False, "stale_boot")


def _rejection_added() -> TraceCapture:
    capture = eligible_capture()
    return replace(capture, ledger=replace(
        capture.ledger, entries=capture.ledger.entries + (REJECTED,)
    ))


def test_a_refusal_naming_a_rejected_ruling_is_unknown() -> None:
    """A rejected ATTITUDE never reached the source: its stamp places
    nothing."""
    refused = association_row(1, ASSOCIATION_REFUSED, None, PERIOD_US, 4)
    capture = _rejection_added()
    labels = case_labels(with_rows(capture, capture.rows + (refused,)))
    assert labels.rows[-1] == (("attitude_slot", UNKNOWN),)


def test_a_pass_is_placed_by_its_admitted_cutoff_not_its_observed_one() -> None:
    """Pass 3 observed the rejected ruling 4; the last ATTITUDE it could
    have been handed is ruling 3's, 10_060 ms, slot 503."""
    capture = _rejection_added()
    capture = replace(capture, commands=replace(
        capture.commands,
        passes=capture.commands.passes + ((3, 4, 3),),
        iterations=3,
    ))
    assert case_labels(capture).passes[-1] == (("admitted_slot", 503),)


def test_a_violation_keeps_its_own_detail_where_clock_labels_are_unknown(
) -> None:
    """D8's table: a VIOLATION is labelled by its own detail, never
    unknown. A step back or a stampless admission makes every CLOCK label
    unknown, and the detail is the row's, not a clock label (review
    round 1)."""
    detail = (10_040_000, 10_040_000)
    violation = (EVENT_VIOLATION, 1, VIOLATION_SOURCE_REPEATED, detail)
    capture = eligible_capture()
    for entry in ((4, 10_000, True, None), (4, None, True, None)):
        ledger = replace(
            capture.ledger, entries=capture.ledger.entries + (entry,)
        )
        labels = case_labels(
            with_rows(replace(capture, ledger=ledger), capture.rows + (violation,))
        ).rows
        assert labels[2] == (("attitude_slot", UNKNOWN),), entry
        assert labels[-1] == (("detail", detail),), entry
