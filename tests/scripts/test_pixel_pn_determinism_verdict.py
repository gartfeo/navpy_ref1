"""The verdict, from a case's files as the child's teardown leaves them.

Through the real writer (``pixel_pn_determinism_evidence``) and, for the
ledger's evidence, the real admission link over a real tap: the files a
traced leg leaves are read back and judged by D8, and the command line prints
that verdict and exits by it. That the ledger subscribed before the source
started (D8.5) is the link's own evidence, so here it is the link that
reports it, not a figure a test wrote.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from navpy.modules.vehicle.admission_tap import AdmissionTap
from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_eligibility import (
    ELIGIBLE,
    INSPECTABLE,
    case_verdict,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
)
from navpy.modules.vision.sim.determinism_evidence import (
    MANIFEST_NAME,
    SOURCE_CLOSE_STEP,
    TRACE_NAME,
)
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from scripts.pixel_pn_admission_ledger import subscribe_admission_ledger
from scripts.pixel_pn_determinism_evidence import write_determinism_evidence
from scripts.pixel_pn_determinism_verdict import (
    ELIGIBLE_EXIT,
    INSPECTABLE_EXIT,
    main,
)
from tests.modules.vision.determinism_case_factory import (
    IDENTITY,
    PERIOD_US,
    stamp_ms,
)
from tests.modules.vision.truth_packet_factory import truth_packet


def _leg(
    directory: Path,
    *,
    late: bool = False,
    identity: Any = IDENTITY,
    **finalization: Any,
) -> None:
    """One traced leg over a real tap, written as the teardown writes it.

    ``late`` opens the source's window BEFORE the ledger subscribes, the
    order D8.5 refuses. ``finalization`` replaces the teardown's fields.
    """
    trace = DeterminismTrace(PERIOD_US)
    tap = AdmissionTap(lambda message: message.time_boot_ms, ("ATTITUDE",))
    vehicle = SimpleNamespace(on_admission=tap.subscribe)
    if late:
        trace.record_subscription(epoch=0, outcome=SUBSCRIPTION_OPENED)
    link = subscribe_admission_ledger(vehicle, trace)
    if not late:
        trace.record_subscription(epoch=0, outcome=SUBSCRIPTION_OPENED)
    trace.record_lifecycle(
        epoch=0, outcome=LIFECYCLE_ACTIVATED, resulting_epoch=1
    )
    PassObserver(trace.ledger, trace.command_log).note_iteration(1)
    for number in (1, 2):
        stamp = stamp_ms(number)
        tap.rule("ATTITUDE", SimpleNamespace(time_boot_ms=stamp), True, None)
        trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=stamp / 1000
        )
    # A v2 recording must also contain an original source-stamped truth packet.
    trace.record_truth(epoch=1, outcome="recorded", message=truth_packet())
    trace.record_subscription(epoch=1, outcome=SUBSCRIPTION_CLOSED)
    trace.record_lifecycle(epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2)
    link.cancel()
    write_determinism_evidence(
        directory,
        trace,
        epoch=1,
        finalization={
            "worker": "stopped",
            "mavlink_quiescence": "unverified",
            "teardown_errors": [],
            "admission": link.evidence(),
            **finalization,
        },
        identity=identity,
    )


def test_a_leg_the_teardown_wrote_is_eligible(tmp_path: Path) -> None:
    _leg(tmp_path)
    verdict = case_verdict(tmp_path)
    assert verdict.status == ELIGIBLE, verdict.reasons


def test_a_ledger_subscribed_once_the_source_had_started_is_not(
    tmp_path: Path,
) -> None:
    """The link counted the window row already in the journal."""
    _leg(tmp_path, late=True)
    manifest = json.loads((tmp_path / MANIFEST_NAME).read_text("utf-8"))
    assert manifest["finalization"]["admission"]["window_rows"] == 1
    verdict = case_verdict(tmp_path)
    assert (verdict.status, verdict.failed_rules) == (INSPECTABLE, (5,))


def test_a_failed_close_is_read_from_the_files(tmp_path: Path) -> None:
    """Complete stores and a confirmed stop, but source.close raised."""
    _leg(
        tmp_path,
        teardown_errors=[
            {"step": SOURCE_CLOSE_STEP, "error": "RuntimeError('close')"}
        ],
    )
    assert case_verdict(tmp_path).failed_rules == (12,)


def test_a_case_with_no_definition_hash_is_not_eligible(tmp_path: Path) -> None:
    """Delivery step 6's review: D8.13 refuses a case with no definition."""
    _leg(
        tmp_path,
        identity={**IDENTITY, "case": {"name": "c", "definition_sha256": None}},
    )
    assert case_verdict(tmp_path).failed_rules == (13,)


def test_a_case_whose_manifest_is_missing_is_refused(tmp_path: Path) -> None:
    _leg(tmp_path)
    (tmp_path / MANIFEST_NAME).unlink()
    verdict = case_verdict(tmp_path)
    assert (verdict.status, verdict.failed_rules) == (INSPECTABLE, (1,))
    assert "missing" in verdict.reasons[0].text


def test_a_truncated_trace_file_is_refused(tmp_path: Path) -> None:
    _leg(tmp_path)
    trace = tmp_path / TRACE_NAME
    trace.write_bytes(trace.read_bytes()[:-1])
    assert case_verdict(tmp_path).failed_rules == (1,)


def test_the_command_line_prints_the_verdict_and_exits_by_it(
    tmp_path: Path, capsys
) -> None:
    eligible, late = tmp_path / "eligible", tmp_path / "late"
    eligible.mkdir()
    late.mkdir()
    _leg(eligible)
    _leg(late, late=True)

    assert main([str(eligible)]) == ELIGIBLE_EXIT
    printed = json.loads(capsys.readouterr().out)
    assert printed == {"status": ELIGIBLE, "failed_rules": [], "reasons": []}

    assert main([str(late)]) == INSPECTABLE_EXIT
    printed = json.loads(capsys.readouterr().out)
    assert (printed["status"], printed["failed_rules"]) == (INSPECTABLE, [5])
    assert [reason["rule"] for reason in printed["reasons"]] == [5]
