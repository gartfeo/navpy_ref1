from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)


def _frame(source="a", generation=0, task=1, obj=2, timestamp=1.0):
    return TerminalVisionFrame(
        source, generation, task, obj, timestamp,
        1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    )


def test_duplicate_and_regression_are_not_admitted():
    ledger = SourceEpochLedger()
    assert ledger.admit(_frame(timestamp=10.0)) is FrameAdmission.FRESH
    assert ledger.admit(_frame(timestamp=10.0)) is FrameAdmission.DUPLICATE
    assert ledger.admit(_frame(timestamp=9.0)) is FrameAdmission.REGRESSION
    assert ledger.admit(_frame(timestamp=11.0)) is FrameAdmission.FRESH


def test_continuity_is_independent_per_source_generation_task_and_object():
    ledger = SourceEpochLedger()
    ledger.admit(_frame(timestamp=10.0))
    variants = [
        _frame(source="b", timestamp=1.0),
        _frame(generation=1, timestamp=1.0),
        _frame(task=3, timestamp=1.0),
        _frame(obj=4, timestamp=1.0),
    ]
    assert all(ledger.admit(frame) is FrameAdmission.FRESH for frame in variants)


def test_named_discontinuity_bumps_only_that_source():
    ledger = SourceEpochLedger()
    assert ledger.generation("a") == 0
    assert ledger.generation("b") == 0
    ledger.note_discontinuity("a")
    assert ledger.generation("a") == 1
    assert ledger.generation("b") == 0
