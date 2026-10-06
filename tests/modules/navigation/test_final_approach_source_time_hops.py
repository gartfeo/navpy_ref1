from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)


def _frame(source, generation, timestamp):
    return FinalApproachVisionFrame(
        source, generation, 1, 2, timestamp,
        1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    )


def test_phase_activity_never_remaps_or_restarts_raw_source_time():
    ledger = SourceEpochLedger()
    assert ledger.admit(_frame("cam", 0, 100.0)) is FrameAdmission.FRESH
    assert ledger.admit(_frame("cam", 0, 1.0)) is FrameAdmission.REGRESSION


def test_explicit_named_discontinuity_is_only_way_to_accept_clock_restart():
    ledger = SourceEpochLedger()
    ledger.admit(_frame("cam-a", 0, 100.0))
    ledger.admit(_frame("cam-b", 0, 100.0))
    ledger.note_discontinuity("cam-a")

    assert ledger.admit(_frame("cam-a", 1, 1.0)) is FrameAdmission.FRESH
    assert ledger.admit(_frame("cam-b", 0, 1.0)) is FrameAdmission.REGRESSION
