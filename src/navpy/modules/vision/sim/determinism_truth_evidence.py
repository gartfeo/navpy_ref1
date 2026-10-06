"""Additional v2 capture checks, separate from full selector-feed certification."""

from __future__ import annotations

from typing import TYPE_CHECKING

from navpy.modules.vision.sim.determinism_events import EVENT_TRUTH

if TYPE_CHECKING:
    from navpy.modules.vision.sim.determinism_reader import CaseEvidence


def raw_truth_reasons(case: CaseEvidence) -> list[str]:
    """14. Every v2 truth event has a usable original stamped frame.

    V1 retains its original recording-only verdict. Even a passing v2 capture
    proves neither complete transport delivery nor full selector inputs: airspeed,
    ATTITUDE payloads, boot identity and scoring-window boundaries remain separate.
    Source repeats/regressions are retained here, not silently reordered.
    """
    if case.manifest["version"] == 1:
        return []
    rows = [(i, row) for i, row in enumerate(case.capture.rows) if row[0] == EVENT_TRUTH]
    if not rows:
        return ["no original SIM_STATE observations captured"]
    return [
        f"TRUTH row {index}: original sample {row[5][1]}"
        for index, row in rows if row[5][1] != "captured"
    ]
