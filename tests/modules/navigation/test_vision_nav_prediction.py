"""The final-approach command core keeps no prediction or gap modules."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
# One full path per vision_nav file; the folders holding them are the command
# core's homes, so the check follows the files wherever they move.
VISION_NAV_FILES = (
    "src/navpy/plugins/laws/vision_pn/command_anchor.py",
    "src/navpy/core/approach/vision_nav/command_executor.py",
    "src/navpy/core/approach/vision_nav/command_freshness.py",
    "src/navpy/core/approach/vision_nav/command_hold.py",
    "src/navpy/core/approach/vision_nav/command_liveness.py",
    "src/navpy/core/approach/vision_nav/command_postprocess.py",
    "src/navpy/core/approach/vision_nav/command_reset.py",
    "src/navpy/core/approach/vision_nav/command_transaction.py",
    "src/navpy/core/approach/vision_nav/confirmation.py",
    "src/navpy/core/approach/vision_nav/diagnostic_capture.py",
    "src/navpy/core/approach/vision_nav/diagnostic_mailbox.py",
    "src/navpy/core/approach/vision_nav/diagnostics.py",
    "src/navpy/core/ports/frame.py",
    "src/navpy/core/approach/vision_nav/frame_projection.py",
    "src/navpy/core/approach/vision_nav/ingress.py",
    "src/navpy/plugins/laws/vision_pn/lateral_rate.py",
    "src/navpy/plugins/laws/vision_pn/law.py",
    "src/navpy/core/ports/law_config.py",
    "src/navpy/plugins/laws/vision_pn/law_plan.py",
    "src/navpy/plugins/laws/vision_pn/pitch_law.py",
    "src/navpy/core/approach/vision_nav/queued_frame.py",
    "src/navpy/plugins/laws/vision_pn/rate_filter.py",
    "src/navpy/core/approach/vision_nav/runtime.py",
    "src/navpy/core/approach/vision_nav/runtime_api.py",
    "src/navpy/core/approach/vision_nav/runtime_composition.py",
    "src/navpy/core/approach/vision_nav/runtime_state.py",
    "src/navpy/core/approach/vision_nav/source_epoch.py",
    "src/navpy/core/approach/vision_nav/source_time_adapter.py",
    "src/navpy/core/approach/vision_nav/source_time_ports.py",
    "src/navpy/core/approach/vision_nav/visual_pass.py",
)
GAP_MODULES = ("prediction.py", "runtime_support.py", "command_pipeline.py")


def test_final_approach_command_core_has_no_prediction_or_gap_modules():
    missing = [relative for relative in VISION_NAV_FILES
               if not (ROOT / relative).is_file()]
    assert not missing, missing
    homes = {(ROOT / relative).parent for relative in VISION_NAV_FILES}
    for home in sorted(homes):
        for name in GAP_MODULES:
            assert not (home / name).exists(), home / name
