from pathlib import Path


PACKAGE = Path("src/navpy/modules/navigation/nav/vision_nav")


def test_final_approach_command_core_has_no_prediction_or_gap_modules():
    assert not (PACKAGE / "prediction.py").exists()
    assert not (PACKAGE / "runtime_support.py").exists()
    assert not (PACKAGE / "command_pipeline.py").exists()
