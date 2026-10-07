"""Bench runners must resolve default YOLO models inside the repo's .models/."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def test_gimbal_tracker_default_model_is_in_repo_models_dir():
    import sys
    sys.path.insert(0, str(REPO / "scripts" / "python"))
    try:
        from run_gimbal_tracker_assembly import resolve_model_path
    finally:
        sys.path.pop(0)
    assert Path(resolve_model_path(None, "yolov8s.pt")) == (REPO / ".models" / "yolov8s.pt").resolve()


def test_gimbal_tuning_default_model_is_in_repo_models_dir():
    import sys
    sys.path.insert(0, str(REPO))
    try:
        from tools.cam.gimbal_tuning_assembly import default_model_path
    finally:
        sys.path.pop(0)
    assert Path(default_model_path()).parent == (REPO / ".models").resolve()
