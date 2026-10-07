"""Tests for vision CLI arguments."""

from navpy.args.navpy_argparse import make_parser


def test_detector_debug_show_defaults_false():
    parser = make_parser(description="test")

    args = parser.parse_args(["-ss", "1"])

    assert args.detector_debug_show is False


def test_detector_debug_show_flag_sets_true():
    parser = make_parser(description="test")

    args = parser.parse_args(["-ss", "1", "--detector-debug-show"])

    assert args.detector_debug_show is True


def test_real_detector_defaults_to_charuco_without_a_model_path():
    """The default real backend is the ChArUco dock board, never a bench model."""
    from navpy.args.vision_args import VisionArgs

    args = make_parser(description="test").parse_args(["-ss", "1"])
    vision_args = VisionArgs.from_args(args)

    assert args.detector_backend == "charuco"
    assert args.detector_model_path == ""
    assert vision_args.detector_backend == "charuco"
    assert vision_args.model_path == ""


def test_yolo_backend_is_explicit_opt_in_with_model_path():
    from navpy.args.vision_args import VisionArgs

    args = make_parser(description="test").parse_args([
        "-ss", "1",
        "--detector-type", "real",
        "--detector-backend", "yolo",
        "--detector-model-path", ".models/bench.pt",
    ])
    vision_args = VisionArgs.from_args(args)

    assert vision_args.detector_type == "real"
    assert vision_args.detector_backend == "yolo"
    assert vision_args.model_path == ".models/bench.pt"


def test_unknown_detector_backend_is_rejected():
    import pytest

    with pytest.raises(SystemExit):
        make_parser(description="test").parse_args(
            ["-ss", "1", "--detector-backend", "face"]
        )
