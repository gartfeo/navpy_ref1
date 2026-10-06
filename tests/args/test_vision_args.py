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
