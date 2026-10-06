"""Regression tests for the fully composed NavPy command-line parser."""

from navpy.args.navpy_argparse import make_parser


def test_full_parser_help_renders_literal_percent_sign() -> None:
    help_text = make_parser(description="test").format_help()

    assert "vehicle pitch -10%" in help_text
