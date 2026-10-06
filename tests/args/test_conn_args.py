"""Tests for vehicle connection CLI arguments."""

import pytest

from navpy.args.conn_args import ConnArgs
from navpy.args.navpy_argparse import make_parser


def test_source_system_is_the_single_node_identity():
    """The companion shares its aircraft's sysid (it is told apart by
    component id 191), so ``-ss`` is the one and only identity argument."""
    parser = make_parser(description="test")

    args = parser.parse_args(["-ss", "7"])
    conn_args = ConnArgs(args)

    assert conn_args.source_system == 7
    assert not hasattr(conn_args, "mav_source_system")


def test_mav_source_system_flag_is_gone():
    """The fake-sysid override was removed; passing it must be a hard argparse
    error, not a silently ignored flag that leaves a companion on a stale id."""
    parser = make_parser(description="test")

    with pytest.raises(SystemExit):
        parser.parse_args(["-ss", "7", "--mav-source-system", "107"])
