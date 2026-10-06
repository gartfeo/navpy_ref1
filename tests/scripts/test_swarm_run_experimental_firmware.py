"""Experimental firmware stays behind explicit, safely quoted launch paths."""

from pathlib import Path
import shlex
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import swarm_run_wsl as wsl  # noqa: E402
from swarm_run_cli import build_parser  # noqa: E402


def command(**kwargs):
    return wsl.launch_command(1, 120, "16590", 1, 0, **kwargs)


def test_default_launch_is_unchanged():
    assert "ARDUPILOT_DIR=" not in command()
    assert "BIN=" not in command()


def test_shell_metacharacters_stay_in_single_arguments():
    root = "/home/gart/fork ' $(false); echo bad"
    binary = root + "/probe"
    words = shlex.split(command(firmware_root=root, binary_path=binary))
    assert f"ARDUPILOT_DIR={root}" in words
    assert f"BIN={binary}" in words
    assert "echo" not in words


@pytest.mark.parametrize("options", [
    {"binary_path": "/tmp/a"}, {"firmware_root": "relative"},
    {"firmware_root": ""}, {"firmware_root": "/tmp/x\x00y"},
])
def test_invalid_paths_are_rejected(options):
    with pytest.raises(ValueError):
        command(**options)


def test_parser_and_default_binary():
    args = build_parser(10).parse_args(["--eval", "--sitl-root", "/tmp/probe"])
    assert args.sitl_root == "/tmp/probe"
    assert "BIN=/tmp/probe/build/sitl/bin/arduplane" in command(firmware_root=args.sitl_root)


def test_preflight_does_not_invoke_wsl_for_default():
    with patch.object(wsl.subprocess, "run") as run:
        wsl.ensure_experimental_paths(None, None)
    run.assert_not_called()


def test_invalid_preflight_exits_before_wsl():
    with patch.object(wsl.subprocess, "run") as run:
        with pytest.raises(SystemExit, match="requires an isolated firmware root"):
            wsl.ensure_experimental_paths(None, "/tmp/binary")
    run.assert_not_called()
